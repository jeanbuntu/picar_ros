"""
tf_bridge.py -- Ubuntu ROS2 node: Pi WebSocket → TF + MarkerArray trajectory visualization.

Connects to the Pi's dashboard WebSocket, receives tag_detections messages,
computes world-frame transforms, and publishes TF + visualization_msgs/MarkerArray
topics for RViz2.

World frame is established from the midpoint of the tag_world_a / tag_world_b pair
(default IDs 2 and 3).  Only frames where the pair passes the geometric consistency
check (pair_valid=True) update the camera TF and filtered trajectories.  Individual
tag detections are always recorded in raw trajectories using the last valid camera TF.

Build:
    cd ~/ros2_ws/src && ln -s ~/picar_ros/ros2/tf_bridge .
    cd ~/ros2_ws && colcon build --packages-select tf_bridge
    source install/setup.bash

Run:
    ros2 run tf_bridge tf_bridge
    ros2 run tf_bridge tf_bridge --ros-args -p pi_ws_url:=ws://192.168.1.241:8000/ws
"""

import asyncio
import json
import logging
import os
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import math
from sensor_msgs.msg import JointState

try:
    import websockets
except ImportError as e:
    raise ImportError("websockets not installed: pip3 install websockets") from e

_CYCLE_COLORS = [
    (0.0, 0.85, 0.0, 1.0),   # green
    (0.85, 0.0, 0.0, 1.0),   # red
    (0.15, 0.35, 0.95, 1.0), # blue
    (1.0, 0.5, 0.0, 1.0),    # orange
    (0.8, 0.0, 0.8, 1.0),    # magenta
    (0.0, 0.8, 0.8, 1.0),    # cyan
    (0.8, 0.8, 0.0, 1.0),    # yellow
]

# Fixed colors for the three new series (same across all cycles for quick visual ID)
_RAW_A_COLOR   = (0.0, 0.8, 0.8, 0.7)   # cyan  — raw tag_world_a
_RAW_B_COLOR   = (0.8, 0.0, 0.8, 0.7)   # magenta — raw tag_world_b
_FILT_COLOR    = (1.0, 0.85, 0.0, 1.0)  # gold  — geometrically filtered pair


class TfBridgeNode(Node):
    def __init__(self):
        super().__init__('tf_bridge')

        self.declare_parameter('pi_ws_url',             'ws://192.168.1.241:8000/ws')
        self.declare_parameter('confidence_threshold',  20.0)
        self.declare_parameter('camera_height_m',       0.075)
        self.declare_parameter('camera_height_tol_m',   0.020)
        self.declare_parameter('tag_id_world_a',        2)
        self.declare_parameter('tag_id_world_b',        3)
        self.declare_parameter('world_offset_m',       0.065)
        self.declare_parameter('world_offset_tol_m',   0.020)
        self.declare_parameter('world_near_zero_tol_m', 0.025)
        self.declare_parameter('max_position_jump_m',  0.10)
        self.declare_parameter('position_smooth_alpha', 0.4)
        # TUNE: pan/tilt kinematic offsets — must match picar_tracking.urdf joint origins
        self.declare_parameter('pan_z_m',  0.060)   # height of pan servo above car_base (-Y)
        self.declare_parameter('tilt_z_m', 0.020)   # height of tilt servo above pan_link (-Y)
        self.declare_parameter('cam_z_m',  0.015)   # height of camera above tilt_link (-Y)
        self.declare_parameter('ibvs_anchor_mode',       False)  # use tag0 as world anchor (IBVS-only tracking)
        self.declare_parameter('ibvs_z_filter',          False)  # enable cam-Z PnP-flip filter in ibvs_anchor path
        self.declare_parameter('ibvs_pos_smooth_alpha',  0.3)  # EWMA alpha for car_base position in ibvs_anchor path
        self.declare_parameter('ibvs_max_jump_m',        1.0)    # velocity gate for ibvs_anchor path (large: car may be hand-carried)

        self._ws_url           = self.get_parameter('pi_ws_url').value
        self._conf_threshold   = self.get_parameter('confidence_threshold').value
        self._camera_height     = self.get_parameter('camera_height_m').value
        self._camera_height_tol = self.get_parameter('camera_height_tol_m').value
        self._tag_id_world_a    = self.get_parameter('tag_id_world_a').value
        self._tag_id_world_b   = self.get_parameter('tag_id_world_b').value
        self._world_offset     = self.get_parameter('world_offset_m').value
        self._world_offset_tol = self.get_parameter('world_offset_tol_m').value
        self._world_near_zero  = self.get_parameter('world_near_zero_tol_m').value
        self._max_jump_m       = self.get_parameter('max_position_jump_m').value
        self._pos_smooth_alpha = self.get_parameter('position_smooth_alpha').value
        self._pan_z_m           = self.get_parameter('pan_z_m').value
        self._tilt_z_m          = self.get_parameter('tilt_z_m').value
        self._cam_z_m           = self.get_parameter('cam_z_m').value
        self._ibvs_anchor_mode       = self.get_parameter('ibvs_anchor_mode').value
        self._ibvs_z_filter          = self.get_parameter('ibvs_z_filter').value
        self._ibvs_pos_smooth_alpha  = self.get_parameter('ibvs_pos_smooth_alpha').value
        self._ibvs_max_jump_m        = self.get_parameter('ibvs_max_jump_m').value

        self._tf_broadcaster        = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        self._car_pub        = self.create_publisher(MarkerArray, '/trajectory/car',           10)
        self._tag0_pub       = self.create_publisher(MarkerArray, '/trajectory/tag0',          10)
        self._tag2_raw_pub   = self.create_publisher(MarkerArray, '/trajectory/tag2_raw',      10)
        self._tag3_raw_pub   = self.create_publisher(MarkerArray, '/trajectory/tag3_raw',      10)
        self._pair_filt_pub  = self.create_publisher(MarkerArray, '/trajectory/pair_filtered', 10)
        _latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  reliability=ReliabilityPolicy.RELIABLE)
        self._world_pub      = self.create_publisher(Marker, '/marker/world', _latched_qos)
        self._js_pub         = self.create_publisher(JointState,  '/joint_states',             10)

        self.create_service(Empty, '/reset_markers', self._reset_markers_cb)

        # Per-cycle LINE_STRIP markers: cycle_n -> Marker
        self._car_markers:        dict = {}
        self._tag0_markers:       dict = {}
        self._tag2_raw_markers:   dict = {}
        self._tag3_raw_markers:   dict = {}
        self._pair_filt_markers:  dict = {}
        self._current_cycle = -1
        self._world_marker_published = False

        # Session logging
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_dir = os.path.expanduser(f"~/picar_ros/logs/session_{ts}")
        os.makedirs(self._session_dir, exist_ok=True)

        log_path = os.path.join(self._session_dir, "tf_bridge.log")
        handler  = logging.FileHandler(log_path)
        handler.setFormatter(logging.Formatter(
            "%(asctime)s.%(msecs)03d [%(levelname)-5s] [%(name)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        ))
        self._pylog = logging.getLogger("tf_bridge")
        self._pylog.setLevel(logging.DEBUG)
        self._pylog.addHandler(handler)
        self._pylog.addHandler(logging.StreamHandler())

        # Per-cycle world JSON file
        self._cycle_file       = None
        self._cycle_file_n     = -1
        self._cycle_file_first = True

        # Per-cycle PLY timestamps (cycle_n -> "HHMMSS")
        self._cycle_ts: dict = {}

        # World frame state
        self._world_initialized   = False
        self._T_world_anchor      = None    # 4x4 ndarray, set once at first valid pair
        self._T_world_camera_last = None    # most recent valid camera TF (4x4)
        self._waiting_logged      = False

        # Velocity gate + EWMA smoother (tracks car_base world position)
        self._pos_smooth:      np.ndarray = None
        self._flip_skip_count: int = 0
        self._z_skip_count:    int = 0

        self._first_detection = True

        # Last known joint angles — published at startup so robot_state_publisher
        # has transforms before the Pi connects and sends tag data.
        self._last_pan_deg  = 0.0
        self._last_tilt_deg = 0.0
        self.create_timer(0.1, self._js_timer_cb)   # 10 Hz heartbeat

        self._pylog.info("tf_bridge started | session=%s", self._session_dir)
        self._pylog.info("camera_height=%.3fm±%.3fm  world_tags=[%d,%d]  x_offset=%.3fm  offset_tol=%.3fm  nz_tol=%.3fm",
                         self._camera_height, self._camera_height_tol,
                         self._tag_id_world_a, self._tag_id_world_b,
                         self._world_offset, self._world_offset_tol, self._world_near_zero)
        self._pylog.info("connecting to %s", self._ws_url)

    # ── WebSocket connection loop ──────────────────────────────────────────────

    async def _connect_loop(self):
        delay = 1.0
        while rclpy.ok():
            try:
                async with websockets.connect(self._ws_url) as ws:
                    self._pylog.info("websocket connected to %s", self._ws_url)
                    delay = 1.0
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        if msg.get('type') == 'tag_detections':
                            self._process_frame(msg)
            except (ConnectionRefusedError, OSError) as e:
                self._pylog.warning("connection refused: %s — retry in %.1fs", e, delay)
                await asyncio.sleep(delay)
                delay = min(delay * 2, 30.0)
            except websockets.exceptions.ConnectionClosed as e:
                self._pylog.warning("websocket closed: %s — retry in %.1fs", e, delay)
                await asyncio.sleep(delay)
                delay = min(delay * 2, 30.0)
            except Exception as e:
                self._pylog.error("websocket error: %s — retry in %.1fs", e, delay)
                await asyncio.sleep(delay)
                delay = min(delay * 2, 30.0)

    # ── Frame processing ───────────────────────────────────────────────────────

    def _process_frame(self, msg: dict):
        tags           = msg.get('tags', [])
        cycle          = int(msg.get('cycle', 0))
        ts             = float(msg.get('ts', time.time()))
        pair_valid     = bool(msg.get('pair_valid', False))
        pan_angle_deg  = float(msg.get('pan_angle_deg', 0.0))
        tilt_angle_deg = float(msg.get('tilt_angle_deg', 0.0))
        self._publish_joint_states(pan_angle_deg, tilt_angle_deg)

        # Filter by confidence threshold
        tags = [t for t in tags if t.get('confidence', 0.0) >= self._conf_threshold]

        tag_a = next((t for t in tags if t['id'] == self._tag_id_world_a), None)
        tag_b = next((t for t in tags if t['id'] == self._tag_id_world_b), None)
        tag0  = next((t for t in tags if t['id'] == 0), None)

        if self._first_detection and (tag_a or tag_b or tag0):
            self._first_detection = False
            self._pylog.info("first_detection session cycle=%d", cycle)

        # Cycle transition
        if cycle != self._current_cycle:
            self._on_cycle_start(cycle)

        # When pair is not valid: try ibvs_anchor_mode (tag0-based), otherwise hold last TF.
        if not pair_valid:
            if self._ibvs_anchor_mode and tag0 is not None:
                self._process_ibvs_anchor_frame(tag0, cycle, ts, pan_angle_deg, tilt_angle_deg)
                return
            if not self._waiting_logged:
                self._pylog.info("waiting_for_valid_pair: world not yet initialized or pair failed")
                self._waiting_logged = True
            if self._T_world_camera_last is not None:
                self._record_raw_tags(tag_a, tag_b, cycle)
            self._publish_marker_arrays()
            return
        self._waiting_logged = False

        # Both world tags must be present in a valid-pair frame
        if tag_a is None or tag_b is None:
            self._pylog.warning("pair_valid=True but one world tag missing — skipping")
            return

        t_a = np.array(tag_a['pose_t'], dtype=np.float64).reshape(3)
        t_b = np.array(tag_b['pose_t'], dtype=np.float64).reshape(3)
        R_a = np.array(tag_a['pose_R'], dtype=np.float64)
        R_b = np.array(tag_b['pose_R'], dtype=np.float64)

        t_mid = (t_a + t_b) / 2.0
        # Use the higher-confidence tag's rotation for the anchor frame
        R_mid = R_a if tag_a.get('confidence', 0) >= tag_b.get('confidence', 0) else R_b

        T_camera_anchor_curr = self._build_4x4(R_mid, t_mid)

        # Initialize floor-anchored world frame on first valid pair.
        # World: origin = floor below camera start, Z = up, Y = toward wall, X = along wall.
        if not self._world_initialized:
            R_CW = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)
            T_world_camera_0  = self._build_4x4(R_CW, np.array([0.0, 0.0, self._camera_height]))
            self._T_world_anchor = T_world_camera_0 @ T_camera_anchor_curr
            self._world_initialized = True
            self._pos_smooth = None  # reset smoother so first car_base pos seeds it cleanly
            anchor_pos  = self._T_world_anchor[:3, 3]
            anchor_quat = Rotation.from_matrix(self._T_world_anchor[:3, :3]).as_quat()
            self._publish_static_tf('world', 'world_anchor', anchor_pos, anchor_quat)
            self._pylog.info(
                "world_initialized camera_height=%.3fm anchor=[%.3f, %.3f, %.3f]",
                self._camera_height, *anchor_pos)

        T_world_camera = self._T_world_anchor @ np.linalg.inv(T_camera_anchor_curr)

        # Compute car_base world pose via pan/tilt forward kinematics
        pan_rad  = math.radians(pan_angle_deg)
        tilt_rad = math.radians(tilt_angle_deg)
        T_car_base_camera = self._fk_car_base_to_camera(pan_rad, tilt_rad)
        T_world_car_base  = T_world_camera @ np.linalg.inv(T_car_base_camera)

        det = np.linalg.det(T_world_camera[:3, :3])
        if det <= 0:
            self._pylog.warning("skipping frame: degenerate rotation matrix det=%.4f", det)
            return

        self._T_world_camera_last = T_world_camera

        cam_pos       = T_world_camera[:3, 3]
        car_base_pos  = T_world_car_base[:3, 3]
        car_base_quat = Rotation.from_matrix(T_world_car_base[:3, :3]).as_quat()
        self._publish_tf('world', 'car_base', car_base_pos, car_base_quat)

        # Z filter: PnP flips push Z to ~-0.54m (underground). Only the lower bound
        # matters — valid frames can be slightly above nominal camera height.
        z_lo = self._camera_height - self._camera_height_tol
        if cam_pos[2] < z_lo:
            self._z_skip_count += 1
            self._pylog.debug("z_rejected z=%.3fm threshold=%.3fm z_skips=%d",
                              cam_pos[2], z_lo, self._z_skip_count)
            self._publish_marker_arrays()
            return

        # Velocity gate: reject frames where car_base position jumps more than max_jump_m.
        if self._pos_smooth is not None:
            jump = float(np.linalg.norm(car_base_pos - self._pos_smooth))
            if jump > self._max_jump_m:
                self._flip_skip_count += 1
                self._pylog.debug("pose_flip_rejected jump=%.3fm skip_count=%d", jump, self._flip_skip_count)
                self._publish_marker_arrays()
                return

        # EWMA smooth car_base position for trajectory tracking
        if self._pos_smooth is None:
            self._pos_smooth = car_base_pos.copy()
        else:
            a = self._pos_smooth_alpha
            self._pos_smooth = a * car_base_pos + (1.0 - a) * self._pos_smooth

        # Publish world origin marker once per session
        if not self._world_marker_published:
            self._publish_world_marker()
            self._world_marker_published = True
            self._pylog.info("world_origin_published cycle=%d", cycle)

        # Car trajectory uses smoothed car_base position
        self._append_point(self._car_markers, cycle, self._pos_smooth)

        # World positions for both world tags (this is a valid pair frame)
        T_world_tag_a = T_world_camera @ self._build_4x4(R_a, t_a)
        T_world_tag_b = T_world_camera @ self._build_4x4(R_b, t_b)
        pos_a = T_world_tag_a[:3, 3]
        pos_b = T_world_tag_b[:3, 3]

        # Raw trajectories (all detected frames, including this valid one)
        self._append_point(self._tag2_raw_markers, cycle, pos_a)
        self._append_point(self._tag3_raw_markers, cycle, pos_b)

        # Filtered trajectory — only geometrically consistent frames
        self._append_point(self._pair_filt_markers, cycle, pos_a)
        self._append_point(self._pair_filt_markers, cycle, pos_b)

        # Tag0 trajectory
        tag0_world_pos = None
        if tag0 is not None:
            R0 = np.array(tag0['pose_R'], dtype=np.float64)
            t0 = np.array(tag0['pose_t'], dtype=np.float64).reshape(3)
            T_camera_tag0 = self._build_4x4(R0, t0)
            T_world_tag0  = T_world_camera @ T_camera_tag0

            tag0_world_pos = T_world_tag0[:3, 3]
            tag0_quat = Rotation.from_matrix(T_world_tag0[:3, :3]).as_quat()
            self._publish_tf('world', 'tag0', tag0_world_pos, tag0_quat)
            self._append_point(self._tag0_markers, cycle, tag0_world_pos)

        self._publish_marker_arrays()
        self._append_world_record(ts, cycle, self._pos_smooth.tolist(),
                                  tag0_world_pos.tolist() if tag0_world_pos is not None else None,
                                  pos_a.tolist(), pos_b.tolist())

    def _process_ibvs_anchor_frame(self, tag0: dict, cycle: int, ts: float,
                                    pan_deg: float, tilt_deg: float):
        """Track camera position using tag0 as world anchor (IBVS-only mode).

        On first call: establishes world frame with camera at floor level and
        tag0 as the fixed anchor. Subsequent calls compute T_world_camera from
        tag0's current pose, tracking car movement relative to session start.
        """
        t0 = np.array(tag0['pose_t'], dtype=np.float64).reshape(3)
        R0 = np.array(tag0['pose_R'], dtype=np.float64)
        T_camera_tag0 = self._build_4x4(R0, t0)

        if not self._world_initialized:
            R_CW = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)
            T_world_camera_0 = self._build_4x4(R_CW, np.array([0.0, 0.0, self._camera_height]))
            self._T_world_anchor = T_world_camera_0 @ T_camera_tag0
            self._world_initialized = True
            self._pos_smooth = None
            anchor_pos  = self._T_world_anchor[:3, 3]
            anchor_quat = Rotation.from_matrix(self._T_world_anchor[:3, :3]).as_quat()
            self._publish_static_tf('world', 'world_anchor', anchor_pos, anchor_quat)
            self._pylog.info(
                "ibvs_world_init tag0-anchor camera_height=%.3fm anchor=[%.3f, %.3f, %.3f]",
                self._camera_height, *anchor_pos)

        T_world_camera = self._T_world_anchor @ np.linalg.inv(T_camera_tag0)

        if np.linalg.det(T_world_camera[:3, :3]) <= 0:
            self._pylog.warning("ibvs_anchor: degenerate rotation skipped")
            return

        self._T_world_camera_last = T_world_camera

        T_car_base_camera = self._fk_car_base_to_camera(math.radians(pan_deg), math.radians(tilt_deg))
        T_world_car_base  = T_world_camera @ np.linalg.inv(T_car_base_camera)
        car_base_pos  = T_world_car_base[:3, 3]
        car_base_quat = Rotation.from_matrix(T_world_car_base[:3, :3]).as_quat()

        if self._ibvs_z_filter:
            cam_z = T_world_camera[2, 3]
            if cam_z < -self._camera_height_tol:
                self._z_skip_count += 1
                self._pylog.debug("ibvs_anchor z_skip cam_z=%.3f threshold=%.3f",
                                  cam_z, -self._camera_height_tol)
                self._publish_marker_arrays()
                return

        # Velocity gate — use ibvs_max_jump_m (default 1.0m) since car may be hand-carried
        if self._pos_smooth is not None:
            if np.linalg.norm(car_base_pos - self._pos_smooth) > self._ibvs_max_jump_m:
                self._flip_skip_count += 1
                self._publish_marker_arrays()
                return

        if self._pos_smooth is None:
            self._pos_smooth = car_base_pos.copy()
        else:
            a = self._ibvs_pos_smooth_alpha
            self._pos_smooth = a * car_base_pos + (1.0 - a) * self._pos_smooth

        self._publish_tf('world', 'car_base', self._pos_smooth, car_base_quat)

        T_world_tag0 = T_world_camera @ T_camera_tag0
        tag0_pos  = T_world_tag0[:3, 3]
        tag0_quat = Rotation.from_matrix(T_world_tag0[:3, :3]).as_quat()
        self._publish_tf('world', 'tag0', tag0_pos, tag0_quat)

        if not self._world_marker_published:
            self._publish_world_marker()
            self._world_marker_published = True
            self._pylog.info("world_origin_published cycle=%d", cycle)

        self._append_point(self._car_markers,  cycle, self._pos_smooth)
        self._append_point(self._tag0_markers, cycle, tag0_pos)
        self._publish_marker_arrays()

    def _record_raw_tags(self, tag_a, tag_b, cycle: int):
        """Append raw world-frame positions for individually detected tags using
        the last valid camera TF.  Called when pair_valid=False."""
        T_wc = self._T_world_camera_last
        if tag_a is not None:
            try:
                R_a = np.array(tag_a['pose_R'], dtype=np.float64)
                t_a = np.array(tag_a['pose_t'], dtype=np.float64).reshape(3)
                pos_a = (T_wc @ self._build_4x4(R_a, t_a))[:3, 3]
                self._append_point(self._tag2_raw_markers, cycle, pos_a)
            except Exception:
                pass
        if tag_b is not None:
            try:
                R_b = np.array(tag_b['pose_R'], dtype=np.float64)
                t_b = np.array(tag_b['pose_t'], dtype=np.float64).reshape(3)
                pos_b = (T_wc @ self._build_4x4(R_b, t_b))[:3, 3]
                self._append_point(self._tag3_raw_markers, cycle, pos_b)
            except Exception:
                pass

    # ── Cycle management ───────────────────────────────────────────────────────

    def _on_cycle_start(self, cycle: int):
        if self._current_cycle >= 0:
            self._pylog.info("cycle_end cycle=%d flip_skips=%d z_skips=%d",
                             self._current_cycle, self._flip_skip_count, self._z_skip_count)
            self._flip_skip_count = 0
            self._z_skip_count    = 0
            self._flush_cycle_ply(self._current_cycle)
        self._close_cycle_file()
        self._current_cycle = cycle
        self._cycle_ts[cycle] = datetime.now().strftime("%H%M%S")
        self._pylog.info("cycle_start cycle=%d", cycle)

        color = self._cycle_color(cycle)

        for markers_dict, ns, fixed_color in [
            (self._car_markers,       'car',           None),
            (self._tag0_markers,      'tag0',          None),
            (self._tag2_raw_markers,  'tag2_raw',      _RAW_A_COLOR),
            (self._tag3_raw_markers,  'tag3_raw',      _RAW_B_COLOR),
            (self._pair_filt_markers, 'pair_filtered', _FILT_COLOR),
        ]:
            m = Marker()
            m.header.frame_id = 'world'
            m.ns              = f'trajectory_{ns}'
            m.id              = cycle
            m.type            = Marker.LINE_STRIP
            m.action          = Marker.ADD
            m.scale.x         = 0.01   # 1 cm line width
            if fixed_color is not None:
                r, g, b, a = fixed_color
                m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            else:
                m.color = color
            m.points          = []
            markers_dict[cycle] = m

        ts_str = datetime.now().strftime("%H%M%S")
        fname  = f"cycle_{cycle}_world_{ts_str}.json"
        path   = os.path.join(self._session_dir, fname)
        self._cycle_file       = open(path, 'w')
        self._cycle_file_first = True
        self._cycle_file_n     = cycle
        self._cycle_file.write('[\n')
        self._pylog.info("cycle_file_open file=%s", fname)

    def _close_cycle_file(self):
        if self._cycle_file is None:
            return
        self._cycle_file.write('\n]\n')
        self._cycle_file.flush()
        self._cycle_file.close()
        self._cycle_file = None
        self._pylog.info("cycle_file_close cycle=%d", self._cycle_file_n)

    def _append_world_record(self, ts: float, cycle: int, car_pos, tag0_pos,
                             tag_a_pos=None, tag_b_pos=None):
        if self._cycle_file is None:
            return
        record = {'ts': round(ts, 6), 'cycle': cycle, 'car': car_pos}
        if tag0_pos is not None:
            record['tag0'] = tag0_pos
        if tag_a_pos is not None:
            record[f'tag{self._tag_id_world_a}'] = tag_a_pos
        if tag_b_pos is not None:
            record[f'tag{self._tag_id_world_b}'] = tag_b_pos
        if not self._cycle_file_first:
            self._cycle_file.write(',\n')
        self._cycle_file_first = False
        self._cycle_file.write(json.dumps(record))

    # ── ROS2 publishing helpers ────────────────────────────────────────────────

    def _publish_tf(self, parent: str, child: str, pos, quat):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id  = child
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self._tf_broadcaster.sendTransform(t)

    def _publish_world_marker(self):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns              = 'world_origin'
        m.id              = 0
        m.type            = Marker.SPHERE
        m.action          = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.06   # 6 cm sphere
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
        self._world_pub.publish(m)

    def _flush_cycle_ply(self, cycle: int):
        ts_str = self._cycle_ts.get(cycle, 'unknown')
        wa = self._tag_id_world_a
        wb = self._tag_id_world_b
        for series, markers_dict, r, g, b in [
            ('car',              self._car_markers,        0,   220, 0),
            ('tag0',             self._tag0_markers,       220, 0,   0),
            (f'tag{wa}_raw',     self._tag2_raw_markers,   0,   200, 200),
            (f'tag{wb}_raw',     self._tag3_raw_markers,   200, 0,   200),
            ('pair_filtered',    self._pair_filt_markers,  220, 180, 0),
        ]:
            if cycle not in markers_dict:
                continue
            pts = markers_dict[cycle].points
            if not pts:
                continue
            fname = f"{series}_cycle_{cycle}_{ts_str}.ply"
            path  = os.path.join(self._session_dir, fname)
            self._write_ply(path, pts, r, g, b)
            self._pylog.info("ply_written file=%s points=%d", fname, len(pts))

    def _write_ply(self, path: str, points, r: int, g: int, b: int):
        with open(path, 'w') as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write("end_header\n")
            for p in points:
                f.write(f"{p.x:.4f} {p.y:.4f} {p.z:.4f} {r} {g} {b}\n")

    def _publish_static_tf(self, parent: str, child: str, pos, quat):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id  = child
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self._static_tf_broadcaster.sendTransform(t)

    def _append_point(self, markers_dict: dict, cycle: int, pos):
        if cycle not in markers_dict:
            return
        p = Point()
        p.x = float(pos[0])
        p.y = float(pos[1])
        p.z = float(pos[2])
        markers_dict[cycle].points.append(p)

    def _publish_marker_arrays(self):
        now = self.get_clock().now().to_msg()

        for pub, markers_dict in [
            (self._car_pub,       self._car_markers),
            (self._tag0_pub,      self._tag0_markers),
            (self._tag2_raw_pub,  self._tag2_raw_markers),
            (self._tag3_raw_pub,  self._tag3_raw_markers),
            (self._pair_filt_pub, self._pair_filt_markers),
        ]:
            arr = MarkerArray()
            for m in markers_dict.values():
                m.header.stamp = now
                arr.markers.append(m)
            pub.publish(arr)

    # ── Service handler ────────────────────────────────────────────────────────

    def _reset_markers_cb(self, request, response):
        self._pylog.info("reset_markers service called")
        self._close_cycle_file()
        self._car_markers.clear()
        self._tag0_markers.clear()
        self._tag2_raw_markers.clear()
        self._tag3_raw_markers.clear()
        self._pair_filt_markers.clear()
        self._world_marker_published = False
        self._current_cycle = -1
        self._first_detection = True
        self._world_initialized   = False
        self._T_world_anchor      = None
        self._T_world_camera_last = None
        self._pos_smooth       = None
        self._flip_skip_count  = 0
        self._z_skip_count     = 0
        self._waiting_logged   = False
        # Publish empty arrays to clear RViz displays
        for pub in [self._car_pub, self._tag0_pub,
                    self._tag2_raw_pub, self._tag3_raw_pub, self._pair_filt_pub]:
            pub.publish(MarkerArray())
        return response

    # ── Kinematics helpers ─────────────────────────────────────────────────────

    def _fk_car_base_to_camera(self, pan_rad: float, tilt_rad: float) -> np.ndarray:
        """Return T_car_base_camera: 4x4 transform that maps camera-frame coords to
        car_base-frame coords.  Uses the same coordinate convention as the URDF
        (X=right, Y=down, Z=forward).  Joint offsets match picar_tracking.urdf.
        TUNE: pan_z_m / tilt_z_m / cam_z_m ROS parameters match URDF joint origins."""
        # Translation to pan joint origin (pan servo is pan_z above car_base, -Y = up)
        T_to_pan  = self._build_4x4(np.eye(3), [0.0, -self._pan_z_m, 0.0])
        # Pan rotation around Y (yaw in camera convention)
        R_pan     = Rotation.from_euler('y', pan_rad).as_matrix()
        T_pan_rot = self._build_4x4(R_pan, [0.0, -self._tilt_z_m, 0.0])
        # Tilt rotation around X (pitch in camera convention), then translate to camera
        R_tilt    = Rotation.from_euler('x', tilt_rad).as_matrix()
        T_tilt_rot = self._build_4x4(R_tilt, [0.0, -self._cam_z_m, 0.0])
        return T_to_pan @ T_pan_rot @ T_tilt_rot

    def _js_timer_cb(self):
        self._publish_joint_states(self._last_pan_deg, self._last_tilt_deg)

    def _publish_joint_states(self, pan_deg: float, tilt_deg: float):
        self._last_pan_deg  = pan_deg
        self._last_tilt_deg = tilt_deg
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['pan_joint', 'tilt_joint']
        js.position = [math.radians(pan_deg), math.radians(tilt_deg)]
        self._js_pub.publish(js)

    # ── Math helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _build_4x4(R: np.ndarray, t: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = t
        return T

    @staticmethod
    def _cycle_color(cycle: int) -> ColorRGBA:
        r, g, b, a = _CYCLE_COLORS[cycle % len(_CYCLE_COLORS)]
        c = ColorRGBA()
        c.r = r; c.g = g; c.b = b; c.a = a
        return c


def main(args=None):
    rclpy.init(args=args)
    node = TfBridgeNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(node._connect_loop())
    except KeyboardInterrupt:
        pass
    finally:
        node._close_cycle_file()
        if node._current_cycle >= 0:
            node._flush_cycle_ply(node._current_cycle)
        executor.shutdown(wait=False)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
