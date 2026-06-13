"""
tf_bridge.py -- Ubuntu ROS2 node: Pi WebSocket → TF + MarkerArray trajectory visualization.

Connects to the Pi's dashboard WebSocket, receives tag_detections messages,
computes world-frame transforms, and publishes TF + visualization_msgs/MarkerArray
topics for RViz2.

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
from scipy.spatial.transform import Rotation
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

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


class TfBridgeNode(Node):
    def __init__(self):
        super().__init__('tf_bridge')

        self.declare_parameter('pi_ws_url', 'ws://192.168.1.241:8000/ws')
        self.declare_parameter('confidence_threshold', 20.0)

        self._ws_url        = self.get_parameter('pi_ws_url').value
        self._conf_threshold = self.get_parameter('confidence_threshold').value

        self._tf_broadcaster = TransformBroadcaster(self)

        self._car_pub   = self.create_publisher(MarkerArray, '/trajectory/car',  10)
        self._tag0_pub  = self.create_publisher(MarkerArray, '/trajectory/tag0', 10)
        self._world_pub = self.create_publisher(Marker,      '/marker/world',    1)

        self.create_service(Empty, '/reset_markers', self._reset_markers_cb)

        # Per-cycle LINE_STRIP markers: cycle_n -> Marker
        self._car_markers:  dict = {}
        self._tag0_markers: dict = {}
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

        self._first_detection = True

        self._pylog.info("tf_bridge started | session=%s", self._session_dir)
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
        tags  = msg.get('tags', [])
        cycle = int(msg.get('cycle', 0))
        ts    = float(msg.get('ts', time.time()))

        # Filter by confidence threshold
        tags = [t for t in tags if t.get('confidence', 0.0) >= self._conf_threshold]

        tag1 = next((t for t in tags if t['id'] == 1), None)
        tag0 = next((t for t in tags if t['id'] == 0), None)

        if tag1 is None:
            return

        if self._first_detection:
            self._first_detection = False
            self._pylog.info("first_detection session cycle=%d", cycle)

        # Cycle transition
        if cycle != self._current_cycle:
            self._on_cycle_start(cycle)

        # Compute T_world_camera = inv(T_camera_tag1)
        R1 = np.array(tag1['pose_R'], dtype=np.float64)
        t1 = np.array(tag1['pose_t'], dtype=np.float64).reshape(3)
        T_camera_tag1 = self._build_4x4(R1, t1)
        T_world_camera = np.linalg.inv(T_camera_tag1)

        cam_pos = T_world_camera[:3, 3]
        cam_quat = Rotation.from_matrix(T_world_camera[:3, :3]).as_quat()  # [x,y,z,w]

        self._publish_tf('world', 'camera', cam_pos, cam_quat)

        # Publish world origin marker once per session
        if not self._world_marker_published:
            self._publish_world_marker()
            self._world_marker_published = True
            self._pylog.info("world_origin_published cycle=%d", cycle)

        # Append car point
        self._append_point(self._car_markers, cycle, cam_pos)

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
        self._append_world_record(ts, cycle, cam_pos.tolist(),
                                  tag0_world_pos.tolist() if tag0_world_pos is not None else None)

    # ── Cycle management ───────────────────────────────────────────────────────

    def _on_cycle_start(self, cycle: int):
        if self._current_cycle >= 0:
            self._pylog.info("cycle_end cycle=%d", self._current_cycle)
        self._close_cycle_file()
        self._current_cycle = cycle
        self._pylog.info("cycle_start cycle=%d", cycle)

        color = self._cycle_color(cycle)

        for markers_dict, ns in ((self._car_markers, 'car'), (self._tag0_markers, 'tag0')):
            m = Marker()
            m.header.frame_id = 'world'
            m.ns              = f'trajectory_{ns}'
            m.id              = cycle
            m.type            = Marker.LINE_STRIP
            m.action          = Marker.ADD
            m.scale.x         = 0.01   # 1 cm line width
            m.color           = color
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

    def _append_world_record(self, ts: float, cycle: int, car_pos, tag0_pos):
        if self._cycle_file is None:
            return
        record = {'ts': round(ts, 6), 'cycle': cycle, 'car': car_pos}
        if tag0_pos is not None:
            record['tag0'] = tag0_pos
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

        car_array = MarkerArray()
        for m in self._car_markers.values():
            m.header.stamp = now
            car_array.markers.append(m)
        self._car_pub.publish(car_array)

        tag0_array = MarkerArray()
        for m in self._tag0_markers.values():
            m.header.stamp = now
            tag0_array.markers.append(m)
        self._tag0_pub.publish(tag0_array)

    # ── Service handler ────────────────────────────────────────────────────────

    def _reset_markers_cb(self, request, response):
        self._pylog.info("reset_markers service called")
        self._close_cycle_file()
        self._car_markers.clear()
        self._tag0_markers.clear()
        self._world_marker_published = False
        self._current_cycle = -1
        self._first_detection = True
        # Publish empty arrays to clear RViz displays
        self._car_pub.publish(MarkerArray())
        self._tag0_pub.publish(MarkerArray())
        return response

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
        executor.shutdown(wait=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
