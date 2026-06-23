"""
chaser.py -- TagChaser v2: dual-tag detection, world-frame anchor, TF publishing.

Camera is owned by server.py. Each frame is passed in via process_frame().
Tags world_a and world_b (default IDs 2 and 3) must both be visible and pass the
geometric consistency check to start a chase cycle. Tag ID 0 is the chase target.
The tf_publisher serializes pose data over WebSocket so tf_bridge.py on Ubuntu can
compute TF and publish RViz2 trajectories.
"""

import logging
import os
import threading
import time

import cv2
import numpy as np
import pupil_apriltags as apriltag
import yaml

from .steer_pid import PID
from .tf_publisher import TfPublisher

_logger        = logging.getLogger("picarx")
_marker_logger = logging.getLogger("marker_detector")
_LOG_INTERVAL  = 1.0


def _load_calibration(path: str):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], dtype=np.float64)
    return K, D


class TagChaser:
    def __init__(self, px, config: dict, broadcast_fn=None, session_dir: str = None):
        self.px        = px
        self.broadcast = broadcast_fn or (lambda msg: None)

        pid_cfg   = config['pid']
        chase_cfg = config['chase']
        cam_cfg   = config['camera']
        det_cfg   = config.get('detector', {})

        steer_limit = float(chase_cfg.get('steer_limit_deg', 20))
        self.pid = PID(
            kp=float(pid_cfg['kp']),
            ki=float(pid_cfg['ki']),
            kd=float(pid_cfg['kd']),
            output_limits=(-steer_limit, steer_limit),
        )

        self.stop_dist_cm          = float(chase_cfg.get('stop_distance_cm', 20.0))
        self.tag_lost_timeout      = float(chase_cfg.get('tag_lost_timeout_s', 2.0))
        self._tag_id_chase         = int(chase_cfg.get('tag_id_chase', 0))
        self._tag_id_world_a       = int(chase_cfg.get('tag_id_world_a', 2))
        self._tag_id_world_b       = int(chase_cfg.get('tag_id_world_b', 3))
        self._world_offset_m       = float(chase_cfg.get('world_offset_m', 0.065))
        self._world_offset_tol_m   = float(chase_cfg.get('world_offset_tol_m', 0.020))
        self._world_near_zero_tol  = float(chase_cfg.get('world_near_zero_tol_m', 0.025))
        self.countdown_s           = int(chase_cfg.get('countdown_s', 3))
        self._world_search_timeout = float(chase_cfg.get('world_search_timeout_s', 15))
        self._conf_threshold       = float(chase_cfg.get('confidence_threshold', 20.0))
        self._single_tag_mode      = bool(chase_cfg.get('single_tag_world_mode', False))
        self.cam_w                 = int(cam_cfg.get('width', 640))
        self.cam_h                 = int(cam_cfg.get('height', 480))
        self.tag_size_m            = float(cam_cfg.get('tag_size_m', 0.05))

        calib_rel  = cam_cfg.get('calibration_file', '../../camera_cal_marker/camera_calibration.yaml')
        calib_path = os.path.normpath(os.path.join(os.path.dirname(__file__), calib_rel))
        if not os.path.exists(calib_path):
            raise FileNotFoundError(
                f"Camera calibration file not found: {calib_path}\n"
                "Run ChArUco calibration and save to camera_cal_marker/camera_calibration.yaml"
            )
        self._K, self._D = _load_calibration(calib_path)
        self._fx = self._K[0, 0]
        self._fy = self._K[1, 1]
        self._cx = self._K[0, 2]
        self._cy = self._K[1, 2]

        self.detector = apriltag.Detector(
            families='tag36h11',
            nthreads=int(det_cfg.get('nthreads', 2)),
            quad_decimate=float(det_cfg.get('quad_decimate', 2.0)),
            quad_sigma=float(det_cfg.get('quad_sigma', 0.0)),
            refine_edges=int(det_cfg.get('refine_edges', 1)),
            decode_sharpening=float(det_cfg.get('decode_sharpening', 0.25)),
        )

        # Session logging
        self._session_dir = session_dir or os.path.join(
            os.path.expanduser('~'), 'picar_ros', 'logs', 'session_standalone'
        )
        os.makedirs(self._session_dir, exist_ok=True)
        self._log_handler = None

        # State machine — protected by _state_lock
        self._state_lock = threading.Lock()
        self._state      = 'idle'  # idle | world_search | countdown | chasing

        # Cycle counter (increments on each start(), reset only on __init__)
        self._cycle = -1

        # TfPublisher (created once, manages per-cycle files)
        self._tf_pub = TfPublisher(self._session_dir, self.broadcast)

        # World search tracking
        self._world_search_start  = 0.0
        self._world_search_bcast_t = 0.0

        # Chase state
        self._speed          = 30
        self._last_seen_time = None
        self._last_steer     = 0.0
        self._t_last         = None
        self._last_log_t     = 0.0
        self._last_broadcast_t = 0.0
        self._prev_found     = None
        self._at_stop_dist   = False
        self._world_visible  = False

        # Set up marker_detector logger (StreamHandler — file handler added lazily)
        if not _marker_logger.handlers:
            sh = logging.StreamHandler()
            sh.setLevel(logging.DEBUG)
            sh.setFormatter(logging.Formatter(
                "%(asctime)s.%(msecs)03d [%(levelname)-5s] [%(name)s] %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            ))
            _marker_logger.addHandler(sh)
            _marker_logger.setLevel(logging.DEBUG)

    # ── Public API ─────────────────────────────────────────────────────────────

    def is_running(self) -> bool:
        with self._state_lock:
            return self._state != 'idle'

    def start(self, speed: int = 30):
        with self._state_lock:
            if self._state != 'idle':
                return
            self._cycle += 1
            self._speed = max(0, min(100, speed))
            self._state = 'world_search'
            self._world_search_start   = time.perf_counter()
            self._world_search_bcast_t = 0.0

        self._ensure_log_handler()

        try:
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except Exception:
            pass

        remaining = int(self._world_search_timeout)
        self.broadcast({'type': 'chase_status', 'active': True,
                        'state': 'world_search', 'countdown': remaining})
        _marker_logger.info("chase_toggle_on cycle=%d", self._cycle)

    def stop(self):
        with self._state_lock:
            old_state = self._state
            if old_state == 'idle':
                return
            self._state = 'idle'

        try:
            self.px.stop()
            self.px.set_dir_servo_angle(0)
        except Exception:
            pass

        self._tf_pub.close_cycle()

        if old_state == 'chasing':
            _marker_logger.info("chase_stop cycle=%d reason=toggle_off", self._cycle)

        self.broadcast({'type': 'chase_status', 'active': False, 'state': 'idle'})

    def process_frame(self, frame):
        """Called by server.py capture thread with each RGB frame."""
        with self._state_lock:
            state = self._state
        if state == 'idle':
            return

        gray  = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        t_now = time.perf_counter()

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self._fx, self._fy, self._cx, self._cy],
            tag_size=self.tag_size_m,
        )

        # Filter by confidence and extract tags of interest
        tag0  = next((d for d in detections
                      if d.tag_id == self._tag_id_chase
                      and d.decision_margin >= self._conf_threshold), None)
        tag_a = next((d for d in detections
                      if d.tag_id == self._tag_id_world_a
                      and d.decision_margin >= self._conf_threshold), None)
        tag_b = None
        if not self._single_tag_mode:
            tag_b = next((d for d in detections
                          if d.tag_id == self._tag_id_world_b
                          and d.decision_margin >= self._conf_threshold), None)

        if self._single_tag_mode:
            pair_valid = tag_a is not None
        else:
            pair_valid = (tag_a is not None and tag_b is not None
                          and self._validate_world_pair(tag_a, tag_b))

        if state == 'world_search':
            self._do_world_search(t_now, pair_valid)
        elif state == 'chasing':
            self._do_chasing(t_now, tag0, tag_a, tag_b, pair_valid)

    # ── Internal state handlers ────────────────────────────────────────────────

    def _validate_world_pair(self, tag_a, tag_b) -> bool:
        """Return True if the geometric relationship between the two world tags
        matches the expected constraint: |X|≈world_offset_m (horizontal separation),
        Y≈0 (no vertical offset). Z (depth) is excluded — monocular depth estimation
        has inherent bias and noise that makes it unreliable as a filter."""
        diff = np.array(tag_b.pose_t).reshape(3) - np.array(tag_a.pose_t).reshape(3)
        x_ok = abs(abs(diff[0]) - self._world_offset_m) < self._world_offset_tol_m
        y_ok = abs(diff[1]) < self._world_near_zero_tol
        return bool(x_ok and y_ok)

    def _do_world_search(self, t_now: float, pair_valid: bool):
        elapsed   = t_now - self._world_search_start
        remaining = max(0.0, self._world_search_timeout - elapsed)

        if pair_valid:
            with self._state_lock:
                if self._state != 'world_search':
                    return
                self._state = 'countdown'
            _marker_logger.info("chase_start cycle=%d world=acquired pair_validated", self._cycle)
            threading.Thread(
                target=self._countdown, daemon=True, name='chaser-countdown').start()
            return

        if elapsed >= self._world_search_timeout:
            with self._state_lock:
                if self._state != 'world_search':
                    return
                self._state = 'idle'
            _marker_logger.info("world_search_timeout cycle=%d — no valid world pair found", self._cycle)
            self.broadcast({'type': 'chase_status', 'active': False,
                            'state': 'world_not_found'})
            return

        # Emit world_search status at 1 Hz
        if t_now - self._world_search_bcast_t >= 1.0:
            self._world_search_bcast_t = t_now
            self.broadcast({'type': 'chase_status', 'active': True,
                            'state': 'world_search', 'countdown': int(remaining)})

    def _do_chasing(self, t_now: float, tag0, tag_a, tag_b, pair_valid: bool):
        # Track world-pair visibility changes
        if pair_valid:
            if not self._world_visible:
                self._world_visible = True
                _marker_logger.info("world_reacquired — TF gap closed")
        else:
            if self._world_visible:
                self._world_visible = False
                _marker_logger.info("world_lost — continuing chase, TF gap open")

        if self._t_last is None:
            self._t_last = t_now
        dt = max(t_now - self._t_last, 1e-3)
        self._t_last = t_now

        log_due   = (t_now - self._last_log_t)       >= _LOG_INTERVAL
        bcast_due = (t_now - self._last_broadcast_t) >= 0.1

        # Publish TF data — include tag0 plus both world tags when present
        detected_for_tf = [t for t in [tag0, tag_a, tag_b] if t is not None]
        self._tf_pub.on_frame(t_now, self._cycle, detected_for_tf, bcast_due,
                              pair_valid=pair_valid)

        world_state = 'world_acquired' if self._world_visible else 'world_lost'

        if tag0 is not None:
            self._last_seen_time = t_now
            dist_cm = (float(np.linalg.norm(tag0.pose_t)) * 100
                       if tag0.pose_t is not None else -1.0)

            if self._prev_found is not True:
                _marker_logger.info("detect tag0_acquired dist_cm=%.1f", dist_cm)
                self._prev_found = True

            if 0 < dist_cm <= self.stop_dist_cm:
                if not self._at_stop_dist:
                    self._at_stop_dist = True
                    _marker_logger.info("chase_stop cycle=%d reason=distance_reached dist_cm=%.1f",
                                        self._cycle, dist_cm)
                self.px.stop()
                self.pid.reset()
                if bcast_due:
                    self._last_broadcast_t = t_now
                    self.broadcast({
                        'type': 'chase_detection', 'found': True,
                        'corners': tag0.corners.tolist(), 'center': tag0.center.tolist(),
                        'frame_w': self.cam_w, 'frame_h': self.cam_h,
                        'distance_cm': round(dist_cm, 1),
                    })
                    self.broadcast({'type': 'chase_status', 'active': True,
                                    'state': 'stopping', 'distance_cm': round(dist_cm, 1),
                                    'world': world_state, 'steer_angle': 0})
            else:
                self._at_stop_dist = False
                error = (tag0.center[0] - self.cam_w / 2.0) / (self.cam_w / 2.0)
                steer = self.pid.compute(error, dt)
                if abs(steer - self._last_steer) >= 1.0:
                    _logger.debug("steer angle=%d", int(round(steer)))
                self._last_steer = steer
                self.px.set_dir_servo_angle(int(round(steer)))
                self.px.forward(self._speed)
                if bcast_due:
                    self._last_broadcast_t = t_now
                    self.broadcast({
                        'type': 'chase_detection', 'found': True,
                        'corners': tag0.corners.tolist(), 'center': tag0.center.tolist(),
                        'frame_w': self.cam_w, 'frame_h': self.cam_h,
                        'distance_cm': round(dist_cm, 1),
                    })
                    self.broadcast({'type': 'chase_status', 'active': True,
                                    'state': 'chasing', 'distance_cm': round(dist_cm, 1),
                                    'world': world_state, 'steer_angle': int(round(steer))})

            if log_due:
                _logger.debug("detect tag0_found=True dist_cm=%.1f", dist_cm)
                self._last_log_t = t_now
        else:
            if self._prev_found is not False:
                _marker_logger.info("detect tag0_lost")
                self._prev_found = False

            if self._last_seen_time is not None:
                elapsed = t_now - self._last_seen_time
                if elapsed < self.tag_lost_timeout:
                    self.px.set_dir_servo_angle(int(round(self._last_steer)))
                    self.px.forward(self._speed)
                else:
                    self.px.stop()
                    self.pid.reset()

            if bcast_due:
                self._last_broadcast_t = t_now
                self.broadcast({'type': 'chase_detection', 'found': False,
                                'frame_w': self.cam_w, 'frame_h': self.cam_h})
                self.broadcast({'type': 'chase_status', 'active': True,
                                'state': 'searching', 'world': world_state,
                                'steer_angle': int(round(self._last_steer))})

            if log_due:
                _logger.debug("detect tag0_found=False")
                self._last_log_t = t_now

    def _countdown(self):
        for c in range(self.countdown_s, 0, -1):
            with self._state_lock:
                if self._state != 'countdown':
                    return
            _marker_logger.info("chase_countdown n=%d", c)
            self.broadcast({'type': 'chase_status', 'active': True,
                            'state': 'starting', 'countdown': c})
            time.sleep(1.0)

        with self._state_lock:
            if self._state != 'countdown':
                return
            self._state = 'chasing'

        # Reset chase state
        self.pid.reset()
        self._last_seen_time   = None
        self._last_steer       = 0.0
        self._last_log_t       = 0.0
        self._prev_found       = None
        self._last_broadcast_t = 0.0
        self._t_last           = time.perf_counter()
        self._at_stop_dist     = False
        self._world_visible    = False

        self._tf_pub.open_cycle(self._cycle)
        _marker_logger.info("chasing_started cycle=%d speed=%d", self._cycle, self._speed)

    # ── Internals ──────────────────────────────────────────────────────────────

    def _ensure_log_handler(self):
        """Lazily add FileHandler for marker_detector.log on first start()."""
        if self._log_handler is not None:
            return
        path = os.path.join(self._session_dir, "marker_detector.log")
        h = logging.FileHandler(path)
        h.setLevel(logging.DEBUG)
        h.setFormatter(logging.Formatter(
            "%(asctime)s.%(msecs)03d [%(levelname)-5s] [%(name)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        ))
        _marker_logger.addHandler(h)
        self._log_handler = h
