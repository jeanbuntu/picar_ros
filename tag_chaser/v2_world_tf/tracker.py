"""
tracker.py -- ManualTracker: tag detection + TF publishing without autonomous drive.

Operator drives with WASD; both AprilTags are detected and streamed to tf_bridge
on Ubuntu for RViz2 trajectory visualization. No PID, no motor commands.
Camera is locked to center on start().
"""

import logging
import os
import threading
import time

import cv2
import numpy as np
import pupil_apriltags as apriltag
import yaml

from .tf_publisher import TfPublisher

_logger        = logging.getLogger("picarx")
_marker_logger = logging.getLogger("marker_detector")


def _load_calibration(path: str):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], dtype=np.float64)
    return K, D


class ManualTracker:
    def __init__(self, px, config: dict, broadcast_fn=None, session_dir: str = None):
        self.px        = px
        self.broadcast = broadcast_fn or (lambda msg: None)

        chase_cfg = config['chase']
        cam_cfg   = config['camera']

        self._tag_id_chase   = int(chase_cfg.get('tag_id_chase', 0))
        self._tag_id_world   = int(chase_cfg.get('tag_id_world', 1))
        self._conf_threshold = float(chase_cfg.get('confidence_threshold', 20.0))
        self.cam_w           = int(cam_cfg.get('width', 640))
        self.cam_h           = int(cam_cfg.get('height', 480))
        self.tag_size_m      = float(cam_cfg.get('tag_size_m', 0.05))

        calib_rel  = cam_cfg.get('calibration_file',
                                  '../../camera_cal_marker/camera_calibration_pi.yaml')
        calib_path = os.path.normpath(os.path.join(os.path.dirname(__file__), calib_rel))
        if not os.path.exists(calib_path):
            raise FileNotFoundError(
                f"Camera calibration file not found: {calib_path}\n"
                "Run ChArUco calibration and save to camera_cal_marker/camera_calibration_pi.yaml"
            )
        self._K, self._D = _load_calibration(calib_path)
        self._fx = self._K[0, 0]
        self._fy = self._K[1, 1]
        self._cx = self._K[0, 2]
        self._cy = self._K[1, 2]

        self.detector = apriltag.Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )

        self._session_dir = session_dir or os.path.join(
            os.path.expanduser('~'), 'picar_ros', 'logs', 'session_standalone'
        )
        os.makedirs(self._session_dir, exist_ok=True)
        self._log_handler = None

        self._lock             = threading.Lock()
        self._running          = False
        self._cycle            = -1
        self._tf_pub           = TfPublisher(self._session_dir, self.broadcast)
        self._last_broadcast_t = 0.0
        self._last_status_t    = 0.0
        self._world_visible    = False

        if not _marker_logger.handlers:
            sh = logging.StreamHandler()
            sh.setLevel(logging.DEBUG)
            sh.setFormatter(logging.Formatter(
                "%(asctime)s.%(msecs)03d [%(levelname)-5s] [%(name)s] %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            ))
            _marker_logger.addHandler(sh)
            _marker_logger.setLevel(logging.DEBUG)

    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._cycle            += 1
            self._running           = True
            self._world_visible     = False
            self._last_broadcast_t  = 0.0
            self._last_status_t     = 0.0

        self._ensure_log_handler()

        try:
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except Exception:
            pass

        self._tf_pub.open_cycle(self._cycle)
        _marker_logger.info("manual_track_start cycle=%d", self._cycle)
        self.broadcast({'type': 'track_status', 'active': True,
                        'world': 'searching', 'cycle': self._cycle})

    def stop(self) -> None:
        with self._lock:
            if not self._running:
                return
            self._running = False

        self._tf_pub.close_cycle()
        _marker_logger.info("manual_track_stop cycle=%d", self._cycle)
        self.broadcast({'type': 'track_status', 'active': False, 'state': 'idle'})

    def process_frame(self, frame) -> None:
        with self._lock:
            if not self._running:
                return
            cycle = self._cycle

        gray  = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        t_now = time.perf_counter()

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self._fx, self._fy, self._cx, self._cy],
            tag_size=self.tag_size_m,
        )

        tag0 = next((d for d in detections
                     if d.tag_id == self._tag_id_chase
                     and d.decision_margin >= self._conf_threshold), None)
        tag1 = next((d for d in detections
                     if d.tag_id == self._tag_id_world
                     and d.decision_margin >= self._conf_threshold), None)

        if tag1 is not None:
            if not self._world_visible:
                self._world_visible = True
                _marker_logger.info("manual_track world_acquired cycle=%d", cycle)
        else:
            if self._world_visible:
                self._world_visible = False
                _marker_logger.info("manual_track world_lost cycle=%d — TF gap open", cycle)

        bcast_due = (t_now - self._last_broadcast_t) >= 0.1
        detected_for_tf = [t for t in [tag0, tag1] if t is not None]
        self._tf_pub.on_frame(t_now, cycle, detected_for_tf, bcast_due)
        if bcast_due:
            self._last_broadcast_t = t_now

        if t_now - self._last_status_t >= 1.0:
            self._last_status_t = t_now
            world_state = 'acquired' if self._world_visible else 'searching'
            self.broadcast({'type': 'track_status', 'active': True,
                            'world': world_state, 'cycle': cycle})

    def _ensure_log_handler(self):
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
