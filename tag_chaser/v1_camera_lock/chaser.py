"""
chaser.py -- TagChaser: AprilTag detection + PID steering on Raspberry Pi.

Camera is owned by server.py. Each frame is passed in via process_frame().
This class handles detection, PID steering, and WebSocket broadcasts only.
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

_logger       = logging.getLogger("picarx")
_LOG_INTERVAL = 1.0

_CALIB_FILE = os.path.normpath(
    os.path.join(os.path.dirname(__file__),
                 '..', '..', 'camera_cal_marker', 'camera_calibration_pi.yaml')
)


def _load_calibration(path: str):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], dtype=np.float64)
    return K, D


class TagChaser:
    def __init__(self, px, config: dict, broadcast_fn=None):
        self.px        = px
        self.broadcast = broadcast_fn or (lambda msg: None)

        pid_cfg   = config['pid']
        chase_cfg = config['chase']
        cam_cfg   = config['camera']

        steer_limit = float(chase_cfg.get('steer_limit_deg', 20))
        self.pid = PID(
            kp=float(pid_cfg['kp']),
            ki=float(pid_cfg['ki']),
            kd=float(pid_cfg['kd']),
            output_limits=(-steer_limit, steer_limit),
        )

        self.stop_dist_cm     = float(chase_cfg.get('stop_distance_cm', 10.0))
        self.tag_lost_timeout = float(chase_cfg.get('tag_lost_timeout_s', 0.5))
        self.tag_id           = int(chase_cfg.get('tag_id', 1))
        self.countdown_s      = int(chase_cfg.get('countdown_s', 3))
        self.cam_w            = int(cam_cfg.get('width', 640))
        self.cam_h            = int(cam_cfg.get('height', 480))
        self.tag_size_m       = float(cam_cfg.get('tag_size_m', 0.05))

        self._K, self._D = _load_calibration(_CALIB_FILE)
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

        self._running = threading.Event()
        self._chasing = threading.Event()
        self._speed   = 30

        self._last_seen_time  = None
        self._last_steer      = 0.0
        self._t_last          = None
        self._last_log_t      = 0.0
        self._prev_found      = None
        self._last_broadcast_t = 0.0
        self._at_stop_dist    = False

    # ── Public API ─────────────────────────────────────────────────────────────

    def is_running(self) -> bool:
        return self._running.is_set()

    def start(self, speed: int = 30):
        if self._running.is_set():
            return
        self._speed = max(0, min(100, speed))
        self._running.set()
        self._chasing.clear()
        try:
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except Exception:
            pass
        threading.Thread(
            target=self._countdown, daemon=True, name='chaser-countdown').start()

    def stop(self):
        if not self._running.is_set():
            return
        self._running.clear()
        self._chasing.clear()
        try:
            self.px.stop()
            self.px.set_dir_servo_angle(0)
        except Exception:
            pass
        self.broadcast({'type': 'chase_status', 'active': False, 'state': 'idle'})

    def process_frame(self, frame):
        """Called by server.py capture thread with each BGR frame."""
        if not self._chasing.is_set():
            return

        gray  = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        t_now = time.perf_counter()
        if self._t_last is None:
            self._t_last = t_now
        dt = max(t_now - self._t_last, 1e-3)
        self._t_last = t_now

        log_due   = (t_now - self._last_log_t)      >= _LOG_INTERVAL
        bcast_due = (t_now - self._last_broadcast_t) >= 0.1

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self._fx, self._fy, self._cx, self._cy],
            tag_size=self.tag_size_m,
        )

        tag = next((d for d in detections if d.tag_id == self.tag_id), None)

        if tag is not None:
            self._last_seen_time = t_now
            dist_cm = (float(np.linalg.norm(tag.pose_t)) * 100
                       if tag.pose_t is not None else -1.0)

            if self._prev_found is not True:
                _logger.info("detect tag_acquired id=%d dist_cm=%.1f", self.tag_id, dist_cm)
                self._prev_found = True

            if 0 < dist_cm <= self.stop_dist_cm:
                _logger.info("chase_stop reason=distance_reached dist_cm=%.1f", dist_cm)
                self.px.stop()
                self.pid.reset()
                if bcast_due:
                    self._last_broadcast_t = t_now
                    self.broadcast({
                        'type': 'chase_detection', 'found': True,
                        'corners': tag.corners.tolist(), 'center': tag.center.tolist(),
                        'frame_w': self.cam_w, 'frame_h': self.cam_h,
                        'distance_cm': round(dist_cm, 1),
                    })
                    self.broadcast({'type': 'chase_status', 'active': True,
                                    'state': 'stopping', 'distance_cm': round(dist_cm, 1)})
            else:
                error = (tag.center[0] - self.cam_w / 2.0) / (self.cam_w / 2.0)
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
                        'corners': tag.corners.tolist(), 'center': tag.center.tolist(),
                        'frame_w': self.cam_w, 'frame_h': self.cam_h,
                        'distance_cm': round(dist_cm, 1),
                    })
                    self.broadcast({'type': 'chase_status', 'active': True,
                                    'state': 'chasing', 'distance_cm': round(dist_cm, 1)})

            if log_due:
                _logger.debug("detect found=True dist_cm=%.1f", dist_cm)
                self._last_log_t = t_now
        else:
            if self._prev_found is not False:
                _logger.info("detect tag_lost")
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
                self.broadcast({'type': 'chase_status', 'active': True, 'state': 'searching'})

            if log_due:
                _logger.debug("detect found=False")
                self._last_log_t = t_now

    # ── Internals ──────────────────────────────────────────────────────────────

    def _countdown(self):
        for c in range(self.countdown_s, 0, -1):
            if not self._running.is_set():
                return
            _logger.info("chase_countdown n=%d", c)
            self.broadcast({'type': 'chase_status', 'active': True,
                            'state': 'starting', 'countdown': c})
            time.sleep(1.0)
        if not self._running.is_set():
            return
        self.pid.reset()
        self._last_seen_time = None
        self._last_steer     = 0.0
        self._last_log_t      = 0.0
        self._prev_found      = None
        self._last_broadcast_t = 0.0
        self._t_last          = time.perf_counter()
        self._chasing.set()
        _logger.info("chasing started speed=%d", self._speed)
