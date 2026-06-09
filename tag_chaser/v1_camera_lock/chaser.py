"""
chaser.py -- TagChaser: AprilTag detection + PID steering on Raspberry Pi.

Designed to run as a background thread spawned by dashboard/server.py.
The broadcast_fn callback pushes WebSocket messages to all connected clients.
"""

import os
import threading
import time

import cv2
import numpy as np
import pupil_apriltags as apriltag
import yaml

from .steer_pid import PID

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

        self.stop_dist_cm      = float(chase_cfg.get('stop_distance_cm', 10.0))
        self.tag_lost_timeout  = float(chase_cfg.get('tag_lost_timeout_s', 0.5))
        self.tag_id            = int(chase_cfg.get('tag_id', 1))
        self.countdown_s       = int(chase_cfg.get('countdown_s', 3))
        self.cam_w             = int(cam_cfg.get('width', 640))
        self.cam_h             = int(cam_cfg.get('height', 480))
        self.tag_size_m        = float(cam_cfg.get('tag_size_m', 0.05))

        self._K, self._D = _load_calibration(_CALIB_FILE)
        self._running    = threading.Event()
        self._thread     = None
        self._speed      = 30

    # ── Public API ─────────────────────────────────────────────────────────────

    def is_running(self) -> bool:
        return self._running.is_set()

    def start(self, speed: int = 30):
        if self._running.is_set():
            return
        self._speed = max(0, min(100, speed))
        self._running.set()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name='tag-chaser')
        self._thread.start()

    def stop(self):
        if not self._running.is_set():
            return
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=3.0)
        try:
            self.px.stop()
            self.px.set_dir_servo_angle(0)
        except Exception:
            pass
        self.broadcast({'type': 'chase_status', 'active': False, 'state': 'idle'})
        self._restart_vilib()

    # ── Internals ──────────────────────────────────────────────────────────────

    def _restart_vilib(self):
        try:
            from vilib import Vilib
            Vilib.camera_start(vflip=False, hflip=False)
            Vilib.display(local=False, web=True)
        except Exception:
            pass

    def _loop(self):
        # Lock camera to center immediately
        try:
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except Exception:
            pass

        # Release Vilib so Picamera2 can open
        try:
            from vilib import Vilib
            Vilib.camera_close()
        except Exception:
            pass

        time.sleep(0.4)

        # Init Picamera2
        try:
            from picamera2 import Picamera2
            picam2 = Picamera2()
            picam2.configure(picam2.create_video_configuration(
                main={'format': 'RGB888', 'size': (self.cam_w, self.cam_h)}
            ))
            picam2.start()
            time.sleep(0.5)
        except Exception as e:
            self.broadcast({'type': 'chase_status', 'active': False,
                            'state': 'idle', 'error': str(e)})
            self._running.clear()
            return

        # Init detector
        detector = apriltag.Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )

        fx, fy     = self._K[0, 0], self._K[1, 1]
        cx_c, cy_c = self._K[0, 2], self._K[1, 2]

        # Countdown — broadcast "Starting 3 2 1" before driving
        for c in range(self.countdown_s, 0, -1):
            if not self._running.is_set():
                picam2.stop()
                return
            self.broadcast({'type': 'chase_status', 'active': True,
                            'state': 'starting', 'countdown': c})
            time.sleep(1.0)

        self.pid.reset()
        last_seen_time = None
        last_steer     = 0.0
        t_last         = time.perf_counter()

        try:
            while self._running.is_set():
                raw   = picam2.capture_array()
                frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
                gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                t_now = time.perf_counter()
                dt    = max(t_now - t_last, 1e-3)
                t_last = t_now

                detections = detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=[fx, fy, cx_c, cy_c],
                    tag_size=self.tag_size_m,
                )

                tag = next((d for d in detections if d.tag_id == self.tag_id), None)

                if tag is not None:
                    last_seen_time = t_now

                    dist_cm = (float(np.linalg.norm(tag.pose_t)) * 100
                               if tag.pose_t is not None else -1.0)

                    # Broadcast detection for canvas overlay
                    self.broadcast({
                        'type':        'chase_detection',
                        'found':       True,
                        'corners':     tag.corners.tolist(),
                        'center':      tag.center.tolist(),
                        'frame_w':     self.cam_w,
                        'frame_h':     self.cam_h,
                        'distance_cm': round(dist_cm, 1),
                    })

                    if 0 < dist_cm <= self.stop_dist_cm:
                        # Target reached
                        self.px.stop()
                        self.pid.reset()
                        self.broadcast({'type': 'chase_status', 'active': True,
                                        'state': 'stopping',
                                        'distance_cm': round(dist_cm, 1)})
                    else:
                        # PID steer: normalize error to [-1, 1] across half frame width
                        error = (tag.center[0] - self.cam_w / 2.0) / (self.cam_w / 2.0)
                        steer = self.pid.compute(error, dt)
                        last_steer = steer

                        self.px.set_dir_servo_angle(int(round(steer)))
                        self.px.forward(self._speed)

                        self.broadcast({'type': 'chase_status', 'active': True,
                                        'state': 'chasing',
                                        'distance_cm': round(dist_cm, 1)})

                else:
                    # Tag not visible
                    self.broadcast({'type': 'chase_detection', 'found': False,
                                    'frame_w': self.cam_w, 'frame_h': self.cam_h})

                    if last_seen_time is not None:
                        elapsed = t_now - last_seen_time
                        if elapsed < self.tag_lost_timeout:
                            # Hold last steer briefly
                            self.px.set_dir_servo_angle(int(round(last_steer)))
                            self.px.forward(self._speed)
                        else:
                            self.px.stop()
                            self.pid.reset()

                    self.broadcast({'type': 'chase_status', 'active': True,
                                    'state': 'searching'})

        finally:
            try:
                picam2.stop()
            except Exception:
                pass
