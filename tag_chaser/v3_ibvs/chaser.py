"""
chaser.py -- TagChaser v3: IBVS + world-frame anchor + multi-mode chase.

Four operational modes (set via set_chase_mode()):
  rat_chase   -- No world anchor. IBVS centers tag0, car centering drives forward.
  ibvs_test   -- IBVS centers tag0, motors locked. For servo gain tuning.
  manual_ibvs -- IBVS centers any visible tag, WASD controls the car.
  world_ibvs  -- Full v2+ behavior: world anchor -> countdown -> TF chase.
"""

import csv
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

        # IBVS / world-scan / car-centering config
        ibvs_cfg = config.get('ibvs', {})
        scan_cfg  = config.get('world_scan', {})
        cc_cfg    = config.get('car_centering', {})

        self._ibvs_kp_pan       = float(ibvs_cfg.get('kp_pan', 0.05))
        self._ibvs_kp_tilt      = float(ibvs_cfg.get('kp_tilt', 0.05))
        self._ibvs_alpha        = float(ibvs_cfg.get('ewma_alpha', 0.3))
        self._ibvs_deadband     = float(ibvs_cfg.get('deadband_px', 10))
        self._ibvs_lazyband     = float(ibvs_cfg.get('lazyband_outer_px', 0.0))
        self._ibvs_lost_timeout = float(ibvs_cfg.get('tag_lost_timeout_s', 1.0))
        self._ibvs_only         = bool(ibvs_cfg.get('ibvs_only', False))
        self._ibvs_tilt_invert  = bool(ibvs_cfg.get('tilt_invert', False))
        self._ibvs_log_every    = int(ibvs_cfg.get('log_every_n_frames', 5))
        self._ibvs_log_count    = 0
        self._pan_steer_ff_gain  = float(ibvs_cfg.get('pan_steer_ff_gain',   0.0))
        self._steer_ff_alpha     = float(ibvs_cfg.get('steer_ff_ewma_alpha',  0.4))
        self._ff_inhibit_px      = float(ibvs_cfg.get('ff_inhibit_px',       15.0))
        self._ff_ramp_px         = float(ibvs_cfg.get('ff_ramp_px',          20.0))
        self._steer_ff_commanded = 0.0
        self._steer_ff_smooth    = 0.0
        self._ff_drive_sign      = 1

        # IBVS test recording (frames + CSV) — created lazily on first ibvs_test frame
        self._ibvs_rec_dir         = None
        self._ibvs_csv_f           = None
        self._ibvs_csv_w           = None
        self._ibvs_rec_frame_count = 0
        self._ibvs_rec_t0          = None
        self._ibvs_save_every      = 2     # save 1 JPEG per 2 frames (~15 Hz at 30fps)
        self._ibvs_frame_cap       = 2000  # rolling buffer — delete oldest past this
        self._ibvs_saved_files     = []    # ordered list of saved JPEG paths

        self._scan_speed        = float(scan_cfg.get('scan_speed_deg_per_frame', 3.0))
        self._scan_pan_limit    = float(scan_cfg.get('pan_range_deg', 60.0))
        self._scan_tilt_levels  = list(scan_cfg.get('tilt_levels_deg', [-10, 0, 10]))

        self._cc_kp             = float(cc_cfg.get('kp_car', 0.5))
        self._cc_pan_deadband   = float(cc_cfg.get('pan_deadband_deg', 8.0))
        self._cc_speed          = int(cc_cfg.get('car_centering_speed', 20))
        self._cc_tilt_thresh    = float(cc_cfg.get('tilt_elevation_threshold_deg', 30))
        self._cc_decimation     = int(cc_cfg.get('decimation_n', 3))

        lim_cfg = config.get('servo_limits', {})
        self._pan_min       = float(lim_cfg.get('pan_min_deg',       -90))
        self._pan_max       = float(lim_cfg.get('pan_max_deg',        90))
        self._tilt_min      = float(lim_cfg.get('tilt_min_deg',      -35))
        self._tilt_max      = float(lim_cfg.get('tilt_max_deg',       35))
        self._steer_min     = float(lim_cfg.get('steer_min_deg',     -20))
        self._steer_max     = float(lim_cfg.get('steer_max_deg',      20))
        self._pan_max_delta  = float(lim_cfg.get('pan_max_delta_deg',  float('inf')))
        self._tilt_max_delta = float(lim_cfg.get('tilt_max_delta_deg', float('inf')))

        # IBVS per-frame state
        self._pan_angle         = 0.0
        self._tilt_angle        = 0.0
        self._eu_smooth         = 0.0
        self._ev_smooth         = 0.0
        self._ibvs_active       = False
        self._scan_active       = False
        self._centering_active  = False
        self._tag0_lost_t       = None
        self._scan_pan          = 0.0
        self._scan_pan_dir      = 1
        self._scan_tilt_idx     = 0
        self._cc_frame_count    = 0
        self._chase_mode        = 'rat_chase'

        # Summary tracking for _close_ibvs_recording
        self._ibvs_total_detections  = 0
        self._ibvs_total_deadband    = 0
        self._ibvs_max_eu            = 0.0
        self._ibvs_max_ev            = 0.0

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

    def set_drive_direction(self, direction: str):
        self._ff_drive_sign = -1 if direction == 'backward' else 1

    def update_steer_ff(self, steer_deg: float):
        self._steer_ff_commanded = float(steer_deg)

    def start(self, speed: int = 30):
        with self._state_lock:
            if self._state != 'idle':
                return
            self._cycle += 1
            self._speed = max(0, min(100, speed))
            if self._chase_mode == 'world_ibvs':
                self._state = 'world_search'
                self._world_search_start   = time.perf_counter()
                self._world_search_bcast_t = 0.0
            else:
                self._state = 'chasing'

        self._ensure_log_handler()

        try:
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except Exception:
            pass

        if self._chase_mode == 'world_ibvs':
            remaining = int(self._world_search_timeout)
            self.broadcast({'type': 'chase_status', 'active': True,
                            'state': 'world_search', 'countdown': remaining,
                            'chase_mode': self._chase_mode})
        else:
            # Non-world modes start chasing immediately — reset chase state now
            self.pid.reset()
            self._last_seen_time   = None
            self._last_steer       = 0.0
            self._last_log_t       = 0.0
            self._prev_found       = None
            self._last_broadcast_t = 0.0
            self._t_last           = time.perf_counter()
            self._at_stop_dist     = False
            self._world_visible    = False
            self.broadcast({'type': 'chase_status', 'active': True,
                            'state': 'chasing', 'chase_mode': self._chase_mode})
        _marker_logger.info("chase_toggle_on cycle=%d mode=%s", self._cycle, self._chase_mode)

    # Column order for the IBVS CSV — must match both writerow calls and the summary row
    _IBVS_FIELDNAMES = [
        't_s', 'frame', 'tag_detected', 'state',
        'pan_deg', 'tilt_deg',
        'eu_px', 'ev_px', 'eu_s_px', 'ev_s_px', 'err_px',
        'dpan', 'dtilt', 'steer_ff', 'in_deadband',
        'clamped_pan', 'clamped_tilt', 'servo_sent',
        'tag_conf', 'lost_t',
    ]

    def _ensure_ibvs_recording(self):
        """Lazily create the ibvs_frames_* folder and ibvs_log_*.csv for this run."""
        if self._ibvs_rec_dir is not None:
            return
        ts = time.strftime("%H%M%S")
        self._ibvs_rec_dir = os.path.join(self._session_dir, f"ibvs_frames_{ts}")
        os.makedirs(self._ibvs_rec_dir, exist_ok=True)
        csv_path = os.path.join(self._session_dir, f"ibvs_log_{ts}.csv")
        self._ibvs_csv_f = open(csv_path, 'w', newline='')
        self._ibvs_csv_w = csv.DictWriter(self._ibvs_csv_f,
                                          fieldnames=self._IBVS_FIELDNAMES,
                                          extrasaction='ignore')
        self._ibvs_csv_w.writeheader()
        self._ibvs_rec_t0 = time.perf_counter()
        # Reset summary accumulators
        self._ibvs_total_detections = 0
        self._ibvs_total_deadband   = 0
        self._ibvs_max_eu           = 0.0
        self._ibvs_max_ev           = 0.0
        _marker_logger.info("ibvs_recording_start frames_dir=%s csv=%s",
                            self._ibvs_rec_dir, csv_path)

    def _close_ibvs_recording(self):
        if self._ibvs_csv_f is not None:
            if self._ibvs_csv_w is not None and self._ibvs_rec_frame_count > 0:
                # Write summary sentinel row — all numeric summary fields go in 't_s';
                # use a distinguishable marker in 't_s' and a dedicated comment layout.
                self._ibvs_csv_w.writerow({
                    't_s':          'SUMMARY',
                    'frame':        self._ibvs_rec_frame_count,
                    'tag_detected': self._ibvs_total_detections,
                    'state':        'end',
                    'pan_deg':      round(self._pan_angle, 2),
                    'tilt_deg':     round(self._tilt_angle, 2),
                    'eu_px':        round(self._ibvs_max_eu, 1),
                    'ev_px':        round(self._ibvs_max_ev, 1),
                    'eu_s_px':      '',
                    'ev_s_px':      '',
                    'err_px':       '',
                    'dpan':         '',
                    'dtilt':        '',
                    'steer_ff':     '',
                    'in_deadband':  self._ibvs_total_deadband,
                    'clamped_pan':  '',
                    'clamped_tilt': '',
                    'servo_sent':   '',
                    'tag_conf':     '',
                    'lost_t':       '',
                })
            self._ibvs_csv_f.flush()
            self._ibvs_csv_f.close()
            self._ibvs_csv_f = None
            self._ibvs_csv_w = None
            _marker_logger.info(
                "ibvs_recording_stop frames=%d detections=%d deadband=%d "
                "max_eu=%.1f max_ev=%.1f final_pan=%.2f final_tilt=%.2f dir=%s",
                self._ibvs_rec_frame_count,
                self._ibvs_total_detections,
                self._ibvs_total_deadband,
                self._ibvs_max_eu, self._ibvs_max_ev,
                self._pan_angle, self._tilt_angle,
                self._ibvs_rec_dir,
            )
        self._ibvs_rec_dir         = None
        self._ibvs_rec_frame_count = 0
        self._ibvs_rec_t0          = None
        self._ibvs_saved_files     = []

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
        self._close_ibvs_recording()

        if old_state == 'chasing':
            _marker_logger.info("chase_stop cycle=%d reason=toggle_off", self._cycle)

        self.broadcast({'type': 'chase_status', 'active': False, 'state': 'idle',
                        'chase_mode': self._chase_mode})

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

        # Servo-only modes: IBVS without motor control
        if self._chase_mode in ('ibvs_test', 'manual_ibvs'):
            self._ensure_ibvs_recording()
            # Resolve target detection and its confidence
            if self._chase_mode == 'manual_ibvs':
                ibvs_det = next(
                    (d for d in detections
                     if d.decision_margin >= self._conf_threshold), None)
            else:
                ibvs_det = tag0
            ibvs_target = ibvs_det.center if ibvs_det is not None else None
            tag_conf_val = round(float(ibvs_det.decision_margin), 1) if ibvs_det is not None else ''

            if ibvs_target is not None:
                ibvs_data = self._run_ibvs(ibvs_target)
            else:
                self._handle_tag0_lost(t_now)
                ibvs_data = None

            # Seconds since last detection (tag-lost countdown progress)
            lost_t_val = ''
            if ibvs_data is None and self._tag0_lost_t is not None:
                lost_t_val = round(t_now - self._tag0_lost_t, 2)

            # Current chaser state label for the CSV
            with self._state_lock:
                _raw_state = self._state
            if ibvs_data is not None and ibvs_data['in_deadband']:
                csv_state = 'deadband'
            elif ibvs_data is not None:
                csv_state = 'ibvs_active'
            elif self._tag0_lost_t is not None:
                csv_state = 'tag_lost_countdown'
            else:
                csv_state = _raw_state

            self._ibvs_rec_frame_count += 1
            # Save JPEG (throttled, rolling buffer)
            if self._ibvs_rec_dir and self._ibvs_rec_frame_count % self._ibvs_save_every == 0:
                fname = f"frame_{self._ibvs_rec_frame_count:06d}.jpg"
                fpath = os.path.join(self._ibvs_rec_dir, fname)
                ok, buf = cv2.imencode('.jpg', frame)
                if ok:
                    with open(fpath, 'wb') as fp:
                        fp.write(buf.tobytes())
                    self._ibvs_saved_files.append(fpath)
                    if len(self._ibvs_saved_files) > self._ibvs_frame_cap:
                        oldest = self._ibvs_saved_files.pop(0)
                        try:
                            os.remove(oldest)
                        except OSError:
                            pass
            # Write CSV row
            if self._ibvs_csv_w is not None:
                t_rel = time.perf_counter() - self._ibvs_rec_t0
                if ibvs_data is not None:
                    # Update summary accumulators
                    self._ibvs_total_detections += 1
                    if ibvs_data['in_deadband']:
                        self._ibvs_total_deadband += 1
                    self._ibvs_max_eu = max(self._ibvs_max_eu, abs(ibvs_data['eu']))
                    self._ibvs_max_ev = max(self._ibvs_max_ev, abs(ibvs_data['ev']))
                    self._ibvs_csv_w.writerow({
                        't_s':          f'{t_rel:.3f}',
                        'frame':        self._ibvs_rec_frame_count,
                        'tag_detected': 1,
                        'state':        csv_state,
                        'pan_deg':      round(self._pan_angle, 2),
                        'tilt_deg':     round(self._tilt_angle, 2),
                        'eu_px':        ibvs_data['eu'],
                        'ev_px':        ibvs_data['ev'],
                        'eu_s_px':      ibvs_data['eu_s'],
                        'ev_s_px':      ibvs_data['ev_s'],
                        'err_px':       ibvs_data['err_px'],
                        'dpan':         ibvs_data['dpan'],
                        'dtilt':        ibvs_data['dtilt'],
                        'steer_ff':     ibvs_data['steer_ff'],
                        'in_deadband':  int(ibvs_data['in_deadband']),
                        'clamped_pan':  ibvs_data['clamped_pan'],
                        'clamped_tilt': ibvs_data['clamped_tilt'],
                        'servo_sent':   ibvs_data['servo_sent'],
                        'tag_conf':     tag_conf_val,
                        'lost_t':       '',
                    })
                else:
                    self._ibvs_csv_w.writerow({
                        't_s':          f'{t_rel:.3f}',
                        'frame':        self._ibvs_rec_frame_count,
                        'tag_detected': 0,
                        'state':        csv_state,
                        'pan_deg':      round(self._pan_angle, 2),
                        'tilt_deg':     round(self._tilt_angle, 2),
                        'eu_px':        '',
                        'ev_px':        '',
                        'eu_s_px':      '',
                        'ev_s_px':      '',
                        'err_px':       '',
                        'dpan':         '',
                        'dtilt':        '',
                        'steer_ff':     '',
                        'in_deadband':  '',
                        'clamped_pan':  '',
                        'clamped_tilt': '',
                        'servo_sent':   '',
                        'tag_conf':     tag_conf_val,
                        'lost_t':       lost_t_val,
                    })
            self._broadcast_ibvs_status(t_now)
            return

        # rat_chase + world_ibvs: IBVS + motor control
        if tag0 is not None:
            self._run_ibvs(tag0.center)
        elif tag_a is None and self._chase_mode == 'world_ibvs':
            self._run_world_scan()
        else:
            self._handle_tag0_lost(t_now)

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
                            'state': 'world_search', 'countdown': int(remaining),
                            'pan_angle': round(self._pan_angle, 1),
                            'scan_active': self._scan_active})

    def _do_chasing(self, t_now: float, tag0, tag_a, tag_b, pair_valid: bool):
        # Track world-pair visibility changes (world_ibvs only)
        if self._chase_mode == 'world_ibvs':
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

        # Publish TF data (world_ibvs only)
        if self._chase_mode == 'world_ibvs':
            detected_for_tf = [t for t in [tag0, tag_a, tag_b] if t is not None]
            tag0_uv = tag0.center.tolist() if tag0 is not None else None
            self._tf_pub.on_frame(
                t_now, self._cycle, detected_for_tf, bcast_due,
                pair_valid=pair_valid,
                pan_angle_deg=self._pan_angle,
                tilt_angle_deg=self._tilt_angle,
                tag0_pixel_uv=tag0_uv,
                ibvs_active=self._ibvs_active,
                scan_active=self._scan_active,
                car_centering_active=self._centering_active,
            )

        # Car centering — decimated, replaces PID when IBVS is active
        self._cc_frame_count += 1
        if self._ibvs_active and self._cc_frame_count >= self._cc_decimation:
            self._cc_frame_count = 0
            self._run_car_centering()

        world_state = ('world_acquired' if self._world_visible else 'world_lost') \
                      if self._chase_mode == 'world_ibvs' else 'n/a'

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
                                    'world': world_state, 'steer_angle': 0,
                                    'pan_angle': round(self._pan_angle, 1),
                                    'ibvs_active': self._ibvs_active})
            else:
                self._at_stop_dist = False
                if not self._ibvs_active:
                    # PID fallback when IBVS has not yet acquired tag0
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
                                    'world': world_state,
                                    'steer_angle': int(round(self._last_steer)),
                                    'pan_angle': round(self._pan_angle, 1),
                                    'ibvs_active': self._ibvs_active})

            if log_due:
                _logger.debug("detect tag0_found=True dist_cm=%.1f", dist_cm)
                self._last_log_t = t_now
        else:
            if self._prev_found is not False:
                _marker_logger.info("detect tag0_lost")
                self._prev_found = False

            if not self._ibvs_active:
                # PID fallback: coast then stop
                if self._last_seen_time is not None:
                    elapsed = t_now - self._last_seen_time
                    if elapsed < self.tag_lost_timeout:
                        self.px.set_dir_servo_angle(int(round(self._last_steer)))
                        self.px.forward(self._speed)
                    else:
                        self.px.stop()
                        self.pid.reset()
            # else: car centering is driving; IBVS (or world scan) will re-acquire tag0

            if bcast_due:
                self._last_broadcast_t = t_now
                self.broadcast({'type': 'chase_detection', 'found': False,
                                'frame_w': self.cam_w, 'frame_h': self.cam_h})
                self.broadcast({'type': 'chase_status', 'active': True,
                                'state': 'searching', 'world': world_state,
                                'steer_angle': int(round(self._last_steer)),
                                'pan_angle': round(self._pan_angle, 1),
                                'scan_active': self._scan_active,
                                'ibvs_active': self._ibvs_active})

            if log_due:
                _logger.debug("detect tag0_found=False")
                self._last_log_t = t_now

    def set_ibvs_only(self, enabled: bool):
        self._ibvs_only = enabled

    @property
    def chase_mode(self) -> str:
        return self._chase_mode

    def set_chase_mode(self, mode: str):
        VALID = {'world_ibvs', 'rat_chase', 'ibvs_test', 'manual_ibvs'}
        if mode in VALID:
            self._chase_mode = mode

    def simulate_frame(self, frame) -> dict:
        """Stateless: run detection + IBVS on one frame. Does not modify any state."""
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self._fx, self._fy, self._cx, self._cy],
            tag_size=self.tag_size_m,
        )

        cx = self.cam_w / 2.0
        cy = self.cam_h / 2.0

        det_list = []
        for d in detections:
            if d.decision_margin < self._conf_threshold:
                continue
            eu = d.center[0] - cx
            ev = d.center[1] - cy
            det_list.append({
                'tag_id':          d.tag_id,
                'center':          [round(float(d.center[0]), 1), round(float(d.center[1]), 1)],
                'corners':         [[round(float(c[0]), 1), round(float(c[1]), 1)] for c in d.corners],
                'decision_margin': round(float(d.decision_margin), 1),
                'eu':              round(float(eu), 1),
                'ev':              round(float(ev), 1),
                'err_px':          round(float(np.hypot(eu, ev)), 1),
            })

        tag0 = next((d for d in detections
                     if d.tag_id == self._tag_id_chase
                     and d.decision_margin >= self._conf_threshold), None)
        ibvs_result = None
        if tag0 is not None:
            eu = tag0.center[0] - cx
            ev = tag0.center[1] - cy
            eu_s = self._ibvs_alpha * eu + (1.0 - self._ibvs_alpha) * self._eu_smooth
            ev_s = self._ibvs_alpha * ev + (1.0 - self._ibvs_alpha) * self._ev_smooth
            in_db = bool(abs(eu_s) <= self._ibvs_deadband and abs(ev_s) <= self._ibvs_deadband)
            dpan = dtilt = sim_steer_ff = 0.0
            if not in_db:
                tilt_sign  = 1.0 if self._ibvs_tilt_invert else -1.0
                err_s = float(np.hypot(eu_s, ev_s))
                if self._ibvs_lazyband > self._ibvs_deadband and err_s < self._ibvs_lazyband:
                    lazy_scale = (err_s - self._ibvs_deadband) / (self._ibvs_lazyband - self._ibvs_deadband)
                else:
                    lazy_scale = 1.0
                dpan  =  lazy_scale * self._ibvs_kp_pan  * eu_s
                dtilt =  lazy_scale * tilt_sign * self._ibvs_kp_tilt * ev_s
                if self._pan_steer_ff_gain != 0.0 and self._steer_ff_smooth != 0.0:
                    ff_gate = max(0.0, min(1.0,
                        (abs(eu_s) - self._ff_inhibit_px) / max(1.0, self._ff_ramp_px)))
                    sim_steer_ff = (-self._pan_steer_ff_gain
                                    * self._steer_ff_smooth * self._ff_drive_sign * ff_gate)
                    dpan += sim_steer_ff
            dpan  = float(np.clip(dpan,  -self._pan_max_delta,  self._pan_max_delta))
            dtilt = float(np.clip(dtilt, -self._tilt_max_delta, self._tilt_max_delta))
            ibvs_result = {
                'eu': round(float(eu), 1), 'ev': round(float(ev), 1),
                'eu_s': round(float(eu_s), 1), 'ev_s': round(float(ev_s), 1),
                'in_deadband': in_db,
                'delta_pan':   round(float(dpan),  2),
                'delta_tilt':  round(float(dtilt), 2),
                'steer_ff':    round(float(sim_steer_ff), 3),
                'pan_start':   round(self._pan_angle,  1),
                'tilt_start':  round(self._tilt_angle, 1),
                'pan_cmd':     round(float(np.clip(self._pan_angle  + dpan,  self._pan_min,  self._pan_max)),  1),
                'tilt_cmd':    round(float(np.clip(self._tilt_angle + dtilt, self._tilt_min, self._tilt_max)), 1),
            }

        ann = frame.copy()
        cv2.line(ann, (int(cx) - 20, int(cy)), (int(cx) + 20, int(cy)), (200, 200, 200), 1)
        cv2.line(ann, (int(cx), int(cy) - 20), (int(cx), int(cy) + 20), (200, 200, 200), 1)
        for d in detections:
            if d.decision_margin < self._conf_threshold:
                continue
            pts = np.array(d.corners, dtype=np.int32).reshape((-1, 1, 2))
            col = (0, 255, 0) if d.tag_id == self._tag_id_chase else (255, 165, 0)
            cv2.polylines(ann, [pts], isClosed=True, color=col, thickness=2)
            tc = (int(d.center[0]), int(d.center[1]))
            cv2.circle(ann, tc, 4, (0, 255, 255), -1)
            cv2.line(ann, (int(cx), int(cy)), tc, (0, 80, 255), 1)
            eu_lbl = d.center[0] - cx
            ev_lbl = d.center[1] - cy
            cv2.putText(ann, f"T{d.tag_id} eu={eu_lbl:+.0f} ev={ev_lbl:+.0f}",
                        (tc[0] + 6, max(tc[1] - 6, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        if ibvs_result:
            label = ("DEADBAND" if ibvs_result['in_deadband']
                     else f"dpan={ibvs_result['delta_pan']:+.2f}  dtilt={ibvs_result['delta_tilt']:+.2f}")
            cv2.putText(ann, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return {'detections': det_list, 'ibvs': ibvs_result, 'annotated_frame': ann}

    def _broadcast_ibvs_status(self, t_now: float):
        """Throttled status broadcast for servo-only modes."""
        if t_now - self._last_broadcast_t < 0.1:
            return
        self._last_broadcast_t = t_now
        self.broadcast({
            'type': 'chase_status',
            'active': True,
            'state': 'ibvs_lock',
            'chase_mode': self._chase_mode,
            'pan_angle': round(self._pan_angle, 1),
            'tilt_angle': round(self._tilt_angle, 1),
            'ibvs_active': self._ibvs_active,
            'scan_active': False,
            'car_centering_active': False,
        })

    def _run_ibvs(self, center_px):
        """Track tag: center it in the frame via pan/tilt servo commands."""
        cx = self.cam_w / 2.0
        cy = self.cam_h / 2.0
        eu = center_px[0] - cx
        ev = center_px[1] - cy

        alpha = self._ibvs_alpha
        self._eu_smooth = alpha * eu + (1.0 - alpha) * self._eu_smooth
        self._ev_smooth = alpha * ev + (1.0 - alpha) * self._ev_smooth

        in_deadband = (abs(self._eu_smooth) <= self._ibvs_deadband
                       and abs(self._ev_smooth) <= self._ibvs_deadband)

        # Steer ff EWMA runs every frame so _steer_ff_smooth decays when steer returns to 0
        self._steer_ff_smooth = (self._steer_ff_alpha * self._steer_ff_commanded
                                 + (1 - self._steer_ff_alpha) * self._steer_ff_smooth)

        delta_pan = delta_tilt = steer_ff_delta = 0.0
        clamped_pan = clamped_tilt = False
        servo_sent  = False
        if not in_deadband:
            tilt_sign  = 1.0 if self._ibvs_tilt_invert else -1.0
            err_smooth = float(np.hypot(self._eu_smooth, self._ev_smooth))
            if self._ibvs_lazyband > self._ibvs_deadband and err_smooth < self._ibvs_lazyband:
                lazy_scale = (err_smooth - self._ibvs_deadband) / (self._ibvs_lazyband - self._ibvs_deadband)
            else:
                lazy_scale = 1.0
            delta_pan  =  lazy_scale * self._ibvs_kp_pan  * self._eu_smooth
            delta_tilt =  lazy_scale * tilt_sign * self._ibvs_kp_tilt * self._ev_smooth
            if self._pan_steer_ff_gain != 0.0 and self._steer_ff_smooth != 0.0:
                ff_gate = max(0.0, min(1.0,
                    (abs(self._eu_smooth) - self._ff_inhibit_px) / max(1.0, self._ff_ramp_px)))
                steer_ff_delta = (-self._pan_steer_ff_gain
                                   * self._steer_ff_smooth * self._ff_drive_sign * ff_gate)
                delta_pan += steer_ff_delta
            delta_pan  = float(np.clip(delta_pan,  -self._pan_max_delta,  self._pan_max_delta))
            delta_tilt = float(np.clip(delta_tilt, -self._tilt_max_delta, self._tilt_max_delta))
            unclamped_pan  = self._pan_angle  + delta_pan
            unclamped_tilt = self._tilt_angle + delta_tilt
            new_pan  = float(np.clip(unclamped_pan,  self._pan_min,  self._pan_max))
            new_tilt = float(np.clip(unclamped_tilt, self._tilt_min, self._tilt_max))
            clamped_pan  = (new_pan  != unclamped_pan)
            clamped_tilt = (new_tilt != unclamped_tilt)
            # Anti-windup: only apply if clamp didn't absorb the full delta
            if new_pan != self._pan_angle or new_tilt != self._tilt_angle:
                self._pan_angle  = new_pan
                self._tilt_angle = new_tilt
                servo_sent = True
                try:
                    self.px.set_cam_pan_angle(self._pan_angle)
                    self.px.set_cam_tilt_angle(self._tilt_angle)
                except Exception as exc:
                    _marker_logger.warning("ibvs servo error: %s", exc)

        self._ibvs_log_count += 1
        if self._ibvs_log_count >= self._ibvs_log_every:
            self._ibvs_log_count = 0
            _marker_logger.debug(
                "ibvs eu=%+.0f ev=%+.0f eu_s=%+.1f ev_s=%+.1f "
                "dpan=%+.2f dtilt=%+.2f pan=%.1f tilt=%.1f err=%.0fpx%s%s%s%s",
                eu, ev, self._eu_smooth, self._ev_smooth,
                delta_pan, delta_tilt,
                self._pan_angle, self._tilt_angle,
                float(np.hypot(eu, ev)),
                " DB" if in_deadband else "",
                " CLAMP_PAN" if clamped_pan else "",
                " CLAMP_TILT" if clamped_tilt else "",
                "" if servo_sent or in_deadband else " NO_SEND",
            )

        self._ibvs_active  = True
        self._scan_active  = False
        self._tag0_lost_t  = None
        return {
            'eu':           round(eu, 1),
            'ev':           round(ev, 1),
            'eu_s':         round(self._eu_smooth, 1),
            'ev_s':         round(self._ev_smooth, 1),
            'err_px':       round(float(np.hypot(eu, ev)), 1),
            'dpan':         round(delta_pan, 3),
            'dtilt':        round(delta_tilt, 3),
            'steer_ff':     round(steer_ff_delta, 3),
            'in_deadband':  in_deadband,
            'clamped_pan':  int(clamped_pan),
            'clamped_tilt': int(clamped_tilt),
            'servo_sent':   int(servo_sent),
        }

    def _run_world_scan(self):
        """Raster sweep to re-acquire the world tag when not visible."""
        self._scan_pan += self._scan_pan_dir * self._scan_speed

        if self._scan_pan >= self._scan_pan_limit:
            self._scan_pan = self._scan_pan_limit
            self._scan_pan_dir  = -1
            self._scan_tilt_idx = (self._scan_tilt_idx + 1) % len(self._scan_tilt_levels)
        elif self._scan_pan <= -self._scan_pan_limit:
            self._scan_pan = -self._scan_pan_limit
            self._scan_pan_dir  = 1
            self._scan_tilt_idx = (self._scan_tilt_idx + 1) % len(self._scan_tilt_levels)

        scan_tilt = self._scan_tilt_levels[self._scan_tilt_idx]
        try:
            self.px.set_cam_pan_angle(self._scan_pan)
            self.px.set_cam_tilt_angle(scan_tilt)
        except Exception:
            pass
        self._pan_angle    = self._scan_pan
        self._tilt_angle   = scan_tilt
        self._scan_active  = True
        self._ibvs_active  = False

    def _handle_tag0_lost(self, t_now: float):
        """Hold servos when tag is not visible; reset to neutral after timeout."""
        if self._tag0_lost_t is None:
            self._tag0_lost_t = t_now
            _marker_logger.debug("ibvs tag_lost timer start pan=%.1f tilt=%.1f",
                                 self._pan_angle, self._tilt_angle)
        elif t_now - self._tag0_lost_t > self._ibvs_lost_timeout:
            elapsed = t_now - self._tag0_lost_t
            _marker_logger.info("ibvs servo_reset pan→0 tilt→0 lost=%.1fs", elapsed)
            try:
                self.px.set_cam_pan_angle(0)
                self.px.set_cam_tilt_angle(0)
            except Exception as exc:
                _marker_logger.warning("ibvs servo error on reset: %s", exc)
            self._pan_angle   = 0.0
            self._tilt_angle  = 0.0
            self._eu_smooth   = 0.0
            self._ev_smooth   = 0.0
            self._ibvs_active = False
            self._tag0_lost_t = None  # prevent re-firing every frame until tag reappears
        self._scan_active = False

    def _run_car_centering(self):
        """Slow, pan-angle-based car steering to null pan deflection."""
        if self._ibvs_only:
            self._centering_active = False
            return

        pan = self._pan_angle
        if abs(pan) < self._cc_pan_deadband:
            try:
                self.px.set_dir_servo_angle(0)
            except Exception:
                pass
            self._steer_ff_commanded = 0.0
            self._centering_active = False
            return

        steer = float(np.clip(self._cc_kp * pan, self._steer_min, self._steer_max))
        self._steer_ff_commanded = steer
        try:
            self.px.set_dir_servo_angle(int(round(steer)))
        except Exception:
            pass

        if abs(self._tilt_angle) < self._cc_tilt_thresh:
            try:
                self.px.forward(self._cc_speed)
            except Exception:
                pass
        self._centering_active = True

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
