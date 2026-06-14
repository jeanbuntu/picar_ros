import asyncio
import cv2
import http.server
import json
import logging
import os
import signal
import socketserver

import sys
import threading
import time
from datetime import datetime

import uvicorn
import yaml
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from picarx import Picarx

try:
    from robot_hat import ADC as _ADC
    _battery_adc = _ADC("A4")
    _battery_available = True
except Exception:
    _battery_available = False

_PROJ_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJ_ROOT not in sys.path:
    sys.path.insert(0, _PROJ_ROOT)

from tag_chaser.v2_world_tf.chaser import TagChaser
from tag_chaser.v2_world_tf.tracker import ManualTracker
from tag_chaser.v1_camera_lock.chaser import TagChaser as TagChaserV1

BASE_DIR         = os.path.dirname(os.path.abspath(__file__))
WATCHDOG_TIMEOUT = 5.0
_CAM_W, _CAM_H   = 640, 480

app = FastAPI()
app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "static")), name="static")

px:      Picarx        = None
chaser:  TagChaser     = None
tracker: ManualTracker = None
_picam2                = None

_sensor      = {"distance": -1.0, "grayscale": [0, 0, 0], "battery": None}
_sensor_lock = threading.Lock()
_rec_state      = "stopped"
_rec_last_video = ""

# MJPEG frame buffer
_frame_lock  = threading.Lock()
_frame_bytes = b''

# Video writer
_rec_writer      = None
_rec_writer_lock = threading.Lock()

_chaser_frame_log_t  = 0.0
_frame_diag_logged   = False

# Per-connection send queues — broadcasts go here instead of directly to ws
_ws_queues: dict = {}   # ws → asyncio.Queue
_loop: asyncio.AbstractEventLoop = None

# Global heartbeat clock — any connection's heartbeat resets this, preventing
# orphaned-connection watchdogs from firing after a browser reconnect.
_last_hb: list = [0.0]

PHOTO_DIR = "/home/jvpicar/Pictures/picar-x"
VIDEO_DIR = "/home/jvpicar/Videos/picar-x"
LOG_DIR   = "/home/jvpicar/picar_ros/logs"



_logger = logging.getLogger("picarx")

_session_dir:    str  = ''
_chase_config_v1: dict = {}


# ── MJPEG server ──────────────────────────────────────────────────────────────

def _set_frame(data: bytes):
    global _frame_bytes
    with _frame_lock:
        _frame_bytes = data


class _MJPEGHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    data = _frame_bytes
                if data:
                    self.wfile.write(
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(data)).encode() + b'\r\n'
                        b'\r\n' + data + b'\r\n'
                    )
                time.sleep(0.033)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


class _ReuseAddrServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True


def _start_mjpeg_server(port: int = 9000):
    server = _ReuseAddrServer(('', port), _MJPEGHandler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    _logger.info("MJPEG stream at :%d", port)


# ── Camera capture loop ───────────────────────────────────────────────────────

def _capture_loop():
    while True:
        try:
            raw   = _picam2.capture_array()
            frame = raw

            global _frame_diag_logged
            if not _frame_diag_logged:
                _frame_diag_logged = True
                _logger.info("frame_diag shape=%s dtype=%s ch0_mean=%.1f ch1_mean=%.1f ch2_mean=%.1f",
                             raw.shape, raw.dtype,
                             float(raw[:, :, 0].mean()),
                             float(raw[:, :, 1].mean()),
                             float(raw[:, :, 2].mean()))

            ok, buf = cv2.imencode('.jpg', frame)
            if ok:
                _set_frame(buf.tobytes())

            with _rec_writer_lock:
                if _rec_writer is not None:
                    _rec_writer.write(frame)

            if chaser is not None and chaser.is_running():
                global _chaser_frame_log_t
                _now_mt = time.monotonic()
                if _now_mt - _chaser_frame_log_t >= 30.0:
                    _logger.debug("capture->chaser active")
                    _chaser_frame_log_t = _now_mt
                chaser.process_frame(frame)
            elif tracker is not None and tracker.is_running():
                tracker.process_frame(frame)

        except Exception as e:
            _logger.error("capture_loop error: %s", e)
            time.sleep(0.1)


# ── Chaser broadcast ──────────────────────────────────────────────────────────

async def _broadcast_coro(msg: dict):
    dead = set()
    for ws_obj, q in list(_ws_queues.items()):
        try:
            q.put_nowait(msg)
        except Exception:
            dead.add(ws_obj)
    for ws_obj in dead:
        _ws_queues.pop(ws_obj, None)


def broadcast_chase(msg: dict):
    if _loop and not _loop.is_closed():
        asyncio.run_coroutine_threadsafe(_broadcast_coro(msg), _loop)


@app.on_event("startup")
async def _on_startup():
    global _loop
    _loop = asyncio.get_running_loop()


# ── Sensor loop ───────────────────────────────────────────────────────────────

def _sensor_loop():
    _batt_tick = 24
    while True:
        try:
            dist = px.get_distance()
            gs   = px.get_grayscale_data()
            with _sensor_lock:
                _sensor["distance"]  = round(float(dist), 1)
                _sensor["grayscale"] = [int(v) for v in gs]
        except Exception as e:
            _logger.error("sensor_loop error: %s", e)
        _batt_tick += 1
        if _battery_available and _batt_tick >= 25:
            _batt_tick = 0
            try:
                v = round(_battery_adc.read_voltage() * 3, 2)
                with _sensor_lock:
                    _sensor["battery"] = v
            except Exception:
                pass
        time.sleep(0.2)


# ── HTTP routes ───────────────────────────────────────────────────────────────

@app.get("/")
async def index():
    return FileResponse(os.path.join(BASE_DIR, "static", "index.html"))


@app.get("/download/photo/{filename}")
async def dl_photo(filename: str):
    path = os.path.join(PHOTO_DIR, filename)
    if not os.path.isfile(path):
        raise HTTPException(status_code=404, detail="Not found")
    return FileResponse(path, media_type="image/jpeg", filename=filename)


@app.get("/download/video/{filename}")
async def dl_video(filename: str):
    path = os.path.join(VIDEO_DIR, filename)
    if not os.path.isfile(path):
        raise HTTPException(status_code=404, detail="Not found")
    return FileResponse(path, media_type="video/x-msvideo", filename=filename)


# ── Graceful shutdown ─────────────────────────────────────────────────────────

async def _graceful_shutdown():
    _logger.info("Graceful shutdown initiated — stopping hardware")
    if chaser and chaser.is_running():
        await asyncio.to_thread(chaser.stop)
    if tracker and tracker.is_running():
        await asyncio.to_thread(tracker.stop)
    try:
        px.stop()
        px.set_dir_servo_angle(0)
        px.set_cam_pan_angle(0)
        px.set_cam_tilt_angle(0)
    except Exception as e:
        _logger.error("Hardware stop error: %s", e)

    with _rec_writer_lock:
        if _rec_writer is not None:
            _rec_writer.release()

    try:
        _picam2.stop()
    except Exception:
        pass

    await asyncio.sleep(0.2)  # let shutdown_ack drain to client before killing

    for h in list(_logger.handlers):
        h.flush()
        h.close()

    os.kill(os.getpid(), signal.SIGTERM)


# ── WebSocket ─────────────────────────────────────────────────────────────────

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    global _rec_state, _rec_last_video, _rec_writer, _loop
    if _loop is None or _loop.is_closed():
        _loop = asyncio.get_running_loop()

    await ws.accept()

    out_q: asyncio.Queue = asyncio.Queue()
    _ws_queues[ws] = out_q

    client_ip = ws.client.host if ws.client else "unknown"
    _logger.info("WebSocket connected from %s", client_ip)

    chase_active = chaser.is_running() if chaser else False
    out_q.put_nowait({
        "type":   "chase_status",
        "active": chase_active,
        "state":  "chasing" if chase_active else "idle",
    })

    async def _recv():
        global _rec_state, _rec_last_video, _rec_writer, chaser, tracker
        try:
            while True:
                msg = await ws.receive_json()
                cmd = msg.get("cmd")

                if cmd == "drive":
                    if chaser and chaser.is_running():
                        continue
                    direction = msg.get("direction", "stop")
                    speed     = max(0, min(100, int(msg.get("speed", 50))))
                    _logger.info("drive direction=%s speed=%d", direction, speed)
                    if direction == "forward":
                        await asyncio.to_thread(px.forward, speed)
                    elif direction == "backward":
                        await asyncio.to_thread(px.backward, speed)
                    else:
                        await asyncio.to_thread(px.stop)

                elif cmd == "steer":
                    if chaser and chaser.is_running():
                        continue
                    angle = max(-30, min(30, int(msg.get("angle", 0))))
                    _logger.info("steer angle=%d", angle)
                    await asyncio.to_thread(px.set_dir_servo_angle, angle)

                elif cmd == "gimbal":
                    if chaser and chaser.is_running():
                        continue
                    axis  = msg.get("axis", "pan")
                    angle = int(msg.get("angle", 0))
                    _logger.info("gimbal axis=%s angle=%d", axis, angle)
                    if axis == "pan":
                        angle = max(-90, min(90, angle))
                        await asyncio.to_thread(px.set_cam_pan_angle, angle)
                    else:
                        angle = max(-35, min(65, angle))
                        await asyncio.to_thread(px.set_cam_tilt_angle, angle)

                elif cmd == "tag_chase":
                    action = msg.get("action", "stop")
                    if action == "start":
                        speed = max(0, min(100, int(msg.get("speed", 30))))
                        _logger.info("tag_chase start speed=%d", speed)
                        if chaser:
                            await asyncio.to_thread(chaser.start, speed)
                    elif action == "stop":
                        _logger.info("tag_chase stop")
                        if chaser:
                            await asyncio.to_thread(chaser.stop)
                        await asyncio.to_thread(px.stop)
                        await asyncio.to_thread(px.set_dir_servo_angle, 0)
                    elif action == "switch_to_v1":
                        speed = max(0, min(100, int(msg.get("speed", 30))))
                        _logger.info("tag_chase switch_to_v1 speed=%d", speed)
                        if chaser and chaser.is_running():
                            await asyncio.to_thread(chaser.stop)
                        await asyncio.sleep(0.05)
                        chaser = TagChaserV1(px, _chase_config_v1,
                                             broadcast_fn=broadcast_chase)
                        await asyncio.to_thread(chaser.start, speed)
                    elif action == "cancel":
                        _logger.info("tag_chase cancel — world_not_found popup dismissed")
                        out_q.put_nowait({'type': 'chase_status',
                                          'active': False, 'state': 'idle'})

                elif cmd == "manual_track":
                    action = msg.get("action", "stop")
                    _logger.info("manual_track action=%s tracker_ready=%s from %s",
                                 action, tracker is not None, client_ip)
                    if action == "start":
                        if chaser and chaser.is_running():
                            _logger.info("manual_track: stopping active chaser first")
                            await asyncio.to_thread(chaser.stop)
                            await asyncio.to_thread(px.stop)
                        if tracker:
                            await asyncio.to_thread(tracker.start)
                            _logger.info("manual_track: tracker started cycle=%d", tracker._cycle)
                        else:
                            _logger.error("manual_track: tracker is None — skipped")
                    elif action == "stop":
                        if tracker:
                            await asyncio.to_thread(tracker.stop)
                            _logger.info("manual_track: tracker stopped")

                elif cmd == "photo":
                    ts    = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
                    fname = f"photo_{ts}.jpg"
                    os.makedirs(PHOTO_DIR, exist_ok=True)
                    path = os.path.join(PHOTO_DIR, fname)
                    with _frame_lock:
                        data = _frame_bytes
                    if data:
                        with open(path, 'wb') as f:
                            f.write(data)
                    _logger.info("photo saved file=%s", fname)
                    out_q.put_nowait({"type": "photo_saved", "filename": fname})

                elif cmd == "rec_toggle":
                    if _rec_state == "stopped":
                        ts    = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
                        vname = f"video_{ts}.avi"
                        os.makedirs(VIDEO_DIR, exist_ok=True)
                        path   = os.path.join(VIDEO_DIR, vname)
                        fourcc = cv2.VideoWriter_fourcc(*'XVID')
                        with _rec_writer_lock:
                            _rec_writer = cv2.VideoWriter(path, fourcc, 30, (_CAM_W, _CAM_H))
                        _rec_state      = "recording"
                        _rec_last_video = vname
                        _logger.info("rec_start file=%s", vname)
                        out_q.put_nowait({"type": "rec_state", "state": "recording"})
                    else:
                        with _rec_writer_lock:
                            if _rec_writer is not None:
                                _rec_writer.release()
                                _rec_writer = None
                        _rec_state = "stopped"
                        _logger.info("rec_stop file=%s", _rec_last_video)
                        out_q.put_nowait({"type": "rec_state",   "state": "stopped"})
                        out_q.put_nowait({"type": "rec_stopped", "filename": _rec_last_video})

                elif cmd == "kill":
                    _logger.info("kill command from %s", client_ip)
                    if chaser and chaser.is_running():
                        await asyncio.to_thread(chaser.stop)
                    if tracker and tracker.is_running():
                        await asyncio.to_thread(tracker.stop)
                    await asyncio.to_thread(px.stop)
                    await asyncio.to_thread(px.set_dir_servo_angle, 0)
                    out_q.put_nowait({"type": "kill_confirmed"})

                elif cmd == "heartbeat":
                    _logger.debug("heartbeat from %s", client_ip)
                    _last_hb[0] = time.monotonic()

                elif cmd == "shutdown":
                    _logger.info("shutdown command from %s", client_ip)
                    out_q.put_nowait({"type": "shutdown_ack"})
                    asyncio.create_task(_graceful_shutdown())

                elif cmd == "mode":
                    _logger.info("mode value=%s from %s", msg.get("value"), client_ip)

                elif cmd == "detect":
                    _logger.info("detect_toggle value=%s from %s", msg.get("value"), client_ip)

                elif cmd:
                    _logger.warning("unhandled cmd=%s from %s", cmd, client_ip)

        except (WebSocketDisconnect, RuntimeError):
            pass
        except Exception as e:
            _logger.error("_recv unhandled exception (cmd=%s): %s", cmd, e, exc_info=True)

    async def _send():
        try:
            while True:
                msg = await out_q.get()
                try:
                    await ws.send_json(msg)
                except WebSocketDisconnect:
                    return
                except Exception as e:
                    _logger.debug("ws send error (discarding msg): %s", e)
        except Exception:
            pass

    async def _push_sensors():
        try:
            while True:
                with _sensor_lock:
                    data = dict(_sensor)
                out_q.put_nowait({
                    "type":         "sensors",
                    "distance":     data["distance"],
                    "grayscale":    data["grayscale"],
                    "battery":      data["battery"],
                    "battery_warn": data["battery"] is not None and data["battery"] < 6.5,
                })
                await asyncio.sleep(0.2)
        except Exception:
            pass

    _last_hb[0] = time.monotonic()

    async def _watchdog():
        _fired = False
        while True:
            await asyncio.sleep(1)
            if time.monotonic() - _last_hb[0] > WATCHDOG_TIMEOUT:
                if not _fired:
                    _logger.warning(
                        "Watchdog fired — no heartbeat for %.1fs, stopping motors",
                        WATCHDOG_TIMEOUT)
                    _fired = True
                px.stop()
                try:
                    px.set_dir_servo_angle(0)
                except Exception:
                    pass
            else:
                if _fired:
                    _logger.info("Watchdog reset — heartbeat restored from %s", client_ip)
                    _fired = False

    recv_task    = asyncio.create_task(_recv())
    send_task    = asyncio.create_task(_send())
    sensor_task  = asyncio.create_task(_push_sensors())
    watchdog_task = asyncio.create_task(_watchdog())
    try:
        await asyncio.wait(
            {recv_task, send_task, sensor_task, watchdog_task},
            return_when=asyncio.FIRST_COMPLETED,
        )
        for t in (recv_task, send_task, sensor_task, watchdog_task):
            t.cancel()
    finally:
        _ws_queues.pop(ws, None)
        _logger.info("WebSocket disconnected from %s", client_ip)
        if not _ws_queues and not (chaser and chaser.is_running()) and not (tracker and tracker.is_running()):
            px.stop()


# ── Startup ───────────────────────────────────────────────────────────────────

def main():
    global px, chaser, tracker, _picam2, _session_dir, _chase_config_v1

    os.makedirs(LOG_DIR, exist_ok=True)
    log_ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
    _session_dir = os.path.join(LOG_DIR, f"pi_session_{log_ts}")
    os.makedirs(_session_dir, exist_ok=True)
    log_path = os.path.join(_session_dir, "master.log")

    _logger.setLevel(logging.DEBUG)
    fh = logging.FileHandler(log_path)
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d  %(levelname)-7s  %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    ))
    sh = logging.StreamHandler()
    sh.setLevel(logging.DEBUG)
    sh.setFormatter(fh.formatter)
    _logger.addHandler(fh)
    _logger.addHandler(sh)
    _logger.info("Server starting | session=%s", _session_dir)

    px = Picarx()
    _logger.info("Picarx initialized")

    _chase_config_v2_path = os.path.join(
        _PROJ_ROOT, 'tag_chaser', 'v2_world_tf', 'config.yaml')
    with open(_chase_config_v2_path) as f:
        _chase_config_v2 = yaml.safe_load(f)

    _chase_config_v1_path = os.path.join(
        _PROJ_ROOT, 'tag_chaser', 'v1_camera_lock', 'config.yaml')
    with open(_chase_config_v1_path) as f:
        _chase_config_v1 = yaml.safe_load(f)

    chaser = TagChaser(px, _chase_config_v2, broadcast_fn=broadcast_chase,
                       session_dir=_session_dir)
    _logger.info("TagChaser v2 initialized")

    tracker = ManualTracker(px, _chase_config_v2, broadcast_fn=broadcast_chase,
                            session_dir=_session_dir)
    _logger.info("ManualTracker initialized")

    from picamera2 import Picamera2
    _picam2 = Picamera2()
    _picam2.configure(_picam2.create_video_configuration(
        main={'format': 'RGB888', 'size': (_CAM_W, _CAM_H)}
    ))
    _picam2.start()
    time.sleep(0.5)
    _logger.info("Picamera2 started at %dx%d", _CAM_W, _CAM_H)

    _start_mjpeg_server(9000)
    _logger.info("Camera stream at http://0.0.0.0:9000")

    for d in (PHOTO_DIR, VIDEO_DIR, LOG_DIR):
        os.makedirs(d, exist_ok=True)

    threading.Thread(target=_sensor_loop,  daemon=True).start()
    threading.Thread(target=_capture_loop, daemon=True).start()
    _logger.info("Sensor and capture loops started")

    _logger.info("Dashboard listening at http://0.0.0.0:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")


if __name__ == "__main__":
    main()
