import asyncio
import logging
import os
import signal
import subprocess
import threading
import time
from datetime import datetime

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from picarx import Picarx
from vilib import Vilib
try:
    from robot_hat import ADC as _ADC
    _battery_adc = _ADC("A4")
    _battery_available = True
except Exception:
    _battery_available = False

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WATCHDOG_TIMEOUT = 5.0

app = FastAPI()
app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "static")), name="static")

px: Picarx = None
_sensor = {"distance": -1.0, "grayscale": [0, 0, 0], "battery": None}
_sensor_lock = threading.Lock()
_rec_state = "stopped"
_rec_last_video = ""

try:
    _username = os.getlogin()
except Exception:
    _username = "jvpicar"

PHOTO_DIR = f"/home/{_username}/Pictures/picar-x"
VIDEO_DIR = f"/home/{_username}/Videos/picar-x"
LOG_DIR   = f"/home/{_username}/picar-x/logs"

UBUNTU_LOG_DEST = "jeano@192.168.1.250:/home/jeano/picar_ros/dashboard/sessionlogs/"

_logger = logging.getLogger("picarx")


def _sensor_loop():
    _batt_tick = 24
    while True:
        try:
            dist = px.get_distance()
            gs = px.get_grayscale_data()
            with _sensor_lock:
                _sensor["distance"] = round(float(dist), 1)
                _sensor["grayscale"] = [int(v) for v in gs]
            _logger.debug("sensor distance=%.1f grayscale=%s", _sensor["distance"], _sensor["grayscale"])
        except Exception as e:
            _logger.error("sensor_loop error: %s", e)
        _batt_tick += 1
        if _battery_available and _batt_tick >= 25:
            _batt_tick = 0
            try:
                v = round(_battery_adc.read_voltage() * 3, 2)
                with _sensor_lock:
                    _sensor["battery"] = v
                _logger.debug("battery %.2fV", v)
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
    try:
        px.stop()
        px.set_dir_servo_angle(0)
        px.set_cam_pan_angle(0)
        px.set_cam_tilt_angle(0)
    except Exception as e:
        _logger.error("Hardware stop error: %s", e)

    _logger.info("Transferring logs to Ubuntu: %s", UBUNTU_LOG_DEST)
    for h in _logger.handlers:
        h.flush()

    try:
        result = subprocess.run(
            ["scp", "-r", LOG_DIR + "/", UBUNTU_LOG_DEST],
            timeout=30,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            _logger.info("Log transfer complete")
        else:
            _logger.error("SCP failed (rc=%d): %s", result.returncode, result.stderr.strip())
    except Exception as e:
        _logger.error("SCP exception: %s", e)

    for h in list(_logger.handlers):
        h.flush()
        h.close()

    os.kill(os.getpid(), signal.SIGTERM)


# ── WebSocket ─────────────────────────────────────────────────────────────────

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    global _rec_state, _rec_last_video
    await ws.accept()
    client_ip = ws.client.host if ws.client else "unknown"
    _logger.info("WebSocket connected from %s", client_ip)

    async def _recv():
        global _rec_state, _rec_last_video
        try:
            while True:
                msg = await ws.receive_json()
                cmd = msg.get("cmd")

                if cmd == "drive":
                    direction = msg.get("direction", "stop")
                    speed = max(0, min(100, int(msg.get("speed", 50))))
                    _logger.info("drive direction=%s speed=%d", direction, speed)
                    if direction == "forward":
                        await asyncio.to_thread(px.forward, speed)
                    elif direction == "backward":
                        await asyncio.to_thread(px.backward, speed)
                    else:
                        await asyncio.to_thread(px.stop)

                elif cmd == "steer":
                    angle = max(-30, min(30, int(msg.get("angle", 0))))
                    _logger.info("steer angle=%d", angle)
                    await asyncio.to_thread(px.set_dir_servo_angle, angle)

                elif cmd == "gimbal":
                    axis = msg.get("axis", "pan")
                    angle = int(msg.get("angle", 0))
                    _logger.info("gimbal axis=%s angle=%d", axis, angle)
                    if axis == "pan":
                        angle = max(-90, min(90, angle))
                        await asyncio.to_thread(px.set_cam_pan_angle, angle)
                    else:
                        angle = max(-35, min(65, angle))
                        await asyncio.to_thread(px.set_cam_tilt_angle, angle)

                elif cmd == "photo":
                    ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
                    name = f"photo_{ts}"
                    os.makedirs(PHOTO_DIR, exist_ok=True)
                    Vilib.take_photo(name, PHOTO_DIR + "/")
                    fname = f"{name}.jpg"
                    _logger.info("photo saved file=%s", fname)
                    await ws.send_json({"type": "photo_saved", "filename": fname})

                elif cmd == "rec_toggle":
                    if _rec_state == "stopped":
                        ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
                        vname = f"video_{ts}"
                        os.makedirs(VIDEO_DIR, exist_ok=True)
                        Vilib.rec_video_set["name"] = vname
                        Vilib.rec_video_set["path"] = VIDEO_DIR + "/"
                        await asyncio.to_thread(Vilib.rec_video_run)
                        await asyncio.sleep(0.5)
                        await asyncio.to_thread(Vilib.rec_video_start)
                        _rec_state = "recording"
                        _rec_last_video = vname + ".avi"
                        _logger.info("rec_start file=%s", _rec_last_video)
                        await ws.send_json({"type": "rec_state", "state": "recording"})
                    else:
                        await asyncio.to_thread(Vilib.rec_video_stop)
                        _rec_state = "stopped"
                        _logger.info("rec_stop file=%s", _rec_last_video)
                        await ws.send_json({"type": "rec_state", "state": "stopped"})
                        await ws.send_json({"type": "rec_stopped", "filename": _rec_last_video})

                elif cmd == "kill":
                    _logger.info("kill command from %s", client_ip)
                    await asyncio.to_thread(px.stop)
                    await asyncio.to_thread(px.set_dir_servo_angle, 0)
                    await ws.send_json({"type": "kill_confirmed"})

                elif cmd == "heartbeat":
                    _logger.debug("heartbeat from %s", client_ip)
                    _last_hb[0] = time.monotonic()

                elif cmd == "shutdown":
                    _logger.info("shutdown command from %s", client_ip)
                    await ws.send_json({"type": "shutdown_ack"})
                    asyncio.create_task(_graceful_shutdown())

        except (WebSocketDisconnect, RuntimeError):
            pass

    async def _send():
        try:
            while True:
                with _sensor_lock:
                    data = dict(_sensor)
                await ws.send_json({
                    "type": "sensors",
                    "distance": data["distance"],
                    "grayscale": data["grayscale"],
                    "battery": data["battery"],
                    "battery_warn": data["battery"] is not None and data["battery"] < 6.5,
                })
                await asyncio.sleep(0.2)
        except Exception:
            pass

    _last_hb = [time.monotonic()]

    async def _watchdog():
        _fired = False
        while True:
            await asyncio.sleep(1)
            if time.monotonic() - _last_hb[0] > WATCHDOG_TIMEOUT:
                if not _fired:
                    _logger.warning("Watchdog fired — no heartbeat for %.1fs, stopping motors", WATCHDOG_TIMEOUT)
                    _fired = True
                px.stop()
            else:
                if _fired:
                    _logger.info("Watchdog reset — heartbeat restored from %s", client_ip)
                    _fired = False

    recv_task     = asyncio.create_task(_recv())
    send_task     = asyncio.create_task(_send())
    watchdog_task = asyncio.create_task(_watchdog())
    try:
        await asyncio.wait(
            {recv_task, send_task, watchdog_task},
            return_when=asyncio.FIRST_COMPLETED,
        )
        for t in (recv_task, send_task, watchdog_task):
            t.cancel()
    finally:
        _logger.info("WebSocket disconnected from %s", client_ip)
        px.stop()


# ── Startup ───────────────────────────────────────────────────────────────────

def main():
    global px

    # File logger — always on from server start
    os.makedirs(LOG_DIR, exist_ok=True)
    log_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(LOG_DIR, f"server_{log_ts}.log")

    _logger.setLevel(logging.DEBUG)
    fh = logging.FileHandler(log_path)
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d  %(levelname)-7s  %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    ))
    _logger.addHandler(fh)
    _logger.info("Server starting | log=%s", log_path)

    px = Picarx()
    _logger.info("Picarx initialized")

    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)
    _logger.info("Camera starting...")

    deadline = time.time() + 5.0
    while time.time() < deadline:
        try:
            if Vilib.flask_start:
                break
        except AttributeError:
            time.sleep(1.0)
            break
        time.sleep(0.05)
    _logger.info("Camera stream ready at :9000")
    print("Camera stream ready at :9000")

    for d in (PHOTO_DIR, VIDEO_DIR, LOG_DIR):
        os.makedirs(d, exist_ok=True)

    threading.Thread(target=_sensor_loop, daemon=True).start()
    _logger.info("Sensor loop started")

    _logger.info("Dashboard listening at http://0.0.0.0:8000")
    print("Dashboard at http://0.0.0.0:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")


if __name__ == "__main__":
    main()
