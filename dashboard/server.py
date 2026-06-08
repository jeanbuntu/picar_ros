import asyncio
import json
import os
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
WATCHDOG_TIMEOUT = 3.0

app = FastAPI()
app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "static")), name="static")

px: Picarx = None
_sensor = {"distance": -1.0, "grayscale": [0, 0, 0], "battery": None}
_sensor_lock = threading.Lock()
_rec_state = "stopped"

try:
    _username = os.getlogin()
except Exception:
    _username = "jvpicar"

PHOTO_DIR   = f"/home/{_username}/Pictures/picar-x"
VIDEO_DIR   = f"/home/{_username}/Videos/picar-x"
SESSION_DIR = f"/home/{_username}/Documents/picar-x"

# Session state (mutable dict — no global declarations needed in nested funcs)
_session = {
    "active": False,
    "log": [],
    "last_sensor_ts": 0.0,
    "last_video": "",
}


def _log(entry: dict):
    if not _session["active"]:
        return
    _session["log"].append({
        "ts": datetime.now().isoformat(timespec="milliseconds"),
        **entry,
    })


def _sensor_loop():
    _batt_tick = 24
    while True:
        try:
            dist = px.get_distance()
            gs = px.get_grayscale_data()
            with _sensor_lock:
                _sensor["distance"] = round(float(dist), 1)
                _sensor["grayscale"] = [int(v) for v in gs]
        except Exception:
            pass
        # Battery is slow to change — read every 5 s (every 25 loops at 200 ms)
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


@app.get("/download/session/{filename}")
async def dl_session(filename: str):
    path = os.path.join(SESSION_DIR, filename)
    if not os.path.isfile(path):
        raise HTTPException(status_code=404, detail="Not found")
    return FileResponse(path, media_type="application/json", filename=filename)


# ── WebSocket ─────────────────────────────────────────────────────────────────

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    global _rec_state
    await ws.accept()

    async def _recv():
        global _rec_state
        try:
            while True:
                msg = await ws.receive_json()
                cmd = msg.get("cmd")

                if cmd == "drive":
                    direction = msg.get("direction", "stop")
                    speed = max(0, min(100, int(msg.get("speed", 50))))
                    _log({"event": "drive", "direction": direction, "speed": speed})
                    if direction == "forward":
                        await asyncio.to_thread(px.forward, speed)
                    elif direction == "backward":
                        await asyncio.to_thread(px.backward, speed)
                    else:
                        await asyncio.to_thread(px.stop)

                elif cmd == "steer":
                    angle = max(-30, min(30, int(msg.get("angle", 0))))
                    _log({"event": "steer", "angle": angle})
                    await asyncio.to_thread(px.set_dir_servo_angle, angle)

                elif cmd == "gimbal":
                    axis = msg.get("axis", "pan")
                    angle = int(msg.get("angle", 0))
                    _log({"event": "gimbal", "axis": axis, "angle": angle})
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
                    _log({"event": "photo", "file": fname})
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
                        _session["last_video"] = vname + ".avi"
                        _log({"event": "rec_start", "file": _session["last_video"]})
                        await ws.send_json({"type": "rec_state", "state": "recording"})
                    else:
                        await asyncio.to_thread(Vilib.rec_video_stop)
                        _rec_state = "stopped"
                        _log({"event": "rec_stop", "file": _session["last_video"]})
                        await ws.send_json({"type": "rec_state", "state": "stopped"})
                        await ws.send_json({"type": "rec_stopped", "filename": _session["last_video"]})

                elif cmd == "session_start":
                    _session["active"] = True
                    _session["log"] = []
                    _session["last_sensor_ts"] = 0.0
                    _log({"event": "session_start"})
                    await ws.send_json({"type": "session_state", "state": "active"})

                elif cmd == "session_stop":
                    _log({"event": "session_end"})
                    _session["active"] = False
                    ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
                    sfname = f"session_{ts}.json"
                    os.makedirs(SESSION_DIR, exist_ok=True)
                    with open(os.path.join(SESSION_DIR, sfname), "w") as f:
                        json.dump(_session["log"], f, indent=2)
                    _session["log"] = []
                    await ws.send_json({
                        "type": "session_state",
                        "state": "idle",
                        "filename": sfname,
                    })

                elif cmd == "kill":
                    _log({"event": "kill"})
                    await asyncio.to_thread(px.stop)
                    await asyncio.to_thread(px.set_dir_servo_angle, 0)
                    await ws.send_json({"type": "kill_confirmed"})

                elif cmd == "heartbeat":
                    _last_hb[0] = time.monotonic()

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
                now = time.monotonic()
                if _session["active"] and (now - _session["last_sensor_ts"]) >= 5.0:
                    _log({
                        "event": "sensor",
                        "distance": data["distance"],
                        "grayscale": data["grayscale"],
                    })
                    _session["last_sensor_ts"] = now
                await asyncio.sleep(0.2)
        except Exception:
            pass

    _last_hb = [time.monotonic()]

    async def _watchdog():
        while True:
            await asyncio.sleep(1)
            if time.monotonic() - _last_hb[0] > WATCHDOG_TIMEOUT:
                px.stop()

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
        px.stop()


# ── Startup ───────────────────────────────────────────────────────────────────

def main():
    global px

    px = Picarx()

    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)

    print("Waiting for camera stream...")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        try:
            if Vilib.flask_start:
                break
        except AttributeError:
            time.sleep(1.0)
            break
        time.sleep(0.05)
    print("Camera stream ready at :9000")

    for d in (PHOTO_DIR, VIDEO_DIR, SESSION_DIR):
        os.makedirs(d, exist_ok=True)

    threading.Thread(target=_sensor_loop, daemon=True).start()

    print("Dashboard at http://0.0.0.0:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")


if __name__ == "__main__":
    main()
