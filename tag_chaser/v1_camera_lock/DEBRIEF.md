# Tag Chaser v1 — Camera Lock: Debrief & PRD

## What Was Built

An autonomous AprilTag chaser integrated into the PiCar-X dashboard. The robot detects AprilTag ID 0 (tag36h11 family), steers toward its horizontal center using a PID controller, drives forward at a configurable speed, and stops when within a configurable distance threshold. The system is activated and deactivated from the existing dashboard UI via a Tag Chase toggle button with a 3-second countdown.

---

## Architecture

```
tag_chaser/v1_camera_lock/
├── chaser.py       TagChaser class — detection, PID, WebSocket broadcast
├── steer_pid.py    Discrete PID with anti-windup
├── config.yaml     All tunable parameters
└── main.py         Standalone runner (no dashboard required)

dashboard/
├── server.py       FastAPI server — owns Picamera2, MJPEG stream, WebSocket
├── static/app.js   Dashboard JS — chase toggle, status bar, canvas overlay
├── static/index.html
└── static/style.css
```

**Camera ownership:** `server.py` owns the single Picamera2 instance for the entire session. Vilib is not used. The capture loop runs at full frame rate, encodes frames to JPEG for the MJPEG stream, and passes frames to `chaser.process_frame()` when chase is active.

**WebSocket architecture:** Per-connection `asyncio.Queue` prevents concurrent send corruption. The `_broadcast_coro` puts messages into each client's queue via `put_nowait()`. A single `_send` task per connection drains the queue serially. Transient send errors are logged and discarded rather than killing the task.

**Broadcast throttle:** `chaser.process_frame()` throttles `chase_detection` and `chase_status` broadcasts to 10 fps (100 ms). State transition events (`tag_acquired`, `tag_lost`, `chasing_started`) are logged immediately at INFO regardless of throttle.

**Frame format:** Picamera2 configured as `RGB888`. On this hardware, `capture_array()` returns a `(480, 640, 3) uint8` array in RGB order. The raw array is passed directly to `cv2.imencode` without conversion — this platform's libjpeg encodes correctly from RGB input. AprilTag detection uses `cv2.COLOR_RGB2GRAY` to match.

---

## PID Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| kp | 15.0 | Degrees per normalized error unit (error range −1 to 1) |
| ki | 0.5 | Integral — light accumulation |
| kd | 2.0 | Derivative — dampens overshoot |
| steer_limit_deg | ±20° | Softer cap than hardware max (±30°) |
| stop_distance_cm | 10.0 | Stop threshold |
| tag_lost_timeout_s | 0.5 | Hold last steer before stopping when tag disappears |
| countdown_s | 3 | Pre-drive countdown |
| tag_id | 0 | AprilTag36h11 family ID |
| tag_size_m | 0.05 | Physical tag side length |

---

## Bugs Found and Fixed

**Vilib/Picamera2 incompatibility — full Vilib removal.**
Vilib 0.3.18 uses a Picamera2 internal API (`allocator`) removed in Picamera2 0.3.36. Attempting to restart Vilib after chase caused `'NoneType' object has no attribute 'size'` crash. Solution: remove Vilib entirely. Server owns Picamera2 directly for the full session. MJPEG stream served on `:9000` via a stdlib `ThreadingTCPServer`.

**WebSocket concurrent send corruption.**
Original architecture called `ws.send_json()` from multiple concurrent asyncio tasks (broadcast coroutine and sensor push loop). At `await` boundaries these interleaved, causing Starlette to throw and silently remove the client from the send set. All subsequent broadcasts were dropped — including `chase_status active=False` after stop, so the toggle button never updated. Fix: replaced `_ws_clients: set` with `_ws_queues: dict` (ws → `asyncio.Queue`). All broadcasts use `put_nowait()`. One `_send` task per connection drains the queue serially.

**Camera color inversion.**
`BGR888` Picamera2 format + no conversion → JPEG with R and B channels swapped (red appeared blue). `RGB888` + `cv2.COLOR_RGB2BGR` also produced wrong output on this hardware. Root cause confirmed via `frame_diag` log: `capture_array()` returns RGB. This platform's OpenCV/libjpeg encodes from RGB input correctly without conversion. Fix: `RGB888` format, no `cvtColor` call, `frame = raw` passed directly to `imencode`.

**Tag ID mismatch.**
Physical tag is AprilTag36h11 ID 0. Code was configured for ID 1. Detection consistently returned `found=False`. Fix: `tag_id: 0` in `config.yaml`.

**`chase_status` broadcast flood crashing `_send` task.**
`process_frame` broadcast `chase_detection` and `chase_status` on every frame at 30 fps (~60 messages/second). Under this load `ws.send_json` threw a transient exception, killing `_send` silently. Client then received no further messages — including stop confirmation after toggle-off, leaving the button stuck in active state. Fix: per-frame broadcasts throttled to 10 fps; `_send` catches and logs transient errors instead of dying.

---

## Logging

All events routed to a timestamped log at `/home/jvpicar/picar_ros/logs/server_YYYYMMDD_HHMMSS.log`:

| Level | Events |
|-------|--------|
| INFO | Server start, WS connect/disconnect, all commands (drive, steer, gimbal, tag_chase, photo, rec, kill, shutdown, mode, detect), countdown ticks, chasing started, tag acquired/lost, distance stop reached |
| DEBUG | Heartbeats, steer angle changes (≥1°), detect found/not-found (1 s throttle), capture→chaser active (30 s throttle), frame diagnostic (once at startup), transient WS send errors |

---

## Limitations (v1 Scope)

- Camera pan/tilt locked to 0°. No gimbal tracking — horizontal centering only.
- Fixed forward speed. No speed scaling based on tag distance.
- No recovery behavior when tag is lost beyond holding last steer briefly.
- Distance-based stop is not latching — car resumes chasing if tag is moved back beyond stop threshold while chase is still active.
- `chase_stop reason=distance_reached` log fires per-frame while within stop distance (INFO spam). Throttle not yet implemented.
- Single tag ID (configurable in YAML but not from UI).
- SCP log transfer to Ubuntu (192.168.1.250) fails — SSH not routable between Pi and Ubuntu on current network configuration.

---

## v2 Candidates

**Distance-proportional speed.**
Scale drive speed down as tag distance approaches `stop_distance_cm`. Prevents overshoot at close range.

**Latching stop.**
Once the stop threshold is reached, stay stopped until the toggle is cycled. Current behavior resumes chasing if the tag moves away.

**Throttle `chase_stop` log.**
Only log `chase_stop reason=distance_reached` on state transition into stop zone, not every frame.

**Tilt tracking.**
Unlock camera tilt and use the tag's vertical center error as input to a second PID loop controlling tilt angle.

**UI speed control during chase.**
Allow the speed slider to affect chase speed live without requiring stop/restart.

**Operator override.**
WASD or Kill during chase stops the chaser cleanly and returns manual control.

**Multi-tag support.**
Allow operator to select tag ID from the dashboard before starting chase.

**Fix Ubuntu SCP transfer.**
Configure SSH key-based auth between Pi and Ubuntu so session logs transfer automatically on shutdown.

---

## Test Procedure

1. SSH into Pi: `python3 ~/picar_ros/dashboard/server.py`
2. Open dashboard at `http://192.168.1.241:8000`
3. Hold a 5 cm tag36h11 **ID 0** marker ~50 cm in front of the camera
4. Click **Tag Chase OFF** — button turns amber, status bar reads "Starting 3 2 1"
5. Robot steers toward tag and drives forward; status bar reads "Chasing — X cm"
6. Confirm robot stops at ~10 cm; status bar reads "Stopping"
7. Move tag out of frame — robot holds course briefly, then stops; status bar reads "Searching…"
8. Click **Tag Chase ON** — motors stop immediately, button resets, manual control returns
9. Standalone test: `python3 ~/picar_ros/tag_chaser/v1_camera_lock/main.py 30`

---

## Session Log

### 2026-06-10 — v1 Debugging and Fix Session

**What was investigated and fixed:**

Tag chase toggled on from the dashboard, countdown fired and `chasing started` appeared in logs, but the camera feed was unchanged and no detection output appeared. Four separate issues were diagnosed and resolved.

**Tag ID mismatch.** Physical tag is AprilTag36h11 ID 0. `config.yaml` had `tag_id: 1`. After changing to `tag_id: 0`, detection started immediately: `detect tag_acquired id=0 dist_cm=54.9` appeared on the first pass in front of the camera. All subsequent testing used ID 0.

**Camera color inversion.** The MJPEG feed showed red as blue and blue as red. `BGR888` format with no conversion was the first attempt — still wrong. `RGB888` + `cv2.COLOR_RGB2BGR` was also wrong. Root cause: `capture_array()` on this Pi hardware returns RGB order regardless of the configured format name. This Pi's libjpeg build encodes from RGB input correctly without any `cvtColor` call. Fix: `RGB888` format, `frame = raw` passed directly to `cv2.imencode`. `frame_diag` log confirmed `shape=(480, 640, 3) uint8` with channel means in expected range. User confirmed color fixed.

**Toggle-off not working.** Clicking Tag Chase OFF did not stop the car; Kill button was required. Root cause: `process_frame` was broadcasting `chase_detection` + `chase_status` on every frame at ~30 fps (60 msg/s). Under this load `ws.send_json` threw a transient exception. The original `_send` task caught it with a bare `except Exception: pass` and returned, dying silently. No further messages reached the client — including `chase_status active=False` on stop, leaving the toggle button stuck. Three-part fix: (1) per-frame broadcasts throttled to 10 fps via `bcast_due` check, (2) resilient `_send` — inner try/except logs transient errors but continues, only returns on `WebSocketDisconnect`, (3) server's `tag_chase stop` handler calls `px.stop()` and `px.set_dir_servo_angle(0)` directly after `chaser.stop()` as belt-and-suspenders. Log confirmed: single `tag_chase stop` entry, followed by clean heartbeats, no kill button needed.

**No visibility into chaser internals.** `chaser.py` had zero logging calls. Added `_logger = logging.getLogger("picarx")` and logging throughout: countdown ticks at INFO, `chasing started` at INFO, `detect tag_acquired`/`tag_lost` state transitions at INFO, periodic `detect found=True/False` at DEBUG (1 s throttle), steer angle changes ≥1° at DEBUG, `chase_stop reason=distance_reached` at INFO.

**Known remaining issue.** `chase_stop reason=distance_reached` fires at INFO on every frame (~30/s) while within stop distance. `_at_stop_dist` flag added to `__init__` to enable one-shot logging, but the throttle logic has not been wired into `process_frame` yet. Fix is one conditional: `if not self._at_stop_dist: _logger.info(...); self._at_stop_dist = True` before `px.stop()`.

---

## Environment

| Component | Version / Detail |
|-----------|-----------------|
| Picamera2 | 0.3.36 — owned by server.py, RGB888 format, no color conversion |
| pupil-apriltags | pip3 install, tag36h11 family |
| OpenCV | System install — `imencode` treats input as RGB on this platform |
| Python | 3.13 |
| OS | Raspberry Pi OS (bookworm) |
| Pi IP | 192.168.1.241 (user: jvpicar) |
| Ubuntu IP | 192.168.1.250 (user: jeano) — SSH not currently routable from Pi |
