# Tag Chaser v1 — Camera Lock: Debrief & PRD

## What Was Built

An autonomous AprilTag chaser integrated into the PiCar-X dashboard. The robot detects AprilTag ID 1, steers toward its horizontal center using a PID controller, drives forward at a configurable speed, and stops when within a configurable distance threshold. The system is activated and deactivated from the existing dashboard UI via a Tag Chase toggle button.

---

## Architecture

```
tag_chaser/v1_camera_lock/
├── chaser.py       TagChaser class — detection loop, PID, WebSocket broadcast
├── steer_pid.py    Discrete PID with anti-windup
├── config.yaml     All tunable parameters
└── main.py         Standalone runner (no dashboard required)
```

The chaser shares the same `Picarx` instance as the dashboard. On activation it closes Vilib, opens its own Picamera2 instance, runs detection in a daemon thread, and broadcasts status/detection events over WebSocket back to all connected dashboard clients. On deactivation it releases Picamera2 and restarts Vilib.

The dashboard draws a canvas overlay showing the tag bounding box and center dot, and a status bar that reads: **Starting 3 2 1 → Chasing → Distance to tag X cm → Stopping → Searching**.

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
| tag_id | 1 | AprilTag36h11 family |
| tag_size_m | 0.05 | Physical tag side length |

---

## Known Issues & Fixes Applied

**`Vilib.camera_stop()` does not exist in Vilib 0.3.18.**
The correct method is `Vilib.camera_close()`. Fixed in `chaser.py` after discovery through runtime error.

**Vilib `allocator` attribute error at server startup.**
Vilib 0.3.18 uses a Picamera2 internal API (`allocator`) that was removed in Picamera2 0.3.36. Vilib's listener thread crashes at startup but the camera itself remains functional for the web stream. This does not block chaser operation since the chaser bypasses Vilib entirely and opens its own Picamera2 instance.

**No live video feed during chase.**
When the chaser activates, Vilib is closed and the dashboard camera feed at `:9000` goes dark. The canvas overlay draws detection results on top of the frozen frame, but there is no live image underneath.

---

## Limitations (v1 Scope)

- Camera pan/tilt locked to 0°. No gimbal tracking — horizontal centering only.
- Fixed forward speed. No speed scaling based on tag distance.
- No recovery behavior when tag is lost beyond holding last steer briefly.
- No live camera feed during chase (only detection overlay on frozen frame).
- Single tag ID (configurable in YAML but not from UI).
- Detection runs at full frame rate with no throttle — CPU usage is high on Pi.

---

## v2 Candidates

**Live camera stream during chase.**
Serve a MJPEG from the chaser's Picamera2 instance on a second port (`:8081`). Switch the dashboard `<img>` src from `:9000` to `:8081` when chase activates, then back on deactivation. This gives the operator a live annotated view.

**Distance-proportional speed.**
Scale drive speed down as tag distance approaches `stop_distance_cm`. Prevents overshoot at close range.

**Tilt tracking.**
Unlock camera tilt and use the tag's vertical center error as input to a second PID loop controlling tilt angle.

**Tag-relative stopping.**
Use pose translation vector directly (x, y, z in camera frame) rather than pixel distance to set steer angle. More geometrically accurate at oblique angles.

**UI speed control during chase.**
Allow the speed slider to affect chase speed live without requiring stop/restart.

**Operator override.**
Pressing WASD or Kill during chase stops the chaser cleanly and returns manual control.

**Multi-tag support.**
Allow the operator to select tag ID from the dashboard before starting chase.

---

## Test Procedure

1. SSH into Pi, run `python3 /home/jvpicar/picar_ros/dashboard/server.py`
2. Open dashboard at `http://192.168.1.241:8000`
3. Hold a 5 cm tag36h11 ID 1 marker ~50 cm in front of the camera
4. Click **Tag Chase OFF** — button turns amber, status bar reads "Starting 3 2 1"
5. Robot steers toward tag and drives forward; status bar reads "Chasing — X cm"
6. Confirm robot stops at ~10 cm; status bar reads "Stopping"
7. Move tag out of frame — robot holds course briefly, then stops; status bar reads "Searching…"
8. Click **Tag Chase ON** — Vilib restarts, camera feed resumes, WASD control returns
9. Standalone test: `python3 /home/jvpicar/picar_ros/tag_chaser/v1_camera_lock/main.py 30`

---

## Environment

| Component | Version |
|-----------|---------|
| Vilib | 0.3.18 |
| Picamera2 | 0.3.36 |
| pupil-apriltags | latest (installed via pip3 --break-system-packages) |
| Python | 3.13 |
| OS | Raspberry Pi OS (bookworm) |
