# Tag Chaser v2 — TF Publishing & Live Trajectory Visualization
**GrayMatter Robotics — PiCar-X Tag Chaser Project**
*June 2026 | Status: Draft*

---

## 1. Purpose

Tag Chaser v1 produces no persistent spatial record of a session. The robot chases AprilTag ID 0 successfully but there is no way to review where the car went, where the tag was, or how the two trajectories related. This feature adds spatial awareness to the system: a stationary AprilTag ID 1 defines a world frame, the Pi computes and streams pose data for both tags over WebSocket, an Ubuntu-side ROS2 node transforms that data into TF and `visualization_msgs/MarkerArray` messages, and RViz2 on Ubuntu renders all three trajectories live during a chase session. Each chase cycle within a session is rendered in a distinct color. Session data is persisted to JSON files on both the Pi (raw camera-frame detections) and Ubuntu (world-frame transformed poses).

---

## 2. Scope

### 2.1 In Scope

- AprilTag ID 1 as stationary world frame anchor
- Chase-start gate: Tag 1 must be visible for up to 15 seconds; if not found within that window the dashboard prompts the user to switch to v1 or cancel
- Pi detects both Tag 0 and Tag 1 per frame, publishes pose + confidence for each visible tag at 10 fps over the existing dashboard WebSocket
- Ubuntu ROS2 node subscribes to Pi WebSocket, computes TF chain, publishes `visualization_msgs/MarkerArray` topics and a TF tree
- RViz2 on Ubuntu displays: car trajectory (world frame), Tag 0 trajectory (world frame), Tag 1 world origin marker — each chase cycle in a distinct color
- Trajectory gaps when Tag 1 is out of frame — no interpolation or dead reckoning in this version
- Confidence threshold filter on Ubuntu side — detections below threshold are discarded before TF computation
- Per-cycle JSON session files on Pi (raw) and Ubuntu (transformed)
- Structured logging: one timestamped session folder per server start on Pi, matching structure on Ubuntu for `tf_bridge`
- v2 lives in its own folder (`tag_chaser/v2_world_frame/`) with self-contained `chaser.py`, `tf_publisher.py`, `steer_pid.py`, `config.yaml`, `main.py`, `DEBRIEF.md`
- Dashboard defaults to v2 tag chaser on toggle; v1 fallback available via popup if world not found within 15 seconds at start gate

### 2.2 Out of Scope

- Dead reckoning or odometry bridging during Tag 1 occlusion
- ROS2 installation on the Pi — Pi remains Python-only; Ubuntu does all ROS2 work
- Bag file recording (future feature)
- Gimbal tracking — camera tilt remains locked at 0 degrees
- Multi-session trajectory comparison
- v1 fallback popup mid-session (start gate only)

---

## 3. Architecture

### 3.1 System Diagram

The system has two processes: one on the Pi and one on Ubuntu. No ROS2 runs on the Pi.

| Layer | Process | Responsibility |
|---|---|---|
| Pi — Detection | `server.py` (existing) | Owns Picamera2, runs AprilTag detection on both Tag 0 and Tag 1 at 10 fps, broadcasts pose + confidence JSON over existing dashboard WebSocket on port 8000 |
| Pi — TF Serialization | `tf_publisher.py` (new, v2 folder) | Serializes detection results into `tag_detections` WebSocket messages; owns per-cycle raw JSON file writing |
| Ubuntu — TF Bridge | `tf_bridge.py` (new ROS2 node) | Subscribes to Pi WebSocket, parses detection JSON, computes TF transforms, publishes `/tf`, `/trajectory/car`, `/trajectory/tag0`, `/marker/world`; writes per-cycle world-frame JSON |
| Ubuntu — Visualization | RViz2 (existing install) | Subscribes to ROS2 topics, renders color-coded per-cycle trajectories and world origin marker live |

### 3.2 Coordinate Frames

| Frame ID | Definition | Parent |
|---|---|---|
| `world` | Tag 1 pose at session start — fixed for entire session | None (root) |
| `camera` | Pi camera optical frame | `world` (via Tag 1 detection) |
| `car` | Approximated as camera frame — no separate body transform in v2 | `world` |
| `tag0` | Tag 0 pose in world frame — updates each frame Tag 0 is visible | `world` |

> **Note on car vs camera frame:** In v2, the car frame is treated as identical to the camera frame. The camera is mounted on a pan/tilt gimbal locked at 0 degrees, so this approximation is acceptable for trajectory visualization. A proper `base_link → camera_link` static transform can be added in v3 once the URDF is validated.

### 3.3 TF Chain

When both tags are visible in a frame, the Ubuntu node computes:

```
world → camera:  T_world_camera = T_tag1_camera^-1
world → tag0:    T_world_tag0   = T_world_camera * T_camera_tag0
```

`pupil-apriltags` returns `T_camera_tag` (pose of tag in camera frame). Inverting `T_camera_tag1` gives the camera pose in the world frame. Chaining with `T_camera_tag0` gives Tag 0 in world frame.

When Tag 1 is not visible: no world-anchored transform is computable. The node publishes nothing for that frame. RViz marker arrays will show a gap.

### 3.4 WebSocket Message Format

Pi extends the existing dashboard WebSocket broadcast with a new message type: `tag_detections`. Emitted at 10 fps when chase is active.

```json
{
  "type": "tag_detections",
  "ts": 1749600000.123,
  "cycle": 2,
  "tags": [
    {
      "id": 0,
      "confidence": 42.7,
      "pose_t": [x, y, z],
      "pose_R": [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]]
    },
    {
      "id": 1,
      "confidence": 61.2,
      "pose_t": [x, y, z],
      "pose_R": [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]]
    }
  ]
}
```

`pose_t` is translation in meters `[x, y, z]` in camera frame. `pose_R` is the 3x3 rotation matrix from `pupil-apriltags`. `cycle` is a monotonically incrementing integer, starting at 0 at node launch, incrementing on each toggle-on. Only tags visible in the current frame are included in the `tags` array. If no tags are visible the message is not emitted.

---

## 4. Feature Requirements

### 4.1 Pi-Side Changes (`chaser.py` / `server.py` / `tf_publisher.py`)

| ID | Requirement | Notes |
|---|---|---|
| P-01 | Detect both Tag 0 and Tag 1 in every frame when chase is active | Single `detector.detect()` call returns all visible tags; filter by family `tag36h11` |
| P-02 | Extract `pose_t` and `pose_R` from `pupil-apriltags` result for each visible tag | Requires camera intrinsics matrix and tag physical size passed to detector |
| P-03 | Extract `decision_margin` as confidence value | `pupil-apriltags` field name is `decision_margin` |
| P-04 | Broadcast `tag_detections` JSON at 10 fps when chase is active | Handled by `tf_publisher.py`; reuse existing throttle mechanism |
| P-05 | Chase-start gate: Tag Chase toggle starts a 15-second world-search window | If Tag 1 not detected within 15 s, dashboard popup fires; chase has not started yet |
| P-06 | `tag_size_m` in `config.yaml` must be correct for each tag ID | Both tags must be same physical size in v2, or config must support per-ID size map |
| P-07 | Camera intrinsics loaded from calibration file at server startup | Path configurable in `config.yaml`; fail loudly if file not found |
| P-08 | `tf_publisher.py` writes one raw JSON file per chase cycle to Pi session log folder | Filename: `cycle_<N>_raw_HHMMSS.json`; contains camera-frame pose_t, pose_R, confidence per tag per frame; closed on toggle-off or server shutdown |
| P-09 | `cycle` counter increments on each toggle-on, resets to 0 on server restart | Shared between `chaser.py` and `tf_publisher.py` via a shared state object |

### 4.2 Ubuntu-Side: `tf_bridge.py` ROS2 Node

| ID | Requirement | Notes |
|---|---|---|
| U-01 | Connect to Pi WebSocket `ws://192.168.1.241:8000/ws` on node startup | Reconnect with exponential backoff on disconnect |
| U-02 | Filter detections below confidence threshold before processing | Default threshold: 20.0 (configurable as ROS2 param) |
| U-03 | Convert `pose_R` (3x3 matrix) to quaternion for TF broadcast | Use `scipy.spatial.transform.Rotation` |
| U-04 | Publish TF: `world → camera` on every frame where Tag 1 is visible and above threshold | Frame ID: `world`, child: `camera` |
| U-05 | Publish TF: `world → tag0` on every frame where both Tag 0 and Tag 1 are visible and above threshold | Frame ID: `world`, child: `tag0` |
| U-06 | Publish car trajectory as `visualization_msgs/MarkerArray` on `/trajectory/car` | Each cycle is a separate `LINE_STRIP` marker with a unique color from the palette; marker ID = cycle number |
| U-07 | Publish tag0 trajectory as `visualization_msgs/MarkerArray` on `/trajectory/tag0` | Same cycle-color scheme as car trajectory |
| U-08 | Publish `/marker/world` as `visualization_msgs/Marker` (AXES type) at Tag 1 origin | Published once on first valid Tag 1 detection; static thereafter |
| U-09 | Expose `/reset_markers` service: clears all marker arrays and republishes empty arrays | Allows operator to clear all trajectories without restarting node |
| U-10 | Write one world-frame JSON file per chase cycle to Ubuntu session log folder | Filename: `cycle_<N>_world_HHMMSS.json`; contains world-frame car and tag0 poses per frame; closed on toggle-off or node shutdown |
| U-11 | Cycle color palette: fixed ordered list — green, red, blue, orange, magenta, cyan, yellow — cycling if more than 7 cycles in a session | Cycle 0 = green for car / green for tag0 (same hue, differentiated by topic/display) |

### 4.3 RViz2 Configuration

| ID | Requirement | Notes |
|---|---|---|
| R-01 | Fixed Frame set to `world` | |
| R-02 | MarkerArray display for `/trajectory/car` | Per-cycle colors assigned by tf_bridge |
| R-03 | MarkerArray display for `/trajectory/tag0` | Per-cycle colors assigned by tf_bridge |
| R-04 | Marker display for `/marker/world` — shows axes at world origin | |
| R-05 | TF display showing `world`, `camera`, `tag0` frames | |
| R-06 | Config saved as `picar_trajectory.rviz` in repo | Committed so operator launches with one command |

---

## 5. Data Flow — Per Frame

1. Picamera2 captures frame (RGB888, 640×480)
2. `chaser.process_frame()` runs AprilTag detection — both tags
3. If chase is active and 10 fps throttle is due: `tf_publisher.py` serializes visible tag poses + confidence + cycle number to `tag_detections` JSON, `put_nowait()` into all WebSocket queues; appends raw frame record to open cycle JSON file
4. `tf_bridge.py` on Ubuntu receives message, parses `tags` array
5. For each tag with `confidence >= threshold`: convert rotation matrix to quaternion
6. If Tag 1 present: compute `world→camera` transform, append point to cycle's `LINE_STRIP` marker on `/trajectory/car`, broadcast TF, append to cycle world-frame JSON
7. If Tag 0 also present: compute `world→tag0`, append point to cycle's `LINE_STRIP` marker on `/trajectory/tag0`, broadcast TF
8. Publish updated MarkerArrays; RViz2 renders

---

## 6. Chase-Start Gate — State Logic

### 6.1 Start Gate (Toggle Press)

| Condition | Result | UI |
|---|---|---|
| Tag 1 visible at toggle press, confidence >= threshold | Countdown starts (3s), TF publishing begins, cycle counter increments | Status bar: `"World found — Starting 3 2 1"` |
| Tag 1 not visible at toggle press | 15-second world-search window opens; car does not move | Status bar: `"Searching for world tag… Xs"` (countdown) |
| Tag 1 found within 15-second window | Countdown starts, chase begins | Status bar: `"World found — Starting 3 2 1"` |
| 15 seconds elapsed, Tag 1 still not found | Dashboard popup fires | See Section 6.2 |

### 6.2 World-Not-Found Popup

Fires only at the start gate, never mid-session.

```
┌─────────────────────────────────────────────┐
│  World tag not found after 15 seconds.      │
│  How would you like to proceed?             │
│                                             │
│  [ Switch to v1 ]   [ Cancel tag chase ]    │
└─────────────────────────────────────────────┘
```

- **Switch to v1**: activates Tag Chaser v1 (Tag 0 only, no world frame, no TF). Countdown starts immediately. Status bar: `"v1 mode — Starting 3 2 1"`
- **Cancel tag chase**: toggle returns to OFF state, no chase starts

### 6.3 Mid-Session World Loss

| Condition | Result | UI |
|---|---|---|
| Tag 1 lost during active chase | Chase continues, TF gaps recorded | Status bar: `"Chasing — world lost"` (amber) |
| Tag 1 reacquires | TF publishing resumes, path continues from reacquired anchor | Status bar: `"Chasing — world Xcm"` |

---

## 7. Logging

### 7.1 Philosophy

Log functional state changes, not frame-level data. Every line must answer one of: what happened, what caused it, or what was the result. High-frequency per-frame data is not logged except on state transitions. All log output goes to both terminal (stdout) and the log file simultaneously via Python's `logging` module with a `StreamHandler` and a `FileHandler` on the same logger.

### 7.2 Session Folder Structure — Pi

One timestamped folder created per `server.py` start. All Pi-side logs and raw cycle JSON files live here.

```
/home/jvpicar/picar_ros/logs/
└── session_YYYYMMDD_HHMMSS/
    ├── master.log
    ├── drive_steer.log
    ├── pan_tilt.log
    ├── marker_detector.log          # created only if tag chase activated
    ├── cycle_0_raw_HHMMSS.json      # created on first toggle-on
    ├── cycle_1_raw_HHMMSS.json
    └── ...
```

### 7.3 Session Folder Structure — Ubuntu

Mirror structure, created per `tf_bridge.py` node launch.

```
~/picar_ros/logs/
└── session_YYYYMMDD_HHMMSS/
    ├── tf_bridge.log
    ├── cycle_0_world_HHMMSS.json
    ├── cycle_1_world_HHMMSS.json
    └── ...
```

To consolidate after a session: `rsync jvpicar@192.168.1.241:/home/jvpicar/picar_ros/logs/ ~/picar_ros/logs/`

### 7.4 Log Responsibilities Per File

| Log file | Owner | What it contains |
|---|---|---|
| `master.log` | `server.py` | Server start/stop, unhandled exceptions, WebSocket client connect/disconnect, server-level errors only — not node events |
| `drive_steer.log` | Drive/steer subsystem | Motor commands issued, speed changes, steering angle changes ≥1°, stop events with reason |
| `pan_tilt.log` | Pan/tilt subsystem | Gimbal angle commands, reset events |
| `marker_detector.log` | `chaser.py` + `tf_publisher.py` | Chase toggle on/off, cycle number, world tag acquired/lost (state transitions only), Tag 0 acquired/lost (state transitions only), countdown ticks, distance stop events, chase_stop reason, WebSocket broadcast errors |
| `tf_bridge.log` | `tf_bridge.py` on Ubuntu | Node start/stop, WebSocket connect/disconnect to Pi, first detection per session, cycle start/end, world tag acquisition/loss as seen by Ubuntu, `/reset_markers` service calls, file write events |

### 7.5 Log Levels

| Level | Used for |
|---|---|
| INFO | All functional state changes: chase on/off, tag acquired/lost, world acquired/lost, cycle increments, server start/stop, file open/close, distance stop reached (on transition only) |
| DEBUG | Steer angle changes, heartbeats, WebSocket send errors (transient), detection data (1s throttle during active chase) |
| ERROR | Unhandled exceptions, calibration file failures, WebSocket fatal errors |

### 7.6 Log Format

```
2026-06-10 14:23:01.452 [INFO ] [marker_detector] chase_start cycle=2 world=acquired
2026-06-10 14:23:07.891 [INFO ] [marker_detector] world_lost — continuing chase, TF gap open
2026-06-10 14:23:09.103 [INFO ] [marker_detector] world_reacquired — TF gap closed
2026-06-10 14:23:14.220 [INFO ] [marker_detector] chase_stop cycle=2 reason=toggle_off
```

---

## 8. v2 Folder Structure

v2 is self-contained. It does not modify v1 files. The dashboard routes to v2 by default.

```
tag_chaser/
├── v1_camera_lock/          # unchanged
│   ├── chaser.py
│   ├── steer_pid.py
│   ├── config.yaml
│   ├── main.py
│   └── DEBRIEF.md
└── v2_world_frame/          # new
    ├── chaser.py            # detection, PID steering, cycle management
    ├── tf_publisher.py      # pose serialization, WebSocket broadcast, raw JSON file writing
    ├── steer_pid.py         # copied from v1, tune separately
    ├── config.yaml          # v2 defaults including calibration_file path
    ├── main.py              # standalone runner
    └── DEBRIEF.md           # starts empty, filled post-build

ros2/
└── tf_bridge/
    ├── tf_bridge.py
    ├── package.xml
    └── setup.py

rviz/
└── picar_trajectory.rviz

scripts/
└── launch_tf_bridge.sh

dashboard/
└── server.py                # modified: default to v2, v1 fallback popup
```

---

## 9. File Changes Summary

| Path | New / Modified | Description |
|---|---|---|
| `tag_chaser/v2_world_frame/chaser.py` | New | Detection of both tags, PID steering, cycle counter, integration with `tf_publisher.py` |
| `tag_chaser/v2_world_frame/tf_publisher.py` | New | Pose serialization, `tag_detections` WebSocket broadcast, raw cycle JSON file writing |
| `tag_chaser/v2_world_frame/steer_pid.py` | New | Copy of v1 `steer_pid.py` — tuned independently for v2 |
| `tag_chaser/v2_world_frame/config.yaml` | New | v2 config: `calibration_file`, `tag_size_m`, PID defaults, confidence threshold |
| `tag_chaser/v2_world_frame/main.py` | New | Standalone runner, no dashboard required |
| `tag_chaser/v2_world_frame/DEBRIEF.md` | New | Empty at build time; filled post-session |
| `dashboard/server.py` | Modified | Default tag chase to v2; 15s world-search window; v1 fallback popup |
| `ros2/tf_bridge/tf_bridge.py` | New | Ubuntu ROS2 node |
| `ros2/tf_bridge/package.xml` | New | ROS2 package manifest |
| `ros2/tf_bridge/setup.py` | New | ROS2 Python package setup |
| `rviz/picar_trajectory.rviz` | New | RViz2 config |
| `scripts/launch_tf_bridge.sh` | New | One-command Ubuntu launch |

---

## 10. Dependencies

### 10.1 Pi

| Package | Already installed? | Notes |
|---|---|---|
| `pupil-apriltags` | Yes (v1) | No change needed |
| `numpy` | Yes | Rotation matrix operations |
| `opencv-python` | Yes | Already in use |

### 10.2 Ubuntu

| Package | Install command | Notes |
|---|---|---|
| `rclpy` | ROS2 Humble base install | Already present |
| `geometry_msgs`, `visualization_msgs`, `tf2_ros` | `sudo apt install ros-humble-*` | Standard ROS2 packages |
| `websockets` | `pip3 install websockets` | Async WebSocket client |
| `scipy` | `pip3 install scipy` | Rotation matrix → quaternion |

---

## 11. Verification

### 11.1 Pi-Side

| Test | Pass criterion |
|---|---|
| Toggle chase with Tag 1 visible | Status bar: `"World found — Starting 3 2 1"`, countdown proceeds, `marker_detector.log` shows `chase_start cycle=0 world=acquired` |
| Toggle chase with no Tag 1 visible | 15s countdown in status bar, popup fires at expiry, no chase started |
| Popup: Switch to v1 | v1 chase starts immediately, status bar shows `"v1 mode — Starting 3 2 1"` |
| Popup: Cancel | Toggle returns to OFF, no chase, no countdown |
| Server startup with missing calibration file | Server raises with clear error, does not start |
| Toggle off during active chase | `cycle_N_raw_HHMMSS.json` closed and present in session folder |
| Second toggle-on in same session | Cycle N+1 raw JSON created, new color assigned |

### 11.2 Ubuntu-Side

| Test | Pass criterion |
|---|---|
| `ros2 topic echo /trajectory/car` during chase | MarkerArray messages with LINE_STRIP markers appending at ~10 fps when Tag 1 visible |
| Move Tag 1 out of frame | No new marker points added; existing markers retained |
| Restore Tag 1 | Marker resumes appending; gap visible in RViz |
| Second chase cycle | New marker with different color appears on `/trajectory/car` and `/trajectory/tag0` |
| `/reset_markers` service call | All marker arrays cleared in RViz |
| Toggle off | `cycle_N_world_HHMMSS.json` closed and present in Ubuntu session folder |

### 11.3 RViz

| Test | Pass criterion |
|---|---|
| Launch `picar_trajectory.rviz`, begin chase | Cycle 0 trajectory grows in real time at world origin; axes marker visible |
| Drive arc, toggle off, reposition, toggle on | Two distinct colored trajectories visible simultaneously |
| Drive until Tag 1 lost | Trajectory gap visible; resumes on reacquisition |

### 11.4 Logging

| Test | Pass criterion |
|---|---|
| Session folder created on server start | `/home/jvpicar/picar_ros/logs/session_YYYYMMDD_HHMMSS/` exists with `master.log` |
| `marker_detector.log` absent before tag chase | File does not exist until first tag chase toggle |
| `marker_detector.log` present after tag chase | All state transitions logged; no frame-level spam |
| Ubuntu session folder | `~/picar_ros/logs/session_YYYYMMDD_HHMMSS/tf_bridge.log` exists after node launch |

---

## 12. Open Questions

| ID | Question | Decision needed before |
|---|---|---|
| OQ-01 | Has the ChArUco calibration procedure been run and a `.yaml` file saved? If not, this is a hard prerequisite before any v2 build work. | P-07 implementation |
| OQ-02 | Are both tags the same physical size? If not, `config.yaml` needs a per-ID size map. | P-06 implementation |
| OQ-03 | Ubuntu IP `192.168.1.250` — SSH Pi→Ubuntu not routable. `tf_bridge` connects Ubuntu→Pi (WebSocket), so not a blocker for v2. Confirm. | U-01 implementation |
| OQ-04 | Should trajectory markers persist in RViz after chase toggled off, or auto-clear on next toggle-on? | U-09 scope |
| OQ-05 | Color scheme: car and tag0 markers for the same cycle share the same hue. Is that acceptable, or should car and tag0 always be visually distinct (e.g. car=solid, tag0=dashed)? MarkerArray LINE_STRIP has no dash support — would require alternating point density instead. | R-02/R-03 |

---

## 13. Recommended Build Order

Sequential. Verify each step before proceeding.

1. Confirm calibration file exists — load in Python, print `camera_matrix`. If invalid, run ChArUco calibration first.
2. Pi: scaffold `v2_world_frame/` folder. Copy `steer_pid.py` from v1. Write `config.yaml` with v2 defaults.
3. Pi: extend `chaser.py` to detect both tags, extract pose + confidence. Test with print logging only — no WebSocket yet.
4. Pi: write `tf_publisher.py` — serialize detections to `tag_detections` JSON and print to stdout. Verify message format before any network work.
5. Pi: integrate `tf_publisher.py` broadcast into `server.py` WebSocket. Verify with `wscat` or browser console.
6. Pi: implement 15-second world-search window and v1 fallback popup in `server.py` + dashboard JS.
7. Pi: implement per-cycle raw JSON file writing in `tf_publisher.py`. Verify file is created and closed correctly on toggle.
8. Pi: implement session folder creation and per-node log files in `server.py`. Verify folder and all log files appear on start.
9. Ubuntu: write `tf_bridge.py` — WebSocket receive and JSON parse only, no ROS2. Print detections to confirm data arrives.
10. Ubuntu: add ROS2 TF publishing. Verify with `ros2 topic echo /tf`.
11. Ubuntu: add `visualization_msgs/MarkerArray` publishing with cycle colors. Verify with `ros2 topic echo /trajectory/car`.
12. Ubuntu: add per-cycle world-frame JSON file writing and session log folder. Verify files created correctly.
13. Ubuntu: create `picar_trajectory.rviz`. Launch and confirm end-to-end visualization.
14. Run full verification checklist from Section 11.

---

*GrayMatter Robotics — PiCar-X Tag Chaser Project | Internal Development Document | June 2026*
