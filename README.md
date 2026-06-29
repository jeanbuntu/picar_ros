# picar_ros

A hobby robotics project built on a Sunfounder PiCar-X. The goal was to progressively build an autonomous AprilTag chasing system, starting from simple pixel-space control and working toward full image-based visual servoing with ROS2 trajectory visualization. Three versions were developed over June 2026, each on its own branch.

**Hardware:** Sunfounder PiCar-X (Raspberry Pi 4, Pi Camera Module 3, pan/tilt gimbal, front steering, rear drive motors)
**Stack:** Python on the Pi, ROS2 (Humble) on Ubuntu, FastAPI + WebSocket dashboard, AprilTag36h11 detection via `pupil-apriltags`, OpenCV, RViz2

Reddit posts: [TODO: Reddit link]

---

## Repository Structure

```
tag_chaser/
├── v1_camera_lock/     # v1 — pixel-space PID chase
├── v2_world_tf/        # v2 — dual-tag world frame + RViz trajectory
└── v3_ibvs/            # v3 — image-based visual servoing + pan/tilt

dashboard/
├── server.py           # FastAPI server — owns Picamera2, MJPEG, WebSocket
└── static/             # Dashboard UI (JS/HTML/CSS)

src/
├── tf_bridge/          # ROS2 node — TF math, RViz marker publisher
└── picar_description/  # ROS2 URDF + launch for robot visualization

scripts/
└── launch_tf_bridge.sh # Starts robot_state_publisher + tf_bridge together

rviz/
└── picar_trajectory.rviz

camera_cal_marker/      # Camera calibration data
logs/                   # Session logs (Pi-side and Ubuntu-side)
```

---

## v1 — Camera Lock (branch: `main`)

**Goal:** Get the car to autonomously detect AprilTag ID 0 and drive toward it using a PID controller on horizontal pixel error. No world frame, no gimbal movement, camera fixed.

### What Was Built

A tag chase mode integrated into the browser dashboard. The operator clicks a toggle, a 3-second countdown runs, then the car steers toward the tag's horizontal center and drives forward at a fixed speed. It stops when the tag is within a configurable distance threshold (`stop_distance_cm: 10cm`). Tag loss holds the last steering angle briefly before stopping.

The entire camera pipeline was taken over from scratch: `server.py` owns the single Picamera2 instance for the full session, serves MJPEG on port 9000, and passes frames to the chaser on each tick.

**PID configuration:**

| Parameter | Value |
|-----------|-------|
| kp | 15.0 |
| ki | 0.5 |
| kd | 2.0 |
| steer limit | ±20° |
| stop distance | 10 cm |
| tag family | AprilTag36h11 ID 0 |
| tag size | 5 cm |

### Bugs Diagnosed and Fixed

**Vilib/Picamera2 incompatibility.** Vilib 0.3.18 uses a Picamera2 internal API (`allocator`) removed in 0.3.36. Crashed on restart. Fix: removed Vilib entirely; server owns Picamera2 directly.

**WebSocket concurrent send corruption.** Multiple asyncio tasks calling `ws.send_json()` interleaved at `await` boundaries, causing Starlette to silently drop the client from the send set. The `chase_status active=False` stop confirmation never reached the browser — the toggle button stayed stuck in the active state. Fix: replaced `_ws_clients: set` with `_ws_queues: dict`, one `asyncio.Queue` per connection drained by a single serial `_send` task.

**Camera color inversion.** `BGR888` format produced R/B channel swap. Root cause: `capture_array()` on this Pi hardware returns RGB regardless of format name. Fix: `RGB888` format, no `cvtColor`, frame passed directly to `cv2.imencode`.

**Tag ID mismatch.** Physical tag is ID 0; config had `tag_id: 1`. Fix: corrected in `config.yaml`.

**Broadcast flood killing `_send`.** 30fps per-frame broadcasts (~60 msg/s) triggered transient WebSocket exceptions that silently killed `_send`. Fix: throttled per-frame broadcasts to 10fps; `_send` now logs errors and continues instead of dying.

### Limitations Carried into v2

- Camera pan/tilt locked to 0°. No gimbal tracking.
- Fixed forward speed regardless of distance.
- No latching stop — car resumes if tag moves back beyond threshold while chase is still active.
- No world-frame recording of trajectories.

---

## v2 — World TF (branch: `test/v2_world_tf`)

**Goal:** Establish a fixed world coordinate frame using a pair of AprilTags mounted to a wall, record the car's trajectory in that frame, and visualize it live in RViz2. Add a URDF so the robot renders correctly.

Reddit posts: [TODO: Reddit link]

### What Was Built

**Dual-tag world anchor.** Tags 2 and 3 are mounted to the wall at a known separation (~7cm on Y). The system searches for both simultaneously and validates geometric consistency (`world_validation_tol_m: 0.025m` on each axis) before locking in the world frame. World origin = midpoint between the two tags projected to the floor. Chase target remains tag 0.

**State machine:** `idle → world_search` (15s timeout, both tags required with geometric validation) `→ countdown (3-2-1) → chasing`. Falls back to v1 behavior via popup if world pair not found within timeout.

**tf_bridge ROS2 node.** Ubuntu-side node subscribes to the Pi WebSocket, computes `T_world_camera = inv(T_camera_tag1)`, and publishes:
- `world → camera` TF
- `/trajectory/car` and `/trajectory/tag0` as LINE_STRIP MarkerArrays (5 trajectory series total)
- Per-cycle PLY point cloud files (MeshLab-compatible, vertex-colored)

**URDF visualization.** `picar_description` ROS2 package added with a tracking URDF anchored to the `camera` TF frame. Car body, mount column, pan/tilt links, and 4 wheels as fixed joints. `robot_state_publisher` runs alongside tf_bridge via `launch_tf_bridge.sh`.

**FPS optimization.** `quad_decimate: 2.0`, `nthreads: 2` in the detector config — roughly 2x detection throughput at 640x480 on Pi 4.

**Manual Track mode.** WASD drive while tag detection and TF publishing run in the background. Raw camera-frame poses written to `cycle_N_raw_HHMMSS.json` on the Pi.

### Jitter Analysis

Trajectory plots showed sharp zig-zags with the car stationary. Root cause: AprilTag PnP pose ambiguity. The solver has two valid solutions for a planar tag and alternates between them frame-to-frame. Y-axis noise: ±15cm swing while stationary. The rotation-to-translation amplification makes it worse — at 74cm tag distance, 5° angular error = 6.5cm position error in world frame. No filtering was in the pipeline.

### Tag Size Experiment (2026-06-23)

Hypothesis: larger printed tags reduce PnP ambiguity by giving the corner detector more pixels to work with. Four sessions recorded (stationary camera, varying tag size and lighting):

| Condition | σ X | σ Y | σ Z (depth) |
|-----------|-----|-----|-------------|
| 5cm — lights off | 3.4 cm | 0.5 cm | 4.5 cm |
| 5cm — lights on | 5.1 cm | 1.7 cm | 3.7 cm |
| 20cm — lights on | **2.7 cm** | **0.4 cm** | **1.4 cm** |
| 20cm — lights off | 2.1 cm | 1.0 cm | 1.7 cm |

**Finding:** Tag size is the dominant factor. The 20cm tag reduces depth noise ~3x versus the 5cm tag. For 5cm tags, ambient lighting creates glare that degrades corner localization; for 20cm tags, lighting is a secondary concern. The PnP flip signature is visible as bimodal histograms in the 5cm distributions.

### Bugs Diagnosed and Fixed

**Manual Track button did nothing.** `tracker` missing from `global` declaration in `main()`.

**Motors froze mid-drive, steer held after freeze.** `_last_hb` was per-connection; browser reconnects created orphaned watchdog timers. Fix: promoted to module-level global; added `set_dir_servo_angle(0)` in watchdog.

**Double-shutdown traceback on Ctrl-C.** `rclpy.shutdown()` called twice. Fix: wrapped in try/except.

### Launch

```bash
# Pi
python3 ~/picar_ros/dashboard/server.py

# Ubuntu
./scripts/launch_tf_bridge.sh --ros-args -p pi_ws_url:=ws://192.168.1.241:8000/ws

# RViz (from a shell with install/setup.bash sourced)
rviz2 -d rviz/picar_trajectory.rviz
```

---

## v3 — IBVS (branch: `test/v3_ibvs`)

**Goal:** Add image-based visual servoing (IBVS) so the pan/tilt gimbal actively tracks the tag in pixel space. Eliminate the need for the dual-tag world pair — use tag 0 alone as a world anchor. Visualize the robot live in RViz with the full URDF including pan/tilt joint states.

Reddit posts: [TODO: Reddit link]

### What Was Built

**IBVS core.** Pan/tilt servo control driven by pixel error: `eu = tag_x - cx`, `ev = tag_y - cy`. Smoothed with EWMA (`ewma_alpha: 0.8`). 10px deadband and 30px lazy band prevent hunting. Step capped at 2°/frame for both axes. Confirmed working in `ibvs_test` and `manual_ibvs` modes.

**Four operational modes:**

| Mode | Description |
|------|-------------|
| `ibvs_test` | IBVS pan/tilt only, no drive, no TF. For servo tuning. |
| `manual_ibvs` | IBVS pan/tilt + manual WASD drive from dashboard. |
| `rat_chase` | IBVS pan/tilt + autonomous car centering + forward drive. Tag 0 only. |
| `world_ibvs` | Full v2+ behavior: tag pair world frame + countdown + TF chase. |

**ibvs_anchor_mode world frame.** First tag 0 detection seeds `T_world_anchor = T_world_camera_0 @ T_camera_tag0`. Every subsequent frame: `T_world_camera = T_world_anchor @ inv(T_camera_tag0)`. FK chain derives `car_base` pose from camera pose using physical offsets (`pan_z_m`, `tilt_z_m`, `cam_z_m`). Publishes `world → car_base` TF live.

**Joint state timer.** 10Hz timer in tf_bridge publishes zero joint states from node startup so `robot_state_publisher` can resolve the full TF chain before the Pi connects.

**Tag loss + raster re-acquisition.** On tag 0 loss, the system holds for `tag0_search_hold_s`, then runs a raster scan. Resumes IBVS immediately on reacquisition.

**Trajectory visualizer.** `trajviz.py` at repo root reads PLY files from tf_bridge, produces an interactive Plotly HTML (`trajviz_out.html`) with a red→yellow→green gradient, cubic spline overlay, and sliders for spline order (k=2..10) and smoothing weight. Best session: `car_cycle_0_003542.ply` — 454 points over 73 seconds of hand-carried movement.

### Bugs Diagnosed and Fixed

**TF never broadcast in ibvs_test/manual_ibvs.** `tf_pub.on_frame()` was only called inside `_do_chasing()`, which the ibvs_test/manual_ibvs branch never reaches. Pi was detecting the tag correctly but zero messages reached tf_bridge. Fix: added `tf_pub.on_frame()` directly in the ibvs_test/manual_ibvs block before `return`.

**Z filter blocking all ibvs_anchor frames (two rounds).** Round 1: filter checked `car_base_pos[2] < camera_height - tol` — car base is on the floor at Z≈0, so this always fired. Round 2: after switching to check camera height, valid cam_z readings clustered 0.000–0.054m, just below the 0.055m threshold. Every valid frame still rejected. Fix: made configurable via `ibvs_z_filter` ROS2 param (default false); threshold changed to `-camera_height_tol` so only gross PnP flips are caught when enabled.

**Velocity gate blocking all hand-carried movement.** The 10cm/frame velocity gate designed for autonomous driving rejected every step when the car was carried by hand. Result: `flip_skips=62` in an 11-second cycle, only 2 trajectory points recorded. Fix: added `ibvs_max_jump_m` param (default 1.0m) used exclusively in the ibvs_anchor path.

**World origin marker always red in RViz.** `/marker/world` publisher used Volatile QoS — published once on init, missed by RViz if subscribed later. Fix: changed to Transient Local QoS (`depth=1, TRANSIENT_LOCAL, RELIABLE`); RViz config updated to match.

**URDF wheels rendering as vertical posts.** Wheel visual `rpy="1.5708 0 0"` rotated the cylinder axis to +Y (vertical in world). Fix: changed to `rpy="0 1.5708 0"` for all four wheels — cylinder axis to +X, giving sideways discs.

**Joint states never published before Pi connects.** `_publish_joint_states` was only called inside `_process_frame`. Fix: added a 10Hz timer (`_js_timer_cb`) that publishes zero joint states immediately on node init.

### Current Status

IBVS pan/tilt tracking and single-tag world frame confirmed working. `rat_chase` autonomous drive logic is fully implemented but not yet tested on the floor — first supervised run recommended with a clear path from ~1m to the tag and kill switch ready.

### Key Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `ibvs_anchor_mode` | `true` | single-tag world frame |
| `ibvs_z_filter` | `false` | enable only when car stays on floor |
| `ibvs_pos_smooth_alpha` | `0.3` | EWMA on published car_base position |
| `ibvs_max_jump_m` | `1.0` | velocity gate for ibvs_anchor path |
| `kp_pan` / `kp_tilt` | `0.05` | IBVS proportional gains |
| `ewma_alpha` | `0.8` | pixel error smoothing |
| `deadband_px` | `10` | no correction within this radius |
| `pan_max_delta_deg` | `2` | max pan step per frame |
| `tilt_max_delta_deg` | `2` | max tilt step per frame |
| `car_centering_speed` | `20` | PWM for rat_chase forward drive |
| `stop_distance_cm` | `20` | rat_chase stop threshold |

### Launch

```bash
# Pi
python3 ~/picar_ros/dashboard/server.py

# Ubuntu (ibvs_anchor_mode is set in the script)
./scripts/launch_tf_bridge.sh --ros-args -p pi_ws_url:=ws://192.168.1.241:8000/ws

# RViz
rviz2 -d rviz/picar_trajectory.rviz

# Trajectory visualizer (after a session)
python3 trajviz.py
# open trajviz_out.html in browser
```

---

## Environment

| Component | Detail |
|-----------|--------|
| Pi | Raspberry Pi 4, Raspberry Pi OS (bookworm), Python 3.13 |
| Pi IP | 192.168.1.241 (user: jvpicar) |
| Camera | Pi Camera Module 3, Picamera2 0.3.36, RGB888 format |
| Detection | pupil-apriltags, tag36h11 family |
| Ubuntu | Ubuntu 22.04, ROS2 Humble, Python 3.10 |
| Ubuntu IP | 192.168.1.250 (user: jeano) |
| ROS2 workspace | `~/ros2_ws/` symlinked to `~/picar_ros/src/` |

```bash
# Build ROS2 packages
cd ~/picar_ros && colcon build --packages-select tf_bridge picar_description
source install/setup.bash
```
