# Tag Chaser v3 — IBVS: Debrief

---

## v3 IBVS Development — Session 2026-06-27/28

### Objective

Build image-based visual servoing (IBVS) on top of the v2 tag-chase foundation. The goal was a system where pan/tilt tracks tag0 using pixel error feedback, the car can physically drive toward the tag, and the Ubuntu RViz stack visualizes the car's trajectory in real time from tag0 alone — no tag pair required.

---

### Architecture

Four operational modes in `tag_chaser/v3_ibvs/chaser.py`:

| Mode | Description |
|---|---|
| `ibvs_test` | IBVS pan/tilt only, no motor drive, no TF. Used for servo tuning. |
| `manual_ibvs` | IBVS pan/tilt + manual WASD drive from dashboard. |
| `rat_chase` | IBVS pan/tilt + autonomous car centering + forward drive. Tag0 only. |
| `world_ibvs` | Full v2+ behavior: tag pair world frame + countdown + TF chase. |

Ubuntu side: `src/tf_bridge/tf_bridge/tf_bridge.py` connects to the Pi WebSocket, computes world-frame transforms, and publishes TF + RViz markers. A new `ibvs_anchor_mode` (ROS2 param, default true) uses tag0 as a stationary world anchor — no tag2/tag3 pair needed for the car to appear and move in RViz.

---

### What Works (Confirmed)

**IBVS core** — pan/tilt servo control via pixel error feedback. `eu = tag_x − cx`, `ev = tag_y − cy`. Smoothed with EWMA (`ewma_alpha: 0.8`). Deadband (10px) and lazy band (30px) prevent hunting. Step capped at `pan_max_delta_deg: 2` / `tilt_max_delta_deg: 2` per frame. Confirmed working in `ibvs_test` and `manual_ibvs` modes. Steer feed-forward from pan angle implemented (`pan_steer_ff_gain`, currently 0.0).

**ibvs_anchor_mode world frame** — first tag0 detection seeds `T_world_anchor = T_world_camera_0 @ T_camera_tag0`. Every subsequent frame: `T_world_camera = T_world_anchor @ inv(T_camera_tag0)`. FK chain (`pan_z_m`, `tilt_z_m`, `cam_z_m`) derives `car_base` pose from camera pose. Publishes `world → car_base` TF live, enabling the full URDF to track in RViz.

**EWMA smoothing on TF** — `ibvs_pos_smooth_alpha` (default 0.3) applied to published `car_base` position. Separate from the pair-path alpha. Configurable at runtime via `ros2 param set /tf_bridge ibvs_pos_smooth_alpha <value>`.

**RViz visualization** — robot model renders with correct URDF (pan/tilt joints, chassis, wheels). Car trajectory (`/trajectory/car`) and tag0 trajectory (`/trajectory/tag0`) publish as LINE_STRIP MarkerArrays per cycle. World origin sphere published on `ibvs_anchor_mode` init with Transient Local QoS so late-subscribing RViz sees it.

**Joint state timer** — 10 Hz timer in tf_bridge publishes zero joint states from node startup, so `robot_state_publisher` can resolve the full TF chain before Pi connects.

**Tag0 loss + scan re-acquisition** — `_handle_tag0_lost` holds for `tag0_search_hold_s` then runs raster scan. On reacquisition, IBVS resumes immediately.

**Frame saving** — configurable via `save_frames: false` in `config.yaml`. Defaults off; disk was being hammered during IBVS testing.

**Trajectory visualizer** — `trajviz.py` at repo root: reads PLY files output by tf_bridge, produces interactive Plotly HTML (`trajviz_out.html`) with red→yellow→green gradient, cubic spline overlay (k=3), and two sliders (spline order k=2..10, smoothing s=none..heavy). Best session: `car_cycle_0_003542.ply` — 454 points over ~73 seconds of hand-carried movement.

---

### Bugs Found and Fixed

**Bug 1 — TF never broadcast in ibvs_test/manual_ibvs**
`tf_pub.on_frame()` was only called inside `_do_chasing()`, which is only reached from the `rat_chase`/`world_ibvs` path. The `ibvs_test`/`manual_ibvs` branch exits with `return` before `_do_chasing` is invoked. Pi was detecting tag0 correctly but zero `tag_detections` messages reached tf_bridge. Fix: added `tf_pub.on_frame()` call directly in the ibvs_test/manual_ibvs block before the `return`.

**Bug 2 — Z filter blocking all ibvs_anchor frames (two rounds)**
Round 1: original filter checked `car_base_pos[2] < camera_height − tol` (≈ 0.055m). Car base is on the floor at Z≈0; this condition always fired. Fix: changed to check `cam_z = T_world_camera[2,3]` (camera height in world). Round 2: `camera_height=0.075m`, `tol=0.020m`, threshold=0.055m. Valid cam_z readings clustered 0.000–0.054m — just below threshold. Every valid frame still rejected. User physically lifts the car during testing so Z filtering is inappropriate. Fix: made configurable via `ibvs_z_filter` ROS2 param (default false). Threshold changed to `-camera_height_tol` so only gross PnP flips (cam_z < −0.020m) are rejected when enabled.

**Bug 3 — Velocity gate blocking all hand-carried movement**
`max_position_jump_m = 0.10m` (10cm) is designed for the pair-based path where the car drives autonomously. When physically carrying the car, every step exceeds 10cm. Result: `flip_skips=62` in an 11-second cycle, only 2 trajectory points recorded. `_pos_smooth` never updated → `world → car_base` TF stale → PiCar display red, TF Warn in RViz. Fix: added `ibvs_max_jump_m` param (default 1.0m) used exclusively in the ibvs_anchor path.

**Bug 4 — World Origin display always red in RViz**
`/marker/world` publisher used default Volatile QoS. Published once on ibvs_anchor init; RViz missed it if subscribed later. Fix: publisher changed to Transient Local (`QoSProfile(depth=1, durability=TRANSIENT_LOCAL, reliability=RELIABLE)`). RViz subscriber durability updated to match in `rviz/picar_trajectory.rviz`.

**Bug 5 — URDF wheels rendering as vertical posts**
Wheel visual `rpy="1.5708 0 0"` rotates the cylinder axis to +Y (down in camera convention) = vertical in world. Wheels appeared as upright cylinders. Fix: changed to `rpy="0 1.5708 0"` for all four wheels, rotating cylinder axis to +X (lateral), giving sideways discs.

**Bug 6 — joint_states never published before Pi connects**
`_publish_joint_states` was only called inside `_process_frame`, which requires WebSocket data. `robot_state_publisher` had no TF for pan_link/tilt_link until first frame arrived. Fix: added 10 Hz timer (`_js_timer_cb`) that publishes zero joint states immediately on node init.

---

### Current Parameter Defaults

| Parameter | Default | Notes |
|---|---|---|
| `ibvs_anchor_mode` | `true` | set in `launch_tf_bridge.sh` |
| `ibvs_z_filter` | `false` | enable only when car stays on floor |
| `ibvs_pos_smooth_alpha` | `0.3` | EWMA on published car_base position |
| `ibvs_max_jump_m` | `1.0` | velocity gate for ibvs_anchor path |
| `pan_max_delta_deg` | `2` | max pan step per frame |
| `tilt_max_delta_deg` | `2` | max tilt step per frame |
| `kp_pan` / `kp_tilt` | `0.05` | IBVS proportional gains |
| `ewma_alpha` | `0.8` | pixel error smoothing |
| `deadband_px` | `10` | no correction within this radius |
| `save_frames` | `false` | JPEG buffer to disk |
| `car_centering_speed` | `20` | PWM for rat_chase forward drive |
| `stop_distance_cm` | `20` | rat_chase stop threshold |

---

### rat_chase Status

Logic fully implemented. IBVS (pan/tilt) and tag0 detection confirmed working. Car centering (`_run_car_centering`: pan angle → steering + `px.forward(car_centering_speed)`) and PID fallback are coded but **not yet tested** — ibvs_test and manual_ibvs both return before `_do_chasing` is reached. First supervised run recommended: clear floor, car pointing at tag from ~1m, stop at kill switch.

---

### Session Log

| Date | Mode | Notes |
|---|---|---|
| 2026-06-27 | ibvs_test | First IBVS runs. TF broadcast bug discovered — zero messages from Pi despite tag detection. |
| 2026-06-27 | ibvs_test | Z filter bug (round 1 + round 2) — all frames rejected. Made configurable, defaulted off. |
| 2026-06-27 | ibvs_test | After Z fix: world frame generating, car appears in RViz. Velocity gate bug discovered (flip_skips=62, 2 pts/cycle). |
| 2026-06-27 | ibvs_test | After velocity gate fix (`ibvs_max_jump_m=1.0m`): first clean tracking session. |
| 2026-06-28 | ibvs_test | `car_cycle_0_003542.ply` — 454 points, 73s, hand-carried. Best trajectory to date. Visualized with trajviz.py. |

---

# Tag Chaser v2 — World TF: Debrief

## Status

Active development. Manual Track and autonomous chase confirmed working. tf_bridge Ubuntu node substantially extended with URDF visualization, floor-anchored world frame, and per-cycle PLY export for MeshLab.

---

## What Works

**ManualTracker** — drive with WASD while tag detection + TF publishing runs in the background. Confirmed working in session `pi_session_20260613_115413`. Produces `cycle_N_raw_HHMMSS.json` on the Pi side (camera-frame poses).

**TagChaser v2** — dual-tag detection, world-search gate, PID steering, countdown. Confirmed working in session `pi_session_20260613_112645`.

**Dashboard** — Manual Track toggle button, track status bar, live steer gauge during autonomous chase, world_not_found popup, Stop Session shutdown.

**tf_bridge** — Ubuntu ROS2 node connects to Pi WebSocket, computes floor-anchored world-frame TF, publishes `/trajectory/car` and `/trajectory/tag0` LINE_STRIP markers to RViz2. Also publishes static `world` → `tag1` TF on world initialization.

**Floor-anchored world frame** — World origin is the floor point directly below the camera at first tag1 detection. Axes: X = along wall, Y = toward wall, Z = up. `camera_height_m` ROS param (default 0.075m) controls the floor offset. Tag1 (physically fixed to the wall) is expressed as a static TF `world` → `tag1` and never changes within a session. World is re-initialized on `/reset_markers` service call.

**URDF visualization** — `picar_description` ROS2 package (`src/picar_description/`) provides a tracking URDF anchored to the `camera` TF frame. Robot mesh appears live in RViz2 following the camera's world-frame position. Launched automatically by `launch_tf_bridge.sh`.

**PLY point cloud export** — At the end of each cycle (and on Ctrl-C for the last), tf_bridge writes `car_cycle_N_HHMMSS.ply` and `tag0_cycle_N_HHMMSS.ply` to the session log directory. Files are MeshLab-compatible ASCII PLY with vertex color (car = green, tag0 = red). Open in MeshLab: File → Import Mesh → Render → Color → Per Vertex.

**Session logging** — Pi logs land in `~/picar_ros/logs/pi_session_*/`. Ubuntu tf_bridge writes world-frame cycle JSON and per-cycle PLY to `~/picar_ros/logs/session_*/`.

---

## Bugs Found and Fixed

### Bug 1 — Manual Track button did nothing (2026-06-13)

`tracker` missing from `global` declaration in `main()`. Fix: added to global list. Also added explicit log breadcrumbs in `manual_track` handler and broadened exception handling in `_recv`.

### Bug 2 — Motors froze mid-drive; steer held angle after freeze (2026-06-13)

`_last_hb` was per-connection; browser reconnects created orphaned watchdog timers. Fix: promoted `_last_hb` to module-level global. Also added `px.set_dir_servo_angle(0)` in `_watchdog()` so steer resets on freeze.

### Bug 3 — Double-shutdown traceback on Ctrl-C (2026-06-13)

`rclpy.shutdown()` called twice — once by the executor's spin thread (on SIGINT) and once by the `finally` block. Fix: `executor.shutdown(wait=False)` and `rclpy.shutdown()` wrapped in try/except. Exit code is now clean.

---

## Jitter Analysis (2026-06-13)

Trajectory plots show sharp zig-zags that are measurement noise, not real motion. Root cause chain:

1. **AprilTag PnP pose ambiguity** — the solver has two valid solutions for a planar tag and flips between them frame-to-frame. One axis (Y in the sessions analyzed) swings ±15cm per frame while the car is stationary. This is the primary source of zig-zags.
2. **Rotation-to-translation amplification** — `T_world_camera = inv(T_camera_tag1)`. A small angular error δθ becomes `|t| × sin(δθ)` of position noise in world frame. At ~74cm tag distance, 5° error = 6.5cm position noise.
3. **No filtering** — every raw frame goes straight to TF and trajectory. One bad frame = a spike on the trajectory.

An EWMA + velocity-gate filter was prototyped and tested but removed at user's request — deferred to a future session.

**Mitigation without code changes**: ensure tag1 is squarely in frame, well-lit, and printed at a larger size.

---

## Known Issues / Next Steps

**Jitter** — AprilTag PnP pose ambiguity produces ~7cm std dev noise in the worst axis when stationary. An EWMA filter + velocity gate in `_process_frame` would address this. Deferred.

**World_lost gaps during driving** — Pi-side logs show rapid `world_acquired`/`world_lost` transitions (6+ in 15s) when tag1 leaves frame during turns. tf_bridge drops those frames. A short grace-period hold of the last known transform would prevent TF gaps in RViz. Deferred.

**URDF joint offsets need visual tuning** — The `camera` → `car_body` offsets in `src/picar_description/urdf/picar_tracking.urdf` are approximate. Tune them visually once the system is running with RViz.

**`picar-jv` hostname not resolving on Ubuntu** — DNS doesn't resolve `picar-jv`. Use IP directly (`ws://192.168.1.241:8000/ws`) or add once to `/etc/hosts`: `echo "192.168.1.241 picar-jv" | sudo tee -a /etc/hosts`.

**RViz must be opened from a sourced shell** — RViz needs `~/picar_ros/install/setup.bash` sourced to find `picar_description`. The `world` TF frame won't exist until the first tag1 detection; RViz will briefly warn "Fixed Frame [world] does not exist" during reconnection.

**tf_bridge must be running during the session** — No offline replay tooling yet. World-frame math could be applied post-hoc to the Pi's raw JSON, but that script isn't built.

---

## Tag Size Ambiguity Experiment (2026-06-23)

### Motivation

The jitter analysis from 2026-06-13 identified AprilTag PnP pose ambiguity as the primary noise source (~7cm std dev when stationary). The hypothesis was that printing larger tags would reduce ambiguity by giving the corner-detection step more pixels to work with, reducing the angular error that gets amplified into position noise.

### Method

A `single_tag_world_mode` config flag was added to `config.yaml` (under `chase:`). When `true`, the system requires only `tag_id_world_a` (tag 2) to be visible and skips the geometric pair-validation gate. This lets ManualTracker run and record with a single tag in frame.

For each condition the camera was held stationary pointing directly at the tag for ~2–3 minutes. All raw camera-frame poses were recorded automatically by `TfPublisher` to `cycle_N_raw_HHMMSS.json`. Tag size was set via `camera.tag_size_m` in `config.yaml` (0.05 for 5cm, 0.20 for 20cm).

A plotting script (`tag_chaser/v2_world_tf/plot_poses.py`) was written to load the cycle JSON files and produce per-session time-series and distribution plots. A second set of plots with globally-unified axis scales was generated to enable direct visual comparison across conditions.

### Sessions

| Session directory | Tag size | Lighting | Frames | Duration |
|---|---|---|---|---|
| `pi_session_20260623_071720_5cm_01_lights off` | 5cm | Off | 5026 | 174.7s |
| `pi_session_20260623_072234_5cm_02_lights_on` | 5cm | On | 6005 | 205.3s |
| `pi_session_20260623_074929_20cm_01_lights_on` | 20cm | On | 4363 | 145.9s |
| `pi_session_20260623_075251_20cm_02_lights_off` | 20cm | Off | 4513 | ~150s (idealdataset cycle only) |

The 20cm lights-off session produced two cycles; only `cycle_1_raw_075550_idealdataset.json` was used for analysis.

### Results

Translation std dev (camera frame, stationary camera):

| Condition | σ X | σ Y | σ Z (depth) |
|---|---|---|---|
| 5cm — lights off | 3.4 cm | 0.5 cm | 4.5 cm |
| 5cm — lights on | 5.1 cm | 1.7 cm | 3.7 cm |
| 20cm — lights on | **2.7 cm** | **0.4 cm** | **1.4 cm** |
| 20cm — lights off (idealdataset) | 2.1 cm | 1.0 cm | 1.7 cm |

### Key Findings

**Tag size is the dominant factor.** The 20cm tag reduces depth noise by ~3x compared to the 5cm tag under comparable lighting. X and Y noise also improve, though less dramatically.

**Lighting interacts differently with tag size.** For the 5cm tag, lights-on is worse (σX 5.1 vs 3.4cm), likely because uncontrolled ambient light creates glare that degrades corner localization on a small tag. For the 20cm tag, lights-on is marginally better on depth but worse on X -- results are close enough that lighting is a secondary concern once the tag is large enough.

**20cm + lights on is the best-performing condition** across all three axes simultaneously (σX=2.7cm, σY=0.4cm, σZ=1.4cm).

**The PnP flip signature is visible in the distribution plots.** The 5cm sessions show broad, sometimes bimodal histograms on X and Z, consistent with the solver alternating between its two valid solutions. The 20cm sessions show tighter, more unimodal distributions.

### Plots

Saved to `logs/`:
- `plot_5cm_01_lights_off.png` / `scaled_5cm_01_lights_off.png`
- `plot_5cm_02_lights_on.png` / `scaled_5cm_02_lights_on.png`
- `plot_20cm_01_lights_on.png` / `scaled_20cm_01_lights_on.png`
- `plot_20cm_02_lights_off.png` / `scaled_20cm_02_lights_off.png`

`scaled_*` versions share identical axis limits across all four plots for direct visual comparison.

---

## Session Log

| Date | Session | Notes |
|------|---------|-------|
| 2026-06-13 | `pi_session_20260613_110850` | Manual Track did nothing — tracker global bug (pre-fix) |
| 2026-06-13 | `pi_session_20260613_112645` | Tag chase start confirmed; Manual Track still broken |
| 2026-06-13 | `pi_session_20260613_115413` | Manual Track working (post-fix); 3 watchdog fires |
| 2026-06-13 | tf_bridge `105526` | tf_bridge connected to Pi, published world origin |
| 2026-06-13 | `session_20260613_143848` | First full dual-tag world-frame trajectory session; jitter analyzed |

---

## Data Locations

Pi (camera-frame raw poses):
`~/picar_ros/logs/pi_session_YYYYMMDD_HHMMSS/cycle_N_raw_HHMMSS.json`

Ubuntu (world-frame JSON, written by tf_bridge during session):
`~/picar_ros/logs/session_YYYYMMDD_HHMMSS/cycle_N_world_HHMMSS.json`

Ubuntu (MeshLab PLY point clouds, written at cycle end):
`~/picar_ros/logs/session_YYYYMMDD_HHMMSS/car_cycle_N_HHMMSS.ply`
`~/picar_ros/logs/session_YYYYMMDD_HHMMSS/tag0_cycle_N_HHMMSS.ply`

---

## Launch

```bash
# Ubuntu — starts robot_state_publisher + tf_bridge together
./scripts/launch_tf_bridge.sh --ros-args -p pi_ws_url:=ws://192.168.1.241:8000/ws

# RViz (from same sourced shell, or a shell with install/setup.bash sourced)
rviz2 -d rviz/picar_trajectory.rviz
```

Tune camera height if needed:
```bash
./scripts/launch_tf_bridge.sh --ros-args -p pi_ws_url:=ws://192.168.1.241:8000/ws -p camera_height_m:=0.082
```
