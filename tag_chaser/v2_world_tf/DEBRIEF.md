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
