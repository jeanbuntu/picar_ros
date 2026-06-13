# Tag Chaser v2 — World TF: Debrief

## Status

Active development. Manual Track mode working as of 2026-06-13. Autonomous chase working. Two bugs identified and fixed after first real sessions.

---

## What Works

**ManualTracker** — drive with WASD while tag detection + TF publishing runs in the background. Confirmed working in session `pi_session_20260613_115413`. Produces `cycle_N_raw_HHMMSS.json` on the Pi side (camera-frame poses). World-frame JSON requires tf_bridge running on Ubuntu during the session.

**TagChaser v2** — dual-tag detection, world-search gate, PID steering, countdown. Confirmed working in session `pi_session_20260613_112645` (chase_start fired, countdown began). Chase is fast — ManualTracker is the recommended mode for debugging and data collection.

**Dashboard** — Manual Track toggle button, track status bar, live steer gauge during autonomous chase, world_not_found popup, Stop Session shutdown.

**tf_bridge** — Ubuntu ROS2 node connects to Pi WebSocket, computes world-frame TF, publishes `/trajectory/car` and `/trajectory/tag0` LINE_STRIP markers to RViz2. Confirmed working against mock server and against Pi (session `105526` on 2026-06-13).

**Session logging** — Pi logs land in `~/picar_ros/logs/pi_session_YYYYMMDD_HHMMSS/` (prefix `pi_session_` distinguishes them from Ubuntu tf_bridge sessions). Ubuntu tf_bridge writes world-frame cycle JSON alongside its own session logs.

---

## Bugs Found and Fixed

### Bug 1 — Manual Track button did nothing (2026-06-13)

`tracker` was missing from the `global` declaration in `main()`. `tracker = ManualTracker(...)` created a local variable; module-level `tracker` stayed `None`. Every `if tracker:` in `_recv` silently no-op'd. `ManualTracker initialized` still appeared in logs because that line ran before the assignment, making it look correct.

Fix: added `tracker` to `global px, chaser, tracker, _picam2, _session_dir, _chase_config_v1` in `main()`.

Also added explicit `_logger.info/error` breadcrumbs throughout the `manual_track` handler so future issues are immediately visible, and broadened the `_recv` exception handler from `except (WebSocketDisconnect, RuntimeError)` to also catch all `Exception` with traceback logging.

### Bug 2 — Motors froze mid-drive; wheel kept spinning after freeze (2026-06-13)

Three watchdog fires at 11:55:04, 11:55:45, 11:56:35 during session `pi_session_20260613_115413`.

Root cause: `_last_hb = [time.monotonic()]` was per-connection (inside `ws_handler`). The browser reconnected at 11:54:59, creating a new WS connection. The browser sent heartbeats to the new connection only; the old connection got none. Five seconds later its watchdog fired and called `px.stop()`. This repeated each time the browser reconnected.

Additionally, `_watchdog()` called `px.stop()` but not `px.set_dir_servo_angle(0)`. The steer servo held its last angle (up to 30°) after the freeze, so when the user pressed forward again the car arced.

Fixes applied to `dashboard/server.py`:
- `_last_hb` promoted to module-level global. Any connection's heartbeat updates the shared clock; orphaned connection watchdogs can no longer fire independently.
- `px.set_dir_servo_angle(0)` added inside `_watchdog()` after `px.stop()`.

---

## Known Issues / Next Steps

**World tag flapping** — Tag 1 (world anchor) shows rapid `world_acquired`/`world_lost` cycling (multiple times per second) when it is near the edge of the camera frame or at a borderline confidence level. This produces spurious TF gaps in RViz. Fix: keep Tag 1 more squarely in frame, print it larger, or improve lighting. Could also lower `confidence_threshold` in `config.yaml` (currently `20.0`) with the tradeoff of noisier pose estimates.

**tf_bridge must be running during the session** — No offline replay tooling exists yet. If tf_bridge wasn't running, only the raw camera-frame JSON from the Pi is available. The world-frame TF math could be applied post-hoc in a Python/matplotlib script, but that isn't built.

**SCP to Ubuntu from Pi** — The hardcoded Ubuntu IP in `server.py` was stale (192.168.1.250 got "No route to host"). Confirm Ubuntu IP before sessions that need SCP. Consider removing the auto-SCP and doing it manually.

**Ubuntu IP discovery** — Use `ip addr` on Ubuntu or check router DHCP leases. tf_bridge launch script uses the Pi IP (`ws://192.168.1.241:8000/ws`), which is stable; Ubuntu IP only matters for SCP.

---

## Session Log

| Date | Session | Notes |
|------|---------|-------|
| 2026-06-13 | `pi_session_20260613_110850` | Manual Track did nothing — tracker global bug (pre-fix) |
| 2026-06-13 | `pi_session_20260613_112645` | Tag chase start confirmed; Manual Track still broken |
| 2026-06-13 | `pi_session_20260613_115413` | Manual Track working (post-fix); 3 watchdog fires; cycle_0_raw_115427.json has ~1130 frames |
| 2026-06-13 | tf_bridge `105526` | tf_bridge connected to Pi, published world origin; SCP to Ubuntu failed |

---

## Data Locations

Pi (camera-frame raw poses):
`~/picar_ros/logs/pi_session_YYYYMMDD_HHMMSS/cycle_N_raw_HHMMSS.json`

Ubuntu (world-frame, written by tf_bridge during session):
`~/picar_ros/logs/session_YYYYMMDD_HHMMSS/cycle_N_world_HHMMSS.json`

## Live Visualization

```bash
# Ubuntu terminal 1
./scripts/launch_tf_bridge.sh --ros-args -p pi_ws_url:=ws://192.168.1.241:8000/ws

# Ubuntu terminal 2
rviz2 -d rviz/picar_trajectory.rviz
```
