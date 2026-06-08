# PRD — Phase 0.5: The Control Website (Mobile + Desktop)

Project: PiCar-X "Chase the Cat"
Phase: 0.5 of 3 (cross-cutting component; built within Phase 1, spans into Phase 2)
Status: Not started
Owner: (you)
Last updated: 2026-06-02
Related: Phase 1 (consumes this as its control surface), Phase 2 (changes only the video source)

---

## 1. Purpose

Specify the **control website** as a standalone component: a single web page, served from
the Pi, that shows the live camera feed and exposes the full control surface for the car.
It is the human interface to the whole project — the thing you actually hold in your hand
to drive, supervise, and emergency-stop the car.

It is broken out into its own PRD because:
- It is a **distinct artifact** (a web server + page) with its own tech choices, parity
  rules, and safety responsibilities, independent of the behavior/detection work.
- It **spans phases**: the same page is used in Phase 1 (video from vilib) and Phase 2
  (video from the self-hosted pipeline). Only the video source changes; the controls do
  not. Writing it standalone makes that continuity explicit and prevents the GUI from
  being rebuilt at migration.
- It carries the **parity requirement** — every desktop keyboard action must also be a
  touch control, and vice versa — which is a cross-cutting rule, not a per-phase one.

---

## 2. Scope

### In scope
- A single web page: live video + all controls together (one screen, no tabs).
- Works on **phone (touch)** and **desktop (keyboard + click)**, with **action parity**
  between the two input methods.
- The control web server on the Pi exposing endpoints the page calls.
- A **video source abstraction** so the same page works with vilib's stream (Phase 1) and
  the self-hosted stream (Phase 2) by changing one source URL.
- Safety-critical UI: always-visible, reliable kill button; battery readout; connection
  status; current-mode/state display.
- Responsive layout for one-handed/two-thumb phone use.

### Out of scope
- The behavior state machine, detection, and motor control logic (Phases 1/2). This PRD
  covers the **interface and its server endpoints**, not what the car decides to do.
- The video *producer* (vilib in Phase 1, picamera2 pipeline in Phase 2). This PRD only
  *consumes* a stream URL.
- Authentication / multi-user. Assumed single trusted user on a home LAN (§9).

---

## 3. Success criteria (exit conditions)

The control website is **done** when:

1. On a phone on the same Wi-Fi, opening one URL shows the live feed and lets you fully
   operate the car — drive, steer, pan/tilt, switch mode, start/stop session, capture
   photo, toggle detection, and kill — on a single screen.
2. On desktop, the **same page** accepts keyboard input (WASD/IJKL + the mode/session/kill
   keys) and exposes the identical actions as clickable controls.
3. **Parity is verified**: a checklist confirms every desktop action has a touch control
   and every touch control has a desktop equivalent (§5.2).
4. The **kill button stops the car reliably** and is reachable at all times without
   scrolling.
5. The page shows live **battery**, **connection status**, and **current mode/state**.
6. Swapping the video source URL (vilib ↔ self-hosted) requires **no other change** to the
   page (§6).
7. Latency is acceptable for supervised manual driving (expectations set, §8).

---

## 4. Architecture

```
        Phone browser (touch)          Desktop browser (keyboard + click)
                 │                                   │
                 └──────────────┬────────────────────┘
                                │  same single page, same endpoints
                                ▼
                  Control web server (on the Pi)
                   ├── GET  /            → the single-page UI
                   ├── video source  → <img> points at stream URL (swappable)
                   ├── POST /drive      → forward/back/left/right/stop
                   ├── POST /camera     → pan/tilt/recenter
                   ├── POST /mode       → aggressive | gentle
                   ├── POST /session    → start | stop (standby)
                   ├── POST /kill       → emergency stop → standby (privileged)
                   ├── POST /photo      → capture still
                   ├── POST /detect     → detection on/off
                   ├── GET  /status     → battery, mode, state, conn (poll/websocket)
                   └── (watchdog: server stops car if no client heartbeat in N s)
                                │
                                ▼
                  Behavior/control layer (Phase 1/2) ── Picarx
```

### Key decisions
- **Page is dumb; car is smart.** The page sends intents to endpoints; all decision logic
  lives server-side (Phase 1/2). This keeps the client thin and the same page valid across
  phases.
- **Video is an embedded stream, not part of the control protocol.** The page shows the
  feed via an `<img>`/video element pointed at a **source URL**. Controls go through the
  endpoints above. This separation is what lets the video producer change between phases
  with zero control-side impact.
- **Single server preferred in Phase 2.** Phase 1 may keep video on vilib's :9000 and
  controls on a separate port (page embeds the cross-port stream). Phase 2 folds video into
  the control server. The page handles both via the source-URL abstraction (§6).

---

## 5. The control surface (parity is the core requirement)

### 5.1 Parity rule
One control model, two input methods. **Every action must be operable by both** keyboard
(desktop) and touch (phone). The page is the same on both; desktop additionally binds keys.

> **ASSUMPTION (confirm/correct):** The existing desktop control is the SSH keyboard
> script approach (`3.keyboard_control.py` from setup history), not an already-served web
> page. This PRD therefore treats the website as a **new build** that replicates the
> existing action set and adds touch controls + keyboard handling in the browser. **If a
> served web page already exists, this becomes a responsive re-skin + endpoint-wiring job
> instead — correct here and scope shrinks.**

### 5.2 Action map (must appear for BOTH input methods)

| Group | Desktop key | Touch control | Endpoint | Action |
|---|---|---|---|---|
| Drive | W / S | d-pad up / down | POST /drive | forward / backward |
| Steer | A / D | d-pad left / right | POST /drive | steer left / right |
| Drive stop | (key release / X) | release / stop button | POST /drive | stop |
| Camera tilt | I / K | cam-pad up / down | POST /camera | tilt up / down |
| Camera pan | J / L | cam-pad left / right | POST /camera | pan left / right |
| Camera recenter | (key TBD) | recenter button | POST /camera | center pan/tilt |
| Mode | (key TBD) | toggle switch | POST /mode | aggressive ↔ gentle |
| Session | (key TBD) | start/stop button | POST /session | enter autonomy / standby |
| Photo | Q | camera button | POST /photo | capture still |
| Detection | (key TBD) | on/off toggle | POST /detect | detection on/off |
| **Kill** | SPACE | **large kill button** | POST /kill | emergency stop → standby |

Decision recorded (from brainstorming): **photo capture and detection on/off ARE included**
on the phone (useful while supervising). Remaining vilib debug keys (QR scan, face detect,
color-channel select) are **desktop/debug-only** and not required on the phone — confirm if
you want the full vilib key set mirrored.

### 5.3 Status display (read-only, on the page)
| Readout | Source | Notes |
|---|---|---|
| Battery | GET /status | Already implemented on the car; surface it. Warn at low charge. |
| Current mode | GET /status | aggressive / gentle |
| Current state | GET /status | standby / search / reacquire / chase / play / avoid / manual |
| Connection | client-side | shows if heartbeat to server is healthy (ties to watchdog) |

---

## 6. Video source abstraction (the phase-spanning bit)

The single requirement that makes this page survive the Phase 1→2 migration:

- The page references the live feed by a **single configurable source URL**, not a
  hardcoded `http://<pi>:9000/mjpg`.
- **Phase 1:** source = vilib's MJPEG stream (`:9000/mjpg`), possibly cross-port.
- **Phase 2:** source = the self-hosted MJPEG endpoint on the control server itself.
- Changing phases = changing that one URL (config value), nothing else on the page.
- Optional Phase 2 nicety: the self-hosted stream can show the **annotated** frame
  (detection bbox drawn), turning the phone view into a live debug view. The page doesn't
  care — it just renders whatever the source sends.

---

## 7. Safety responsibilities of the UI

The website is part of the safety system, not just a convenience:

1. **Kill button** — large, fixed in the layout, reachable without scrolling, visually
   distinct. Hits the privileged `/kill` endpoint. Must work even if other UI is sluggish.
2. **Connection heartbeat** — the page sends a periodic heartbeat; the server's watchdog
   stops the car if it stops hearing it (N seconds). The page also *shows* connection
   health so a degrading link is visible before it fails.
3. **Manual preempt is instant** — any drive/camera input immediately preempts autonomy
   server-side (the page just sends the intent; the server enforces preemption).
4. **No accidental kill-defeat** — the kill button should not be placed where a thumb
   resting on the drive pad can't reach it, nor where it's easily hit by accident in a way
   that disrupts legitimate driving. Balance "always reachable" vs "not fat-fingered."
5. **Low-battery warning** — visible alert so a session doesn't die mid-room.

---

## 8. Non-functional requirements

| Property | Target / note |
|---|---|
| Latency | MJPEG-over-Wi-Fi has visible lag; acceptable for *supervised* manual driving. Set expectation: you're steering on a slightly delayed feed. Improvement options are out of scope here. |
| Responsiveness | One page, no horizontal scroll on phone; controls sized for thumbs; kill button always visible. |
| Browser support | Mobile Safari + Chrome (iOS/Android), desktop Chrome/Safari. No app install — it's a web page. |
| Footprint | Lightweight; the Pi is also running vision + control. Avoid heavy frameworks; vanilla or minimal is fine. |
| Statelessness | Page holds no authority; server is source of truth for mode/state. Refreshing the page must not change the car's state. |
| No browser storage | (If built as an artifact-style page) keep state in memory; don't rely on localStorage. For the real deployed page on the Pi this is unrestricted, but keep it simple. |

---

## 9. Risks and assumptions

| Risk / assumption | Status | Mitigation |
|---|---|---|
| Existing control is keyboard script, not web page | ASSUMED | New build; correct §5.1 if a page exists. |
| Cross-port video embed (Phase 1) blocked by browser | Possible | Embed vilib stream as `<img src>` (MJPEG works cross-port); verify no mixed-content/CORS issue on the LAN. |
| Kill button latency if server busy | High-impact | `/kill` is privileged/fail-safe; watchdog is the backstop if the request itself can't get through. |
| Wi-Fi dead zones drop the page mid-drive | Handled | Heartbeat + server watchdog auto-stops the car; page shows connection health. |
| No auth on the LAN | Accepted | Single trusted user, home network. Not internet-exposed. Note as a conscious decision. |
| Parity drift (an action added to one input method only) | Process risk | The §5.2 action map is the single source of truth; verify against it at completion. |

---

## 10. Deliverables

1. **Single-page control UI** — video + full control surface + status, responsive, phone +
   desktop, parity verified against §5.2.
2. **Control web server** — the endpoints in §4, with the privileged `/kill` and the
   client-heartbeat watchdog.
3. **Video source abstraction** — one config point for the stream URL (§6).
4. **Safety UI** — kill button, connection health, low-battery warning, mode/state display.
5. **Parity checklist** — completed, confirming desktop↔touch action equivalence.

---

## 11. Open questions

- **CONFIRM:** Existing desktop control = keyboard script (assumed) or existing web page?
  Biggest scope determinant (§5.1).
- **CONFIRM:** Mirror the *full* vilib debug key set to phone, or just the core set +
  photo + detection toggle (assumed)? (§5.2)
- **CONFIRM:** Desktop key bindings for mode / session / recenter / detection (currently
  TBD in the action map).
- **OPEN:** Heartbeat interval and watchdog timeout N (recommend ~1 s heartbeat, ~2 s
  watchdog; tune in practice).
- **OPEN:** Phase 1 — keep controls on a separate server from vilib's video, or piggyback
  vilib's server? (Affects whether the video embed is cross-port.)
- **OPEN:** Exact kill-button placement that is always-reachable but not fat-finger-prone
  (§7.4).
