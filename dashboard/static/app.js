'use strict';

const piHost = window.location.hostname;
const VIDEO_SRC = `http://${piHost}:9000/mjpg`;

let ws = null;

// Drive state
const heldKeys = new Set();

// Gimbal state
const gimbalAngles = { pan: 0, tilt: 0 };
const gimbalIntervals = {};
const driveIntervals  = {};
const DRIVE_REPEAT_MS = 100;
const GIMBAL_STEP = 5;
const GIMBAL_MS = 80;

// Servo state
let steerAngle = 0;

// Mode / detect state
let modeState = 'gentle';
let detectOn = false;
let _battWarnShown = false;

// Tag chase state
let chaseActive = false;

// Motor display state
const motorState = { leftPct: 0, rightPct: 0, dir: 'stop' };

// Ultrasonic signal buffer (last 5 readings)
const distBuf = [];
const DIST_BUF_SIZE = 5;

// DOM refs
let speedSlider, speedValue, photoBtn, recBtn, sessionBtn;
let connStatus, recStatus, sessionStatus, notifEl;
let distanceVal, distanceBar, usSignal, batteryVal, batteryBar;
let motorLBar, motorRBar, motorLPct, motorRPct, motorLDir, motorRDir;
let gsVals, gsBars;
let keyEls;
let notifTimer = null;

// New DOM refs
let killBtn, modeBadge, stateBadge;
let tPhotoBtn, tDetectBtn, tModeBtn, tSessionBtn;

// Chase DOM refs
let chaseBtn, tChaseBtn, chaseStatusbar, chaseCanvas, chaseCtx;

// Servo gauges
let gaugeSteer = null, gaugePan = null, gaugeTilt = null;

// ── WebSocket ──────────────────────────────────────────────────────────────

function connect() {
    ws = new WebSocket(`ws://${piHost}:8000/ws`);

    ws.onopen = () => {
        connStatus.textContent = 'Connected';
        connStatus.className = 'status-connected';
        if (sessionStatus) { sessionStatus.textContent = 'LOG ●'; sessionStatus.className = 'session-active'; }
        _battWarnShown = false;
    };

    ws.onmessage = (e) => {
        const msg = JSON.parse(e.data);

        if (msg.type === 'sensors') {
            updateSensors(msg.distance, msg.grayscale);
            updateBattery(msg.battery, msg.battery_warn);
        } else if (msg.type === 'rec_state') {
            updateRecState(msg.state);
        } else if (msg.type === 'rec_stopped') {
            const f = msg.filename;
            showNotif(`Video saved · <a href="/download/video/${f}" target="_blank">Download</a>`);
        } else if (msg.type === 'photo_saved') {
            const f = msg.filename;
            showNotif(`Photo saved · <a href="/download/photo/${f}" target="_blank">Download</a>`);
        } else if (msg.type === 'shutdown_ack') {
            sessionBtn.disabled = true;
            if (tSessionBtn) tSessionBtn.disabled = true;
            showNotif('Server shutting down…');
        } else if (msg.type === 'kill_confirmed') {
            const bar = document.getElementById('kill-bar');
            bar.classList.add('kill-confirmed');
            setTimeout(() => bar.classList.remove('kill-confirmed'), 800);
        } else if (msg.type === 'chase_status') {
            updateChaseStatus(msg);
        } else if (msg.type === 'chase_detection') {
            drawTagOverlay(msg);
        }
    };

    ws.onclose = () => {
        connStatus.textContent = 'Disconnected — reconnecting…';
        connStatus.className = 'status-disconnected';
        if (sessionStatus) { sessionStatus.textContent = 'LOG ○'; sessionStatus.className = 'session-inactive'; }
        emergencyStop();
        setTimeout(connect, 2000);
    };

    ws.onerror = () => ws.close();
}

function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(obj));
    }
}

// ── Servo send helpers ─────────────────────────────────────────────────────

function sendSteer(angle) {
    steerAngle = angle;
    send({ cmd: 'steer', angle });
    if (gaugeSteer) gaugeSteer.update(angle);
}

function sendGimbal(axis, angle) {
    send({ cmd: 'gimbal', axis, angle });
    if (axis === 'pan'  && gaugePan)  gaugePan.update(angle);
    if (axis === 'tilt' && gaugeTilt) gaugeTilt.update(angle);
}

// ── Gauge factory ──────────────────────────────────────────────────────────

function makeGauge(svgId, min, max, color) {
    const svg = document.getElementById(svgId);
    if (!svg) return { update: () => {} };
    const NS = 'http://www.w3.org/2000/svg';
    const cx = 60, cy = 70, r = 52, rNeedle = 44;

    function el(tag, attrs, text) {
        const e = document.createElementNS(NS, tag);
        for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v));
        if (text !== undefined) e.textContent = text;
        return e;
    }

    const toRad = d => d * Math.PI / 180;

    // Background arc (full semicircle)
    svg.appendChild(el('path', {
        d: `M 8,70 A ${r},${r} 0 0,1 112,70`,
        fill: 'none', stroke: '#22263a', 'stroke-width': 8, 'stroke-linecap': 'round'
    }));

    // Zero tick — radial mark at the 0° position
    const zf = (0 - min) / (max - min);
    const za = 180 - zf * 180; // screen angle for zero
    const zr = toRad(za);
    svg.appendChild(el('line', {
        x1: (cx + 42 * Math.cos(zr)).toFixed(2), y1: (cy - 42 * Math.sin(zr)).toFixed(2),
        x2: (cx + r  * Math.cos(zr)).toFixed(2), y2: (cy - r  * Math.sin(zr)).toFixed(2),
        stroke: '#8892a4', 'stroke-width': 2
    }));

    // Value arc (updated on each call)
    const varc = el('path', {
        fill: 'none', stroke: color, 'stroke-width': 8, 'stroke-linecap': 'round', d: ''
    });
    svg.appendChild(varc);

    // Needle
    const needle = el('line', {
        x1: cx, y1: cy, x2: cx, y2: cy - rNeedle,
        stroke: '#e2e8f0', 'stroke-width': 2, 'stroke-linecap': 'round'
    });
    svg.appendChild(needle);

    // Center dot
    svg.appendChild(el('circle', { cx, cy, r: 4, fill: '#e2e8f0' }));

    // Digital readout
    const readout = el('text', {
        x: cx, y: 50, 'text-anchor': 'middle', 'dominant-baseline': 'middle',
        fill: '#e2e8f0', 'font-size': 13, 'font-weight': 700
    }, '0°');
    svg.appendChild(readout);

    // Min / max labels
    svg.appendChild(el('text', { x: 6,   y: 74, 'text-anchor': 'middle',
        fill: '#8892a4', 'font-size': 7 }, min + '°'));
    svg.appendChild(el('text', { x: 114, y: 74, 'text-anchor': 'middle',
        fill: '#8892a4', 'font-size': 7 }, '+' + max + '°'));

    const zeroX = cx + r * Math.cos(zr);
    const zeroY = cy - r * Math.sin(zr);

    function update(val) {
        const frac = Math.max(0, Math.min(1, (val - min) / (max - min)));
        const aDeg = 180 - frac * 180;
        const aRad = toRad(aDeg);

        // Needle
        needle.setAttribute('x2', (cx + rNeedle * Math.cos(aRad)).toFixed(2));
        needle.setAttribute('y2', (cy - rNeedle * Math.sin(aRad)).toFixed(2));

        // Value arc from zero tick to current angle
        const curX = cx + r * Math.cos(aRad);
        const curY = cy - r * Math.sin(aRad);
        const span = Math.abs(za - aDeg);
        if (span < 0.5) {
            varc.setAttribute('d', '');
        } else {
            // Clockwise (sweep=1) when val>0 (aDeg moved right of zero, i.e. aDeg < za)
            const sweep = aDeg < za ? 1 : 0;
            varc.setAttribute('d',
                `M ${zeroX.toFixed(2)},${zeroY.toFixed(2)} A ${r},${r} 0 0,${sweep} ${curX.toFixed(2)},${curY.toFixed(2)}`
            );
        }

        readout.textContent = (val > 0 ? '+' : '') + val + '°';
    }

    update(0);
    return { update };
}

// ── Drive (WASD) ───────────────────────────────────────────────────────────

function getSpeed() {
    return parseInt(speedSlider.value, 10);
}

function sendDriveState() {
    const fwd   = heldKeys.has('w');
    const bwd   = heldKeys.has('s');
    const left  = heldKeys.has('a');
    const right = heldKeys.has('d');

    const angle = (left && !right) ? -30 : (right && !left) ? 30 : 0;
    sendSteer(angle);

    if (fwd && !bwd) {
        send({ cmd: 'drive', direction: 'forward', speed: getSpeed() });
        updateMotorDisplay(getSpeed(), 'fwd');
    } else if (bwd && !fwd) {
        send({ cmd: 'drive', direction: 'backward', speed: getSpeed() });
        updateMotorDisplay(getSpeed(), 'bwd');
    } else {
        send({ cmd: 'drive', direction: 'stop', speed: 0 });
        updateMotorDisplay(0, 'stop');
    }
}

function emergencyStop() {
    for (const k of Object.keys(driveIntervals)) {
        clearInterval(driveIntervals[k]);
        delete driveIntervals[k];
    }
    for (const k of Object.keys(gimbalIntervals)) {
        clearInterval(gimbalIntervals[k]);
        delete gimbalIntervals[k];
    }
    heldKeys.clear();
    updateKeyDisplay();
    updateMotorDisplay(0, 'stop');
    send({ cmd: 'drive', direction: 'stop', speed: 0 });
    sendSteer(0);
}

// ── Kill ───────────────────────────────────────────────────────────────────

function doKill() {
    send({ cmd: 'kill' });
    emergencyStop();
}

// ── Gimbal (IJKL) ──────────────────────────────────────────────────────────

function startGimbal(key) {
    if (gimbalIntervals[key]) return;

    const step = () => {
        let axis, delta;
        if      (key === 'j') { axis = 'pan';  delta = -GIMBAL_STEP; }
        else if (key === 'l') { axis = 'pan';  delta =  GIMBAL_STEP; }
        else if (key === 'i') { axis = 'tilt'; delta =  GIMBAL_STEP; }
        else if (key === 'k') { axis = 'tilt'; delta = -GIMBAL_STEP; }
        else return;

        const limits = axis === 'pan' ? [-90, 90] : [-35, 65];
        gimbalAngles[axis] = Math.max(limits[0], Math.min(limits[1], gimbalAngles[axis] + delta));
        sendGimbal(axis, gimbalAngles[axis]);
    };

    step();
    gimbalIntervals[key] = setInterval(step, GIMBAL_MS);
}

function stopGimbal(key) {
    clearInterval(gimbalIntervals[key]);
    delete gimbalIntervals[key];
}

function recenterCamera() {
    gimbalAngles.pan = 0;
    gimbalAngles.tilt = 0;
    sendGimbal('pan', 0);
    sendGimbal('tilt', 0);
}

// ── Mode / detect ──────────────────────────────────────────────────────────

function toggleMode() {
    modeState = modeState === 'gentle' ? 'aggressive' : 'gentle';
    send({ cmd: 'mode', value: modeState });
    updateModeBadge();
    if (tModeBtn) tModeBtn.textContent = modeState === 'gentle' ? 'Gentle' : 'Aggressive';
}

function toggleDetect() {
    detectOn = !detectOn;
    send({ cmd: 'detect', value: detectOn });
    if (tDetectBtn) {
        tDetectBtn.textContent = detectOn ? 'Detect ON' : 'Detect OFF';
        tDetectBtn.classList.toggle('btn-detect-active', detectOn);
    }
}

function toggleChase() {
    const action = chaseActive ? 'stop' : 'start';
    send({ cmd: 'tag_chase', action, speed: getSpeed() });
}

function updateChaseStatus(msg) {
    chaseActive = msg.active;

    const on = msg.active;
    if (chaseCanvas) chaseCanvas.style.display = on ? 'block' : 'none';
    if (!on && chaseCtx) chaseCtx.clearRect(0, 0, chaseCanvas.width, chaseCanvas.height);

    const label = on ? 'Tag Chase ON' : 'Tag Chase OFF';
    if (chaseBtn)   { chaseBtn.textContent = label;   chaseBtn.classList.toggle('btn-chase-active', on); }
    if (tChaseBtn)  { tChaseBtn.textContent = on ? 'Chase ON' : 'Chase';
                      tChaseBtn.classList.toggle('btn-chase-active', on); }

    if (!chaseStatusbar) return;
    if (!on) {
        chaseStatusbar.classList.remove('chase-bar-visible');
        chaseStatusbar.textContent = '';
        return;
    }
    chaseStatusbar.classList.add('chase-bar-visible');
    const state = msg.state || 'idle';
    let text = '';
    if (state === 'starting') {
        text = `Starting… ${msg.countdown}`;
    } else if (state === 'chasing') {
        text = msg.distance_cm != null ? `Chasing — ${msg.distance_cm} cm` : 'Chasing';
    } else if (state === 'stopping') {
        text = msg.distance_cm != null ? `Stopping — ${msg.distance_cm} cm` : 'Stopping';
    } else if (state === 'searching') {
        text = 'Searching…';
    } else {
        text = state.charAt(0).toUpperCase() + state.slice(1);
    }
    chaseStatusbar.textContent = text;
}

function drawTagOverlay(msg) {
    if (!chaseCanvas || !chaseCtx || !chaseActive) return;
    const camEl = document.getElementById('cam-feed');
    if (!camEl) return;
    const rect = camEl.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;

    chaseCanvas.width  = rect.width;
    chaseCanvas.height = rect.height;
    chaseCtx.clearRect(0, 0, rect.width, rect.height);

    if (!msg.found) return;

    // Map frame coords to canvas coords accounting for object-fit:contain letterboxing
    const fW = msg.frame_w, fH = msg.frame_h;
    const imgAspect = fW / fH;
    const boxAspect = rect.width / rect.height;
    let rW, rH, oX, oY;
    if (imgAspect > boxAspect) {
        rW = rect.width;
        rH = rect.width / imgAspect;
        oX = 0;
        oY = (rect.height - rH) / 2;
    } else {
        rH = rect.height;
        rW = rect.height * imgAspect;
        oX = (rect.width - rW) / 2;
        oY = 0;
    }
    const sX = rW / fW, sY = rH / fH;

    const corners = msg.corners;
    chaseCtx.strokeStyle = '#f59e0b';
    chaseCtx.lineWidth = 2.5;
    chaseCtx.beginPath();
    chaseCtx.moveTo(oX + corners[0][0] * sX, oY + corners[0][1] * sY);
    for (let i = 1; i < corners.length; i++) {
        chaseCtx.lineTo(oX + corners[i][0] * sX, oY + corners[i][1] * sY);
    }
    chaseCtx.closePath();
    chaseCtx.stroke();

    const cx = oX + msg.center[0] * sX;
    const cy = oY + msg.center[1] * sY;
    chaseCtx.fillStyle = '#f59e0b';
    chaseCtx.beginPath();
    chaseCtx.arc(cx, cy, 5, 0, Math.PI * 2);
    chaseCtx.fill();
}

function updateModeBadge() {
    if (!modeBadge) return;
    modeBadge.textContent = modeState.toUpperCase();
    modeBadge.className = 'mode-badge ' + (modeState === 'aggressive' ? 'mode-aggressive' : 'mode-gentle');
}

// ── Virtual joysticks ──────────────────────────────────────────────────────

const JOYSTICK_R = 46;   // max thumb travel px (base 140px r=70, thumb 48px r=24 → 70-24=46)
const DEAD_ZONE  = 0.15; // normalised fraction below which input is zeroed

function setupJoysticks() {
    // ── Drive joystick (left) ──────────────────────────
    const driveBase  = document.getElementById('joy-drive');
    const driveThumb = document.getElementById('joy-drive-thumb');
    if (!driveBase) return;

    let driveActive = false;
    let driveCx = 0, driveCy = 0;

    driveBase.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        driveBase.setPointerCapture(e.pointerId);
        const r = driveBase.getBoundingClientRect();
        driveCx = r.left + r.width  / 2;
        driveCy = r.top  + r.height / 2;
        driveActive = true;
        onDriveMove(e.clientX - driveCx, e.clientY - driveCy);
    });
    driveBase.addEventListener('pointermove', (e) => {
        if (!driveActive) return;
        onDriveMove(e.clientX - driveCx, e.clientY - driveCy);
    });
    const driveRelease = () => {
        if (!driveActive) return;
        driveActive = false;
        driveThumb.style.transform = 'translate(-50%, -50%)';
        send({ cmd: 'drive', direction: 'stop', speed: 0 });
        sendSteer(0);
        updateMotorDisplay(0, 'stop');
    };
    driveBase.addEventListener('pointerup',     driveRelease);
    driveBase.addEventListener('pointercancel', driveRelease);

    function onDriveMove(rawDx, rawDy) {
        const dist    = Math.hypot(rawDx, rawDy);
        const clamped = Math.min(dist, JOYSTICK_R);
        const angle   = Math.atan2(rawDy, rawDx);
        const dx = Math.cos(angle) * clamped;
        const dy = Math.sin(angle) * clamped;
        driveThumb.style.transform =
            `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
        const nx = dx / JOYSTICK_R;          // -1..1, positive = right = steer right
        const ny = dy / JOYSTICK_R;          // -1..1, positive = screen-down = backward
        const forwardNorm = -ny;              // positive = forward
        sendSteer(Math.round(nx * 30));
        if (Math.abs(forwardNorm) < DEAD_ZONE) {
            send({ cmd: 'drive', direction: 'stop', speed: 0 });
            updateMotorDisplay(0, 'stop');
        } else if (forwardNorm > 0) {
            const spd = Math.round(forwardNorm * 100);
            send({ cmd: 'drive', direction: 'forward', speed: spd });
            updateMotorDisplay(spd, 'fwd');
        } else {
            const spd = Math.round(-forwardNorm * 100);
            send({ cmd: 'drive', direction: 'backward', speed: spd });
            updateMotorDisplay(spd, 'bwd');
        }
    }

    // ── Camera joystick (right) ────────────────────────
    const camBase  = document.getElementById('joy-cam');
    const camThumb = document.getElementById('joy-cam-thumb');
    if (!camBase) return;

    let camActive = false;
    let camCx = 0, camCy = 0;
    let camNx = 0, camNy = 0;
    let camInterval = null;

    camBase.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        camBase.setPointerCapture(e.pointerId);
        const r = camBase.getBoundingClientRect();
        camCx = r.left + r.width  / 2;
        camCy = r.top  + r.height / 2;
        camActive = true;
        camNx = 0; camNy = 0;
        camInterval = setInterval(stepCam, GIMBAL_MS);
        onCamMove(e.clientX - camCx, e.clientY - camCy);
    });
    camBase.addEventListener('pointermove', (e) => {
        if (!camActive) return;
        onCamMove(e.clientX - camCx, e.clientY - camCy);
    });
    const camRelease = () => {
        if (!camActive) return;
        camActive = false;
        clearInterval(camInterval);
        camInterval = null;
        camNx = 0; camNy = 0;
        camThumb.style.transform = 'translate(-50%, -50%)';
    };
    camBase.addEventListener('pointerup',     camRelease);
    camBase.addEventListener('pointercancel', camRelease);

    function onCamMove(rawDx, rawDy) {
        const dist    = Math.hypot(rawDx, rawDy);
        const clamped = Math.min(dist, JOYSTICK_R);
        const angle   = Math.atan2(rawDy, rawDx);
        const dx = Math.cos(angle) * clamped;
        const dy = Math.sin(angle) * clamped;
        camThumb.style.transform =
            `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
        camNx = dx / JOYSTICK_R;
        camNy = dy / JOYSTICK_R;
    }

    function stepCam() {
        if (Math.abs(camNx) > DEAD_ZONE) {
            const delta = Math.round(camNx * GIMBAL_STEP * 2);
            gimbalAngles.pan = Math.max(-90, Math.min(90, gimbalAngles.pan + delta));
            sendGimbal('pan', gimbalAngles.pan);
        }
        if (Math.abs(camNy) > DEAD_ZONE) {
            // positive screen-Y = thumb pushed down = tilt down (negative delta)
            const delta = Math.round(camNy * GIMBAL_STEP * 2);
            gimbalAngles.tilt = Math.max(-35, Math.min(65, gimbalAngles.tilt - delta));
            sendGimbal('tilt', gimbalAngles.tilt);
        }
    }
}

// ── Keyboard events ────────────────────────────────────────────────────────

const DRIVE_KEYS  = new Set(['w', 'a', 's', 'd']);
const GIMBAL_KEYS = new Set(['i', 'j', 'k', 'l']);

document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    const key = e.key.toLowerCase();
    if (e.repeat) return;

    if (DRIVE_KEYS.has(key)) {
        heldKeys.add(key);
        updateKeyDisplay();
        sendDriveState();
        if (!driveIntervals[key]) {
            driveIntervals[key] = setInterval(sendDriveState, DRIVE_REPEAT_MS);
        }
    } else if (GIMBAL_KEYS.has(key)) {
        heldKeys.add(key);
        updateKeyDisplay();
        startGimbal(key);
    } else if (key === 'r') {
        recenterCamera();
    } else if (key === 'm') {
        toggleMode();
    } else if (key === 'e') {
        toggleDetect();
    } else if (key === 'enter') {
        if (document.activeElement && document.activeElement.tagName === 'BUTTON') return;
        send({ cmd: 'shutdown' });
    } else if (key === ' ') {
        e.preventDefault();
        doKill();
    }
});

document.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();

    if (DRIVE_KEYS.has(key)) {
        clearInterval(driveIntervals[key]);
        delete driveIntervals[key];
        heldKeys.delete(key);
        updateKeyDisplay();
        sendDriveState();
    } else if (GIMBAL_KEYS.has(key)) {
        heldKeys.delete(key);
        updateKeyDisplay();
        stopGimbal(key);
    }
});

window.addEventListener('blur', emergencyStop);

// ── UI: motor display ──────────────────────────────────────────────────────

function updateMotorDisplay(pct, dir) {
    const w = pct + '%';
    motorLBar.style.width = w;
    motorRBar.style.width = w;
    motorLPct.textContent = pct + '%';
    motorRPct.textContent = pct + '%';
    const arrow = dir === 'fwd' ? '→' : dir === 'bwd' ? '←' : '';
    motorLDir.textContent = arrow;
    motorRDir.textContent = arrow;
    const active = pct > 0;
    motorLBar.className = 'bar-fill ' + (active ? 'bar-motor-active' : 'bar-motor');
    motorRBar.className = 'bar-fill ' + (active ? 'bar-motor-active' : 'bar-motor');
}

// ── UI: key display ────────────────────────────────────────────────────────

function updateKeyDisplay() {
    for (const [k, el] of Object.entries(keyEls)) {
        el.classList.toggle('key-active', heldKeys.has(k));
    }
}

// ── UI: sensors ────────────────────────────────────────────────────────────

function flashEl(el) {
    el.classList.remove('value-flash');
    void el.offsetWidth; // force reflow to restart animation
    el.classList.add('value-flash');
}

function updateSensors(distance, grayscale) {
    // Ultrasonic — update signal buffer
    distBuf.push(distance);
    if (distBuf.length > DIST_BUF_SIZE) distBuf.shift();
    const hasSignal = distBuf.some(v => v > 0);
    usSignal.textContent = hasSignal ? '●' : '○';
    usSignal.className = 'signal-badge ' + (hasSignal ? 'signal-yes' : 'signal-no');

    flashEl(distanceVal);
    if (distance <= 0) {
        distanceVal.textContent = 'Out of range';
        distanceVal.className = 'sensor-value out-of-range value-flash';
        distanceBar.style.width = '0%';
        distanceBar.className = 'bar-fill bar-ok';
    } else {
        distanceVal.textContent = distance.toFixed(1) + ' cm';
        const cls = distance < 15 ? 'sensor-value dist-danger' :
                    distance < 35 ? 'sensor-value dist-warn'   : 'sensor-value dist-ok';
        distanceVal.className = cls + ' value-flash';
        const pct = Math.min(100, (distance / 100) * 100);
        distanceBar.style.width = pct + '%';
        distanceBar.className = 'bar-fill ' + (
            distance < 15 ? 'bar-danger' :
            distance < 35 ? 'bar-warn'   : 'bar-ok'
        );
    }

    // Grayscale (0–4096)
    grayscale.forEach((val, i) => {
        gsVals[i].textContent = val;
        gsBars[i].style.width = ((val / 4096) * 100).toFixed(1) + '%';
    });
}

function updateBattery(v, warn) {
    if (v === null || v === undefined) return;
    batteryVal.textContent = v.toFixed(2) + ' V';
    // 2S LiPo: 6.0 V empty → 8.4 V full
    const pct = Math.max(0, Math.min(100, ((v - 6.0) / (8.4 - 6.0)) * 100));
    batteryBar.style.width = pct + '%';
    batteryVal.className = 'sensor-value ' + (
        v < 6.5 ? 'dist-danger' :
        v < 7.0 ? 'dist-warn'   : 'dist-ok'
    );
    batteryBar.className = 'bar-fill ' + (
        v < 6.5 ? 'bar-danger' :
        v < 7.0 ? 'bar-warn'   : 'bar-ok'
    );
    if (warn && !_battWarnShown) {
        _battWarnShown = true;
        showNotif('Low battery: ' + v.toFixed(2) + ' V — charge soon');
    }
}

// ── UI: recording ──────────────────────────────────────────────────────────

function updateRecState(state) {
    const active = state === 'recording';
    recBtn.textContent = active ? 'Stop Recording' : 'Record Video';
    recBtn.classList.toggle('btn-recording', active);
    recStatus.textContent = active ? 'REC ●' : 'REC ○';
    recStatus.className   = active ? 'rec-active' : 'rec-inactive';
}

// ── UI: notifications ──────────────────────────────────────────────────────

function showNotif(html) {
    notifEl.innerHTML = html;
    notifEl.style.opacity = '1';
    clearTimeout(notifTimer);
    notifTimer = setTimeout(() => { notifEl.style.opacity = '0'; }, 6000);
}

// ── Init ───────────────────────────────────────────────────────────────────

window.addEventListener('DOMContentLoaded', () => {
    speedSlider  = document.getElementById('speed-slider');
    speedValue   = document.getElementById('speed-value');
    photoBtn     = document.getElementById('photo-btn');
    recBtn       = document.getElementById('rec-btn');
    sessionBtn   = document.getElementById('session-btn');
    connStatus   = document.getElementById('conn-status');
    recStatus    = document.getElementById('rec-status');
    sessionStatus = document.getElementById('session-status');  // LOG ●/○ badge
    notifEl      = document.getElementById('notif');

    distanceVal  = document.getElementById('distance-val');
    distanceBar  = document.getElementById('distance-bar');
    usSignal     = document.getElementById('us-signal');
    batteryVal   = document.getElementById('battery-val');
    batteryBar   = document.getElementById('battery-bar');
    motorLBar    = document.getElementById('motor-l-bar');
    motorRBar    = document.getElementById('motor-r-bar');
    motorLPct    = document.getElementById('motor-l-pct');
    motorRPct    = document.getElementById('motor-r-pct');
    motorLDir    = document.getElementById('motor-l-dir');
    motorRDir    = document.getElementById('motor-r-dir');
    gsVals = ['gs-l-val', 'gs-m-val', 'gs-r-val'].map(id => document.getElementById(id));
    gsBars = ['gs-l-bar', 'gs-m-bar', 'gs-r-bar'].map(id => document.getElementById(id));

    keyEls = {};
    for (const k of ['w', 'a', 's', 'd', 'i', 'j', 'k', 'l']) {
        keyEls[k] = document.getElementById(`key-${k}`);
    }

    killBtn     = document.getElementById('kill-btn');
    modeBadge   = document.getElementById('mode-badge');
    stateBadge  = document.getElementById('state-badge');
    tPhotoBtn   = document.getElementById('t-photo-btn');
    tDetectBtn  = document.getElementById('t-detect-btn');
    tModeBtn    = document.getElementById('t-mode-btn');
    tSessionBtn = document.getElementById('t-session-btn');

    chaseBtn       = document.getElementById('chase-btn');
    tChaseBtn      = document.getElementById('t-chase-btn');
    chaseStatusbar = document.getElementById('chase-statusbar');
    chaseCanvas    = document.getElementById('chase-overlay');
    chaseCtx       = chaseCanvas ? chaseCanvas.getContext('2d') : null;

    document.getElementById('cam-feed').src = VIDEO_SRC;

    speedSlider.addEventListener('input', () => {
        speedValue.textContent = speedSlider.value + '%';
        if (heldKeys.has('w') || heldKeys.has('s')) sendDriveState();
    });

    photoBtn.addEventListener('click',   () => send({ cmd: 'photo' }));
    recBtn.addEventListener('click',     () => send({ cmd: 'rec_toggle' }));
    sessionBtn.addEventListener('click', () => {
        send({ cmd: 'shutdown' });
        sessionBtn.disabled = true;
        if (tSessionBtn) tSessionBtn.disabled = true;
    });

    if (killBtn)     killBtn.addEventListener('click', doKill);
    if (tPhotoBtn)   tPhotoBtn.addEventListener('click', () => send({ cmd: 'photo' }));
    if (tDetectBtn)  tDetectBtn.addEventListener('click', toggleDetect);
    if (tModeBtn)    tModeBtn.addEventListener('click', toggleMode);
    if (chaseBtn)    chaseBtn.addEventListener('click', toggleChase);
    if (tChaseBtn)   tChaseBtn.addEventListener('click', toggleChase);
    if (tSessionBtn) tSessionBtn.addEventListener('click', () => {
        send({ cmd: 'shutdown' });
        sessionBtn.disabled = true;
        if (tSessionBtn) tSessionBtn.disabled = true;
    });

    gaugeSteer = makeGauge('gauge-steer', -30,  30, '#4f8ef7');
    gaugePan   = makeGauge('gauge-pan',   -90,  90, '#a78bfa');
    gaugeTilt  = makeGauge('gauge-tilt',  -35,  65, '#22c55e');

    setupJoysticks();

    connect();
    setInterval(() => send({ cmd: 'heartbeat' }), 1000);
});
