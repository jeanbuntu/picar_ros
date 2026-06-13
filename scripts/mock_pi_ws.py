#!/usr/bin/env python3
"""
mock_pi_ws.py -- Synthetic tag_detections WebSocket server for testing tf_bridge.py
without physical Pi hardware.

Streams scripted AprilTag pose data at 10 fps through five phases that exercise:
  - Normal chasing (both tags visible)
  - Tag 0 disappears mid-chase
  - World tag (Tag 1) lost → TF gap
  - World tag reacquired
  - New cycle (different color in RViz)

Usage:
    python3 scripts/mock_pi_ws.py [--port 8000]

Then in separate terminals:
    # ROS2 node pointed at mock server
    ./scripts/launch_tf_bridge.sh --ros-args -p pi_ws_url:=ws://localhost:8000/ws

    # RViz2
    rviz2 -d rviz/picar_trajectory.rviz

Pose convention (same as tf_bridge.py expects):
    pose_t = [x, y, z]  -- tag position in camera frame (meters)
    pose_R = 3x3 list   -- rotation matrix of tag in camera frame
    identity rotation = tag facing camera flat, no tilt

TF math preview (what tf_bridge.py will compute):
    T_world_camera = inv(T_camera_tag1)
    camera_world_pos ≈ -pose_t of tag1  (with identity rotation)
    Animating tag1.pose_t.x → car trajectory sweeps horizontally in world frame
"""

import argparse
import asyncio
import json
import math
import time

try:
    import websockets
except ImportError:
    raise SystemExit("Install websockets: pip3 install websockets")

RATE_HZ = 10
FRAME_S = 1.0 / RATE_HZ


def _R_id():
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def _tag(tag_id, x, y, z, conf=50.0):
    return {'id': tag_id, 'confidence': conf, 'pose_t': [x, y, z], 'pose_R': _R_id()}


def _scenario():
    """
    Generator of (cycle: int, tags: list).
    Empty tags list → no message sent (simulates world tag out of frame).
    Loops indefinitely.
    """
    while True:
        # ── Phase 0 (15 s): cycle 0, both tags, tag0 sweeps left/right ────────
        _phase("Phase 0 [15s]: cycle=0 — both tags, tag0 sweeps L/R")
        for i in range(int(15 * RATE_HZ)):
            x_car  = math.sin(i / 15.0) * 0.25     # camera drifts; negate for world pos
            x_tag0 = math.sin(i / 5.0)  * 0.12     # tag0 oscillates independently
            yield (0, [_tag(1, -x_car, 0.0, 0.50),
                       _tag(0,  x_tag0, 0.0, 0.30, conf=45.0)])

        # ── Phase 1 (5 s): cycle 0, tag0 gone, world still visible ───────────
        _phase("Phase 1 [5s]: cycle=0 — tag0 gone, world anchor still visible")
        for i in range(int(5 * RATE_HZ)):
            x_car = math.sin((int(15 * RATE_HZ) + i) / 15.0) * 0.25
            yield (0, [_tag(1, -x_car, 0.0, 0.50)])

        # ── Phase 2 (5 s): cycle 0, world lost (neither tag) ─────────────────
        _phase("Phase 2 [5s]: cycle=0 — WORLD LOST, gap opens in RViz")
        for _ in range(int(5 * RATE_HZ)):
            yield (0, [])        # empty → tf_bridge receives no message this frame

        # ── Phase 3 (10 s): cycle 0, both tags resume, arc motion ─────────────
        _phase("Phase 3 [10s]: cycle=0 — world reacquired, arc motion resumes")
        for i in range(int(10 * RATE_HZ)):
            a = i * 0.020
            x_car  =  math.sin(a) * 0.30
            z_tag1  =  0.48 + math.cos(a) * 0.04
            x_tag0  = -x_car * 0.35
            yield (0, [_tag(1, -x_car, 0.0, z_tag1),
                       _tag(0,  x_tag0, 0.0, 0.28, conf=43.0)])

        # ── Phase 4 (15 s): cycle 1, new cycle → red trajectory in RViz ───────
        _phase("Phase 4 [15s]: cycle=1 — NEW CYCLE (red in RViz), sweeping arc")
        for i in range(int(15 * RATE_HZ)):
            a = i * 0.025
            x_car  =  math.cos(a) * 0.35
            z_tag1  =  0.50 + math.sin(a * 0.6) * 0.06
            x_tag0  =  math.sin(a * 1.4) * 0.14
            yield (1, [_tag(1, -x_car, 0.01, z_tag1),
                       _tag(0,  x_tag0, -0.01, 0.25, conf=44.0)])

        print("[mock] scenario complete — looping from Phase 0")


_last_phase = [None]


def _phase(label: str):
    if _last_phase[0] != label:
        _last_phase[0] = label
        print(f"[mock] {label}")


async def _drain(websocket, queue: asyncio.Queue):
    """Send queued messages to one client until it disconnects."""
    try:
        while True:
            msg = await queue.get()
            await websocket.send(json.dumps(msg))
    except (websockets.exceptions.ConnectionClosed, Exception):
        pass


async def _broadcast_loop(clients: set):
    for cycle, tags in _scenario():
        if not tags:
            await asyncio.sleep(FRAME_S)
            continue
        msg = {
            'type': 'tag_detections',
            'ts':    round(time.time(), 6),
            'cycle': cycle,
            'tags':  tags,
        }
        for q in list(clients):
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                pass
        await asyncio.sleep(FRAME_S)


async def main(port: int):
    clients: set = set()

    async def handler(websocket, path=None):
        q = asyncio.Queue(maxsize=30)
        clients.add(q)
        addr = getattr(websocket, 'remote_address', '?')
        print(f"[mock] client connected from {addr}")
        try:
            await _drain(websocket, q)
        finally:
            clients.discard(q)
            print(f"[mock] client disconnected from {addr}")

    print(f"[mock] listening on ws://localhost:{port}  (connects to any path)")
    print(f"[mock] point tf_bridge at:  --ros-args -p pi_ws_url:=ws://localhost:{port}/ws")

    async with websockets.serve(handler, 'localhost', port, ping_interval=None):
        await _broadcast_loop(clients)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Mock Pi WebSocket server for tf_bridge testing')
    ap.add_argument('--port', type=int, default=8000, help='Port to listen on (default 8000)')
    args = ap.parse_args()
    try:
        asyncio.run(main(args.port))
    except KeyboardInterrupt:
        print('\n[mock] stopped')
