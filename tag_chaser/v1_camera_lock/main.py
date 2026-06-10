"""
main.py -- Standalone tag chaser runner (no dashboard required).

Runs directly on the Pi. Owns Picamera2 and runs the capture loop itself.
Ctrl+C to stop cleanly.

Usage:
    python3 main.py [speed]
    speed: optional drive speed 0-100 (default 30)
"""

import os
import signal
import sys
import time

import cv2
import yaml

_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from picarx import Picarx
from tag_chaser.v1_camera_lock.chaser import TagChaser

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'config.yaml')


def main():
    speed = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    speed = max(0, min(100, speed))

    with open(CONFIG_FILE) as f:
        config = yaml.safe_load(f)

    cam_cfg = config['camera']
    cam_w   = int(cam_cfg.get('width', 640))
    cam_h   = int(cam_cfg.get('height', 480))

    px     = Picarx()
    chaser = TagChaser(px, config, broadcast_fn=None)

    from picamera2 import Picamera2
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={'format': 'BGR888', 'size': (cam_w, cam_h)}
    ))
    picam2.start()
    time.sleep(0.5)

    def shutdown(sig, frame):
        print('\nStopping...')
        chaser.stop()
        picam2.stop()
        try:
            px.reset()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f'Tag chaser v1 — standalone | speed={speed} | Ctrl+C to stop')
    chaser.start(speed=speed)

    while True:
        try:
            raw   = picam2.capture_array()
            frame = raw  # BGR888 — no conversion needed
            chaser.process_frame(frame)
        except Exception as e:
            print(f"Frame error: {e}")
            time.sleep(0.1)


if __name__ == '__main__':
    main()
