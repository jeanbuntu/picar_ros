"""
main.py -- Standalone tag chaser v2 runner (no dashboard required).

Runs directly on the Pi. Owns Picamera2 and runs the capture loop itself.
Creates a session folder for logs and raw cycle JSON files.
Ctrl+C to stop cleanly.

Usage:
    python3 main.py [speed]
    speed: optional drive speed 0-100 (default 30)
"""

import logging
import os
import signal
import sys
import time
from datetime import datetime

import yaml

_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from picarx import Picarx
from tag_chaser.v2_world_tf.chaser import TagChaser

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'config.yaml')
LOG_DIR     = os.path.join(os.path.expanduser('~'), 'picar_ros', 'logs')


def main():
    speed = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    speed = max(0, min(100, speed))

    log_ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(LOG_DIR, f"session_{log_ts}")
    os.makedirs(session_dir, exist_ok=True)

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s.%(msecs)03d [%(levelname)-5s] [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(os.path.join(session_dir, "master.log")),
        ],
    )

    with open(CONFIG_FILE) as f:
        config = yaml.safe_load(f)

    cam_cfg = config['camera']
    cam_w   = int(cam_cfg.get('width', 640))
    cam_h   = int(cam_cfg.get('height', 480))

    px     = Picarx()
    chaser = TagChaser(px, config, broadcast_fn=None, session_dir=session_dir)

    from picamera2 import Picamera2
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={'format': 'RGB888', 'size': (cam_w, cam_h)}
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

    print(f'Tag chaser v2 — standalone | speed={speed} | Ctrl+C to stop')
    chaser.start(speed=speed)

    while True:
        try:
            raw = picam2.capture_array()
            chaser.process_frame(raw)
        except Exception as e:
            logging.getLogger("picarx").error("frame error: %s", e)
            time.sleep(0.1)


if __name__ == '__main__':
    main()
