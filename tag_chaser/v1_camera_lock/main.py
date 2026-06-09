"""
main.py -- Standalone tag chaser runner (no dashboard required).

Runs directly on the Pi. Does not use Vilib (no web stream).
Ctrl+C to stop cleanly.

Usage:
    python3 main.py [speed]
    speed: optional drive speed 0-100 (default 30)
"""

import os
import signal
import sys
import time

import yaml

# Resolve picar_ros root so we can import tag_chaser as a package
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

    px      = Picarx()
    chaser  = TagChaser(px, config, broadcast_fn=None)

    def shutdown(sig, frame):
        print('\nStopping...')
        chaser.stop()
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
        time.sleep(1.0)


if __name__ == '__main__':
    main()
