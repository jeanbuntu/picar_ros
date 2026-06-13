"""
tf_publisher.py -- Serializes dual-tag detections to WebSocket + raw cycle JSON files.

Called from chaser.py on each frame during active chase. Owns per-cycle raw JSON
file writing on the Pi side. The broadcast goes over the existing dashboard WebSocket
so tf_bridge.py on Ubuntu can receive tag_detections messages.
"""

import json
import logging
import os
import time

_logger = logging.getLogger("marker_detector")


class TfPublisher:
    def __init__(self, session_dir: str, broadcast_fn):
        self._session_dir = session_dir
        self._broadcast = broadcast_fn
        self._cycle_file = None
        self._cycle_file_first = True
        self._cycle_n = -1

    def open_cycle(self, cycle_n: int) -> None:
        self.close_cycle()
        self._cycle_n = cycle_n
        ts = time.strftime("%H%M%S")
        fname = f"cycle_{cycle_n}_raw_{ts}.json"
        path = os.path.join(self._session_dir, fname)
        self._cycle_file = open(path, 'w')
        self._cycle_file.write('[\n')
        self._cycle_file_first = True
        _logger.info("cycle_file_open cycle=%d file=%s", cycle_n, fname)

    def close_cycle(self) -> None:
        if self._cycle_file is None:
            return
        self._cycle_file.write('\n]\n')
        self._cycle_file.flush()
        self._cycle_file.close()
        self._cycle_file = None
        _logger.info("cycle_file_close cycle=%d", self._cycle_n)

    def on_frame(self, ts: float, cycle: int, detected_tags: list, bcast_due: bool) -> None:
        """
        detected_tags: list of pupil_apriltags Detection objects (already confidence-filtered).
        Appends raw record to the open cycle JSON file unconditionally.
        Broadcasts tag_detections WebSocket message when bcast_due is True.
        """
        if not detected_tags:
            return

        tag_records = []
        for det in detected_tags:
            if det.pose_t is None or det.pose_R is None:
                continue
            tag_records.append({
                'id': det.tag_id,
                'confidence': round(float(det.decision_margin), 3),
                'pose_t': det.pose_t.flatten().tolist(),
                'pose_R': det.pose_R.tolist(),
            })

        if not tag_records:
            return

        # Append raw record to cycle JSON file (every frame, no throttle)
        if self._cycle_file is not None:
            record = {'ts': round(ts, 6), 'tags': tag_records}
            if not self._cycle_file_first:
                self._cycle_file.write(',\n')
            self._cycle_file_first = False
            self._cycle_file.write(json.dumps(record))

        # Broadcast to WebSocket clients at 10 fps
        if bcast_due:
            msg = {
                'type': 'tag_detections',
                'ts': round(ts, 6),
                'cycle': cycle,
                'tags': tag_records,
            }
            try:
                self._broadcast(msg)
            except Exception as e:
                _logger.debug("tf_publisher broadcast error: %s", e)
