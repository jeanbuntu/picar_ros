"""
live.py — qualitative real-time cat detection viewer.

Usage:
    python live.py --model yolov8n --conf 0.4 --camera 0

Keys:
    [  /  ]   lower / raise confidence threshold (±0.05)
    m         cycle to next model
    l         toggle JSON-lines logging to stdout
    q         quit
"""
from __future__ import annotations
import argparse
import collections
import json
import sys
import time

import cv2
import numpy as np

from detectors import load_detector, YOLO_MODELS, Detection

ALL_MODELS = YOLO_MODELS  # mobilenet-ssd added here when wrapper is ready

# ── Drawing helpers ────────────────────────────────────────────────────────────

GREEN  = (0, 220, 80)
GREY   = (120, 120, 120)
RED    = (0, 60, 220)
YELLOW = (0, 200, 220)
WHITE  = (240, 240, 240)
BLACK  = (0, 0, 0)

FONT  = cv2.FONT_HERSHEY_SIMPLEX
FS    = 0.55
FT    = 1


def _text(img, msg, xy, color=WHITE, scale=FS, thickness=FT):
    cv2.putText(img, msg, xy, FONT, scale, BLACK, thickness + 2, cv2.LINE_AA)
    cv2.putText(img, msg, xy, FONT, scale, color, thickness, cv2.LINE_AA)


def _draw_detection(img, det: Detection):
    h, w = img.shape[:2]
    if det.found and det.bbox:
        x1, y1, x2, y2 = det.bbox
        px1, py1 = int(x1 * w), int(y1 * h)
        px2, py2 = int(x2 * w), int(y2 * h)
        cv2.rectangle(img, (px1, py1), (px2, py2), GREEN, 2)
        label = f"cat {det.confidence:.2f}"
        _text(img, label, (px1, max(py1 - 8, 14)), GREEN, scale=0.5)
    if det.person_present:
        _text(img, "PERSON (suppressed)", (8, h - 12), YELLOW, scale=0.45)


def _draw_hud(img, model_id, conf, fps, det: Detection, logging_on: bool):
    h, w = img.shape[:2]
    status = "FOUND ●" if det.found else "NOT FOUND ○"
    color  = GREEN if det.found else GREY
    _text(img, f"{model_id}  thr={conf:.2f}", (8, 20))
    _text(img, f"{fps:.1f} FPS", (w - 95, 20))
    _text(img, status, (w - 130, h - 12), color)
    if logging_on:
        _text(img, "LOG ON", (8, 42), YELLOW, scale=0.45)


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model",  default="yolov8n", choices=ALL_MODELS)
    ap.add_argument("--conf",   type=float, default=0.4)
    ap.add_argument("--camera", type=int, default=0)
    args = ap.parse_args()

    model_idx  = ALL_MODELS.index(args.model) if args.model in ALL_MODELS else 0
    conf       = args.conf
    logging_on = False
    frame_id   = 0

    print(f"Loading {ALL_MODELS[model_idx]}…")
    detector = load_detector(ALL_MODELS[model_idx], conf=conf)
    detector.warmup()
    print("Ready. Press 'q' to quit.")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        sys.exit(f"Cannot open camera {args.camera}")

    fps_buf = collections.deque(maxlen=30)
    t_last  = time.perf_counter()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        det = detector.detect(frame)

        t_now = time.perf_counter()
        fps_buf.append(1.0 / max(t_now - t_last, 1e-6))
        t_last = t_now
        fps = sum(fps_buf) / len(fps_buf)

        vis = frame.copy()
        _draw_detection(vis, det)
        _draw_hud(vis, ALL_MODELS[model_idx], conf, fps, det, logging_on)

        cv2.imshow("PiCar-X Cat Detector — live.py", vis)

        if logging_on:
            record = {
                "frame_id": frame_id,
                "model": ALL_MODELS[model_idx],
                "conf_threshold": conf,
                "found": det.found,
                "x": round(det.x, 4),
                "y": round(det.y, 4),
                "confidence": round(det.confidence, 4),
                "bbox": [round(v, 4) for v in det.bbox] if det.bbox else None,
                "person_present": det.person_present,
                "fps": round(fps, 1),
            }
            print(json.dumps(record), flush=True)

        frame_id += 1

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('['):
            conf = max(0.05, conf - 0.05)
            detector.set_conf(conf)
            print(f"Threshold → {conf:.2f}")
        elif key == ord(']'):
            conf = min(0.95, conf + 0.05)
            detector.set_conf(conf)
            print(f"Threshold → {conf:.2f}")
        elif key == ord('m'):
            model_idx = (model_idx + 1) % len(ALL_MODELS)
            new_model = ALL_MODELS[model_idx]
            print(f"Loading {new_model}…")
            detector = load_detector(new_model, conf=conf)
            detector.warmup()
            print(f"Switched to {new_model}")
        elif key == ord('l'):
            logging_on = not logging_on
            print(f"Logging {'ON' if logging_on else 'OFF'}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
