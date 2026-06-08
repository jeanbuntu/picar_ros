from __future__ import annotations
import numpy as np
from .base import BaseDetector, Detection, COCO_CAT, COCO_PERSON


class YOLODetector(BaseDetector):
    """Wraps ultralytics YOLO (v8n, v8s, v11n, …) to emit the Detection contract."""

    SUPPORTED = ("yolov8n", "yolov8s", "yolov11n", "yolov8m")

    def __init__(self, model_name: str = "yolov8n", conf: float = 0.4):
        from ultralytics import YOLO
        self.model_id = model_name
        self.conf = conf
        self._model = YOLO(model_name + ".pt")

    def set_conf(self, conf: float):
        self.conf = max(0.05, min(0.95, conf))

    def detect(self, frame: np.ndarray) -> Detection:
        h, w = frame.shape[:2]
        results = self._model(frame, verbose=False, conf=self.conf)

        best_cat = None       # (conf, xyxy)
        person_present = False

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                xyxy   = box.xyxy[0].tolist()  # pixels: [x1, y1, x2, y2]

                if cls_id == COCO_PERSON:
                    person_present = True
                elif cls_id == COCO_CAT:
                    if best_cat is None or conf > best_cat[0]:
                        best_cat = (conf, xyxy)

        if best_cat is None:
            return Detection(found=False, x=0.0, y=0.0, confidence=0.0,
                             bbox=None, person_present=person_present)

        conf, (x1, y1, x2, y2) = best_cat
        cx = ((x1 + x2) / 2) / w
        cy = ((y1 + y2) / 2) / h
        bbox_norm = (x1 / w, y1 / h, x2 / w, y2 / h)
        return Detection(found=True, x=cx, y=cy, confidence=conf,
                         bbox=bbox_norm, person_present=person_present)

    def warmup(self):
        self.detect(np.zeros((480, 640, 3), dtype=np.uint8))
