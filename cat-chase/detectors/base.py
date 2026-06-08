from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Tuple
import numpy as np

# COCO class IDs (0-indexed, as used by ultralytics/YOLO and TFLite COCO models)
COCO_CAT    = 15
COCO_PERSON = 0


@dataclass
class Detection:
    found: bool
    x: float                                    # bbox center, normalized 0–1 (left→right)
    y: float                                    # bbox center, normalized 0–1 (top→bottom)
    confidence: float
    bbox: Optional[Tuple[float, float, float, float]]  # (x1,y1,x2,y2) normalized, or None
    person_present: bool = False                # person seen but suppressed for targeting


class BaseDetector:
    model_id: str = "base"

    def detect(self, frame: np.ndarray) -> Detection:
        raise NotImplementedError

    def warmup(self):
        h, w = 480, 640
        self.detect(np.zeros((h, w, 3), dtype=np.uint8))
