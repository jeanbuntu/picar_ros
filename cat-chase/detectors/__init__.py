from .base import Detection, BaseDetector, COCO_CAT, COCO_PERSON
from .yolo import YOLODetector

YOLO_MODELS = list(YOLODetector.SUPPORTED)


def load_detector(name: str, conf: float = 0.4) -> BaseDetector:
    if name in YOLO_MODELS:
        return YOLODetector(model_name=name, conf=conf)
    if name == "mobilenet-ssd":
        from .mobilenet import MobileNetDetector
        return MobileNetDetector(conf=conf)
    raise ValueError(f"Unknown detector '{name}'. Choose from: {YOLO_MODELS + ['mobilenet-ssd']}")


__all__ = ["Detection", "BaseDetector", "load_detector", "YOLO_MODELS"]
