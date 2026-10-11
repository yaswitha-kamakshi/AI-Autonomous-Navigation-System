"""OpenCV obstacle detection: blur -> color threshold -> clean up -> contours."""
from dataclasses import dataclass
import cv2
import numpy as np
from src.utils import config as cfg


def _bgr(rgb):
    return (rgb[2], rgb[1], rgb[0])


CLASSES = {
    "static": _bgr(cfg.STATIC_OBS),
    "moving": _bgr(cfg.MOVING_OBS),
    "wall": _bgr(cfg.WALL),
}


@dataclass
class Detection:
    label: str
    x: int
    y: int
    w: int
    h: int

    @property
    def center(self):
        return (self.x + self.w // 2, self.y + self.h // 2)


def detect(frame):
    """Return (list of Detection, combined obstacle mask)."""
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    kernel = np.ones((3, 3), np.uint8)
    combined = np.zeros(frame.shape[:2], np.uint8)
    detections = []

    for label, color in CLASSES.items():
        c = np.array(color, dtype=np.int32)
        lo = np.clip(c - cfg.COLOR_TOL, 0, 255).astype(np.uint8)
        hi = np.clip(c + cfg.COLOR_TOL, 0, 255).astype(np.uint8)
        mask = cv2.inRange(blur, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove speckle
        combined = cv2.bitwise_or(combined, mask)

        # [-2] works for both 2-value and 3-value return styles of OpenCV
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for cnt in contours:
            if cv2.contourArea(cnt) < cfg.MIN_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            detections.append(Detection(label, x, y, w, h))

    return detections, combined