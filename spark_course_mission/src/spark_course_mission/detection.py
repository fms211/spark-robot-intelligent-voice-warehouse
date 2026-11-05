"""HSV candidate extraction and short-window evidence filtering."""
from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Optional, Sequence
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class Candidate:
    center_u: float
    center_v: float
    area_px: float


@dataclass(frozen=True)
class InspectionDecision:
    found: bool
    center_u: float
    center_v: float
    area_px: float
    confidence: float
    stamp: float


def detect_hsv(frame: np.ndarray, hsv_lower: Sequence[int], hsv_upper: Sequence[int],
               roi: Sequence[int], min_area: float) -> Optional[Candidate]:
    """Find the largest color contour inside a pixel-coordinate ROI."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("expected a BGR color image")
    height, width = frame.shape[:2]
    x0, y0, x1, y1 = (int(value) for value in roi)
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(width, x1), min(height, y1)
    if x0 >= x1 or y0 >= y1:
        return None

    cropped = frame[y0:y1, x0:x1]
    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lower, dtype=np.uint8),
                       np.array(hsv_upper, dtype=np.uint8))
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area:
        return None
    moments = cv2.moments(contour)
    if moments["m00"] <= 0:
        return None
    return Candidate(
        center_u=x0 + moments["m10"] / moments["m00"],
        center_v=y0 + moments["m01"] / moments["m00"],
        area_px=area,
    )


class StableTracker:
    """Require consistent positive or negative evidence in a fresh frame window."""

    def __init__(self, window: int = 7, required: int = 5,
                 max_jitter_px: float = 12.0, max_age_s: float = 0.5):
        if window < 1 or required < 1 or required > window:
            raise ValueError("invalid detection window")
        self.window = window
        self.required = required
        self.max_jitter_px = max_jitter_px
        self.max_age_s = max_age_s
        self._samples = deque(maxlen=window)
        self._last_stamp = None

    def reset(self):
        self._samples.clear()
        self._last_stamp = None

    def add(self, candidate: Optional[Candidate], stamp: float,
            now: float) -> Optional[InspectionDecision]:
        if (not math.isfinite(stamp) or not math.isfinite(now)
                or stamp < 0 or now - stamp > self.max_age_s
                or stamp - now > 0.1
                or (self._last_stamp is not None and stamp <= self._last_stamp)):
            return None
        while self._samples and now - self._samples[0][0] > self.max_age_s:
            self._samples.popleft()
        self._last_stamp = stamp
        self._samples.append((stamp, candidate))
        if len(self._samples) < self.window:
            return None

        positives = [sample for _, sample in self._samples if sample is not None]
        negatives = self.window - len(positives)
        if negatives >= self.required:
            return InspectionDecision(False, 0.0, 0.0, 0.0,
                                      negatives / self.window, stamp)
        if len(positives) < self.required:
            return None

        center_u = median(sample.center_u for sample in positives)
        center_v = median(sample.center_v for sample in positives)
        inliers = [sample for sample in positives
                   if math.hypot(sample.center_u - center_u,
                                 sample.center_v - center_v) <= self.max_jitter_px]
        if len(inliers) < self.required:
            return None
        return InspectionDecision(
            True,
            sum(sample.center_u for sample in inliers) / len(inliers),
            sum(sample.center_v for sample in inliers) / len(inliers),
            sum(sample.area_px for sample in inliers) / len(inliers),
            len(inliers) / self.window,
            stamp,
        )
