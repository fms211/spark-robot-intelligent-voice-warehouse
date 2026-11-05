"""Color detections require fresh, stable evidence before pickup."""
import importlib

import cv2
import numpy as np
import pytest


def detector():
    try:
        return importlib.import_module("spark_course_mission.detection")
    except ImportError as exc:
        pytest.fail("color detector is missing: {}".format(exc))


def colored_frame(color=(255, 0, 0), start=(35, 35), end=(65, 65)):
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.rectangle(frame, start, end, color, -1)
    return frame


def find(frame, roi=(0, 0, 100, 100), min_area=400):
    return detector().detect_hsv(frame, (100, 90, 80), (130, 255, 255), roi, min_area)


def test_blue_object_produces_center_and_area():
    candidate = find(colored_frame())
    assert candidate is not None
    assert candidate.center_u == pytest.approx(50, abs=1)
    assert candidate.center_v == pytest.approx(50, abs=1)
    assert candidate.area_px >= 800


@pytest.mark.parametrize("frame, roi, area", [
    (colored_frame(color=(0, 0, 255)), (0, 0, 100, 100), 400),
    (colored_frame(start=(45, 45), end=(50, 50)), (0, 0, 100, 100), 400),
    (colored_frame(), (0, 0, 30, 30), 400),
])
def test_wrong_color_noise_or_outside_roi_is_ignored(frame, roi, area):
    assert find(frame, roi, area) is None


def test_five_of_seven_consistent_positives_become_stable():
    module = detector()
    tracker = module.StableTracker(window=7, required=5, max_jitter_px=5)
    decision = None
    for idx in range(7):
        candidate = module.Candidate(50 + (idx % 2), 49, 900) if idx < 5 else None
        decision = tracker.add(candidate, stamp=idx * 0.05, now=idx * 0.05)
    assert decision is not None
    assert decision.found is True
    assert decision.center_u == pytest.approx(50.4)
    assert decision.confidence == pytest.approx(5.0 / 7.0)


def test_five_fresh_negative_frames_report_absence():
    module = detector()
    tracker = module.StableTracker(window=7, required=5)
    decision = None
    for idx in range(7):
        candidate = module.Candidate(50, 50, 900) if idx < 2 else None
        decision = tracker.add(candidate, stamp=idx * 0.05, now=idx * 0.05)
    assert decision is not None
    assert decision.found is False
    assert decision.confidence == pytest.approx(5.0 / 7.0)


def test_moving_target_is_not_stable():
    module = detector()
    tracker = module.StableTracker(window=7, required=5, max_jitter_px=3)
    decision = None
    for idx in range(7):
        decision = tracker.add(module.Candidate(10 + idx * 10, 50, 900),
                               stamp=idx * 0.05, now=idx * 0.05)
    assert decision is None


def test_stale_frame_does_not_enter_stability_window():
    module = detector()
    tracker = module.StableTracker(window=7, required=5, max_age_s=0.2)
    assert tracker.add(module.Candidate(50, 50, 900), stamp=1.0, now=2.0) is None
    decision = None
    for idx in range(7):
        now = 2.0 + idx * 0.02
        decision = tracker.add(None, stamp=now, now=now)
    assert decision is not None
    assert decision.found is False


def test_old_window_samples_cannot_make_a_new_target_after_camera_gap():
    module = detector()
    tracker = module.StableTracker(window=7, required=5, max_age_s=0.5)
    for idx in range(6):
        stamp = 1.0 + idx * 0.05
        tracker.add(module.Candidate(50, 50, 900), stamp=stamp, now=stamp)
    decision = tracker.add(module.Candidate(50, 50, 900), stamp=3.0, now=3.0)
    assert decision is None


def test_duplicate_image_timestamp_cannot_fill_stability_window():
    module = detector()
    tracker = module.StableTracker(window=7, required=5)
    decisions = [tracker.add(module.Candidate(50, 50, 900),
                             stamp=4.0, now=4.0) for _ in range(7)]
    assert all(decision is None for decision in decisions)
