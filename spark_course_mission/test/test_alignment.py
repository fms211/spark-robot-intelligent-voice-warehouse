"""Alignment window helpers are pure and reject unsafe geometry."""
import pytest

from spark_course_mission.alignment import (
    DEFAULT_INSET_RATIO, build_window, horizontal_error, inside,
)

ROI = {"u": [100.0, 300.0], "v": [150.0, 250.0]}


def test_default_inset_ratio_is_fifteen_percent():
    assert DEFAULT_INSET_RATIO == 0.15


def test_build_window_insets_each_side():
    window = build_window(ROI, 0.1)
    assert window == {"u": [120.0, 280.0], "v": [160.0, 240.0]}


def test_build_window_rejects_missing_or_bad_roi():
    with pytest.raises(ValueError):
        build_window({})
    with pytest.raises(ValueError):
        build_window({"u": [100.0, 100.0], "v": [0.0, 10.0]})
    with pytest.raises(ValueError):
        build_window({"u": ["x", 10.0], "v": [0.0, 10.0]})
    with pytest.raises(ValueError):
        build_window({"u": [0.0, 10.0], "v": [0.0, 10.0]}, -0.1)
    with pytest.raises(ValueError):
        build_window({"u": [0.0, 10.0], "v": [0.0, 10.0]}, 0.5)


def test_inside_includes_boundaries():
    window = build_window(ROI, 0.1)
    assert inside(window, 120.0, 160.0)
    assert inside(window, 280.0, 240.0)
    assert not inside(window, 119.9, 200.0)
    assert not inside(window, 200.0, 240.1)
    assert not inside(None, 200.0, 200.0)


def test_horizontal_error_uses_window_center():
    window = build_window(ROI, 0.1)
    assert horizontal_error(window, 200.0) == 0.0
    assert horizontal_error(window, 150.0) == 50.0
    assert horizontal_error(window, 250.0) == 50.0
