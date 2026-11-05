"""The public ROS wire contracts must preserve field values."""
from io import BytesIO
import importlib

import pytest


def message_type(name):
    try:
        return getattr(importlib.import_module("spark_course_mission.msg"), name)
    except (ImportError, AttributeError) as exc:
        pytest.fail("generated Spark course mission contract is missing: {}".format(exc))


def test_inspection_result_round_trip():
    InspectionResult = message_type("InspectionResult")
    result = InspectionResult()
    result.station_id = "patrol_a"
    result.found = True
    result.color_name = "blue"
    result.center_u = 321.5
    result.center_v = 216.0
    result.area_px = 1200.0
    result.confidence = 5.0 / 7.0
    result.header.frame_id = "camera_color_optical_frame"

    encoded = BytesIO()
    result.serialize(encoded)
    decoded = InspectionResult().deserialize(encoded.getvalue())

    assert decoded.station_id == "patrol_a"
    assert decoded.found is True
    assert decoded.color_name == "blue"
    assert decoded.center_u == pytest.approx(321.5)
    assert decoded.center_v == pytest.approx(216.0)
    assert decoded.area_px == pytest.approx(1200.0)
    assert decoded.confidence == pytest.approx(5.0 / 7.0)
    assert decoded.header.frame_id == "camera_color_optical_frame"


def test_pick_place_goal_and_result_contract():
    Goal = message_type("PickPlaceGoal")
    Result = message_type("PickPlaceResult")
    Feedback = message_type("PickPlaceFeedback")

    goal = Goal()
    assert (Goal.PICK, Goal.PLACE) == (1, 2)
    goal.mode = Goal.PICK
    goal.station_id = "pickup_a"
    goal.center_u = 320.0
    goal.center_v = 240.0
    result = Result()
    result.success = False
    result.carrying = True
    result.code = "NO_PORT"
    result.detail = "arm unavailable"
    feedback = Feedback()
    feedback.phase = "PRECHECK"

    encoded = BytesIO()
    goal.serialize(encoded)
    decoded = Goal().deserialize(encoded.getvalue())
    assert decoded.mode == Goal.PICK
    assert decoded.station_id == "pickup_a"
    assert decoded.center_u == pytest.approx(320.0)
    assert decoded.center_v == pytest.approx(240.0)
    assert (result.success, result.carrying, result.code, result.detail, feedback.phase) == (
        False, True, "NO_PORT", "arm unavailable", "PRECHECK"
    )
