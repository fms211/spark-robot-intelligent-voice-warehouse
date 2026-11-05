"""Navigation poses must be safe before they reach move_base."""
import importlib
import math

import pytest


def navigation_module():
    try:
        return importlib.import_module("spark_course_mission.navigation")
    except ImportError as exc:
        pytest.fail("navigation pose validation is missing: {}".format(exc))


def test_accepts_finite_normalized_2d_pose():
    module = navigation_module()
    yaw = -1.70349
    pose = [0.510633, -0.864964, 0.0, 0.0, 0.0,
            math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    assert module.validate_navigation_pose(pose) == pytest.approx(pose)


@pytest.mark.parametrize("pose", [
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    [0.0, float("nan"), 0.0, 0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.510633, -0.864964, 0.0, 0.0, 0.0, 0.0, -1.70349],
])
def test_rejects_malformed_or_non_normalized_pose(pose):
    module = navigation_module()

    with pytest.raises(ValueError):
        module.validate_navigation_pose(pose)


@pytest.mark.parametrize("pose", [
    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, math.sin(0.2), 0.0, 0.0, math.cos(0.2)],
])
def test_rejects_non_planar_navigation_pose(pose):
    module = navigation_module()

    with pytest.raises(ValueError, match="planar"):
        module.validate_navigation_pose(pose)


def test_rejects_waypoint_config_when_any_pose_is_invalid():
    module = navigation_module()
    config = {
        "stations": [
            {"id": "a", "pose": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
             "pickup_enabled": False},
            {"id": "b", "pose": [0.5, -0.8, 0.0, 0.0, 0.0, 0.0, -1.70349],
             "pickup_enabled": True},
        ],
        "drop_pose": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "home_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    }

    with pytest.raises(ValueError, match="station b"):
        module.validate_navigation_config(config)


def test_rejects_string_pickup_permission_that_yaml_truthiness_would_enable():
    module = navigation_module()
    config = {
        "stations": [
            {
                "id": "inspection_only",
                "pose": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "pickup_enabled": "false",
            },
            {
                "id": "pickup",
                "pose": [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "pickup_enabled": True,
            },
        ],
        "drop_pose": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "home_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    }

    with pytest.raises(ValueError, match="pickup_enabled must be boolean"):
        module.validate_navigation_config(config)


@pytest.mark.parametrize("stations, expected", [
    (
        [{"id": "only", "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
          "pickup_enabled": True}],
        "at least two",
    ),
    (
        [
            {"id": "pickup_a", "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
             "pickup_enabled": True},
            {"id": "pickup_b", "pose": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
             "pickup_enabled": True},
        ],
        "exactly one",
    ),
])
def test_requires_two_patrol_stations_and_one_pickup_station(stations, expected):
    module = navigation_module()
    config = {
        "stations": stations,
        "drop_pose": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "home_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    }

    with pytest.raises(ValueError, match=expected):
        module.validate_navigation_config(config)
