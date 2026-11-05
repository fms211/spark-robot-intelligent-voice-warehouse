"""Field waypoint capture must reject untrusted localization samples."""
import importlib
import math

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy


def candidate_module():
    return importlib.import_module("spark_course_mission.waypoint_capture")


def amcl_sample(stamp=100.0):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.from_sec(stamp)
    message.pose.pose.position.x = 1.2
    message.pose.pose.position.y = -0.3
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.01
    message.pose.covariance[7] = 0.02
    message.pose.covariance[35] = 0.04
    return message


def test_station_candidate_uses_measured_pose_and_boolean_permission():
    module = candidate_module()
    result = module.capture_candidate(
        amcl_sample(), 100.2, "station", "pickup_b", True)
    assert result == (
        "- id: pickup_b\n"
        "  pose: [1.200000, -0.300000, 0.000000, 0.000000, 0.000000, "
        "0.000000, 1.000000]\n"
        "  pickup_enabled: true")


@pytest.mark.parametrize("kind, expected", [
    ("home", "home_pose:"),
    ("drop", "drop_pose:"),
])
def test_home_and_drop_candidates_use_config_keys(kind, expected):
    module = candidate_module()
    result = module.capture_candidate(amcl_sample(), 100.2, kind, "", False)
    assert result.startswith(expected + " [1.200000, -0.300000")


@pytest.mark.parametrize("change, reason", [
    (lambda m: setattr(m.header, "frame_id", "odom"), "map frame"),
    (lambda m: setattr(m.header, "stamp", rospy.Time.from_sec(98.0)), "stale"),
    (lambda m: setattr(m.header, "stamp", rospy.Time.from_sec(101.0)), "future"),
    (lambda m: m.pose.covariance.__setitem__(0, 0.09), "covariance"),
    (lambda m: m.pose.covariance.__setitem__(35, 0.25), "covariance"),
    (lambda m: setattr(m.pose.pose.position, "x", math.nan), "finite"),
    (lambda m: setattr(m.pose.pose.position, "z", 0.2), "planar"),
])
def test_rejects_bad_localization_before_formatting(change, reason):
    module = candidate_module()
    sample = amcl_sample()
    change(sample)
    with pytest.raises(ValueError, match=reason):
        module.capture_candidate(sample, 100.2, "station", "patrol_a", False)


def test_rejects_missing_localization():
    module = candidate_module()
    with pytest.raises(ValueError, match="no AMCL"):
        module.capture_candidate(None, 100.0, "station", "patrol_a", False)


def test_rejects_uninitialized_zero_covariance():
    module = candidate_module()
    sample = amcl_sample()
    sample.pose.covariance = [0.0] * 36
    with pytest.raises(ValueError, match="covariance"):
        module.capture_candidate(sample, 100.2, "station", "patrol_a", False)


def test_rejects_invalid_uncertainty_thresholds():
    module = candidate_module()
    with pytest.raises(ValueError, match="threshold"):
        module.capture_candidate(
            amcl_sample(), 100.2, "station", "patrol_a", False,
            max_xy_sigma_m=-0.25)


@pytest.mark.parametrize("kind, station_id, pickup_enabled", [
    ("station", "bad:yaml", False),
    ("station", "", False),
    ("home", "surprise", False),
    ("drop", "", True),
    ("unknown", "", False),
])
def test_rejects_invalid_request_instead_of_emitting_yaml(
        kind, station_id, pickup_enabled):
    module = candidate_module()
    with pytest.raises(ValueError):
        module.capture_candidate(
            amcl_sample(), 100.2, kind, station_id, pickup_enabled)
