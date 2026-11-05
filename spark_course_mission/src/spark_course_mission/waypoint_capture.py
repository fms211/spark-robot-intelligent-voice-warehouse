"""Turn a fresh, localized AMCL sample into a reviewable waypoint snippet."""
import math
import re

from spark_course_mission.navigation import validate_navigation_pose


_STATION_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,31}$")


def capture_candidate(message, now_s, kind, station_id, pickup_enabled,
                      max_age_s=1.0, max_xy_sigma_m=0.25,
                      max_yaw_sigma_rad=0.35):
    """Return YAML text; never write to a mission configuration file."""
    if kind not in ("station", "home", "drop"):
        raise ValueError("kind must be station, home or drop")
    if type(pickup_enabled) is not bool:
        raise ValueError("pickup_enabled must be boolean")
    if kind == "station":
        if not isinstance(station_id, str) or not _STATION_ID.fullmatch(station_id):
            raise ValueError("station_id must be a simple identifier")
    elif station_id or pickup_enabled:
        raise ValueError("home/drop require empty station_id and false pickup_enabled")
    thresholds = (max_age_s, max_xy_sigma_m, max_yaw_sigma_rad)
    if not all(math.isfinite(value) and value > 0 for value in thresholds):
        raise ValueError("capture threshold must be finite and positive")

    if message is None:
        raise ValueError("no AMCL pose received")
    if message.header.frame_id != "map":
        raise ValueError("AMCL pose must use map frame")
    stamp = message.header.stamp.to_sec()
    age = float(now_s) - stamp
    if not math.isfinite(stamp) or stamp <= 0 or not math.isfinite(age):
        raise ValueError("AMCL timestamp is invalid")
    if age < -0.1:
        raise ValueError("AMCL pose timestamp is in the future")
    if age > float(max_age_s):
        raise ValueError("AMCL pose is stale")

    covariance = message.pose.covariance
    if len(covariance) != 36 or not all(math.isfinite(value) for value in covariance):
        raise ValueError("AMCL covariance is invalid")
    if all(covariance[index] == 0 for index in (0, 7, 35)):
        raise ValueError("AMCL covariance is uninitialized")
    for index, limit in ((0, max_xy_sigma_m), (7, max_xy_sigma_m),
                         (35, max_yaw_sigma_rad)):
        variance = covariance[index]
        if variance < 0 or variance > float(limit) ** 2:
            raise ValueError("AMCL covariance exceeds capture threshold")

    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    pose = validate_navigation_pose((
        position.x, position.y, position.z,
        orientation.x, orientation.y, orientation.z, orientation.w,
    ))
    encoded = "[{}]".format(", ".join("{:.6f}".format(value) for value in pose))
    if kind == "station":
        return "- id: {}\n  pose: {}\n  pickup_enabled: {}".format(
            station_id, encoded, str(pickup_enabled).lower())
    return "{}_pose: {}".format(kind, encoded)
