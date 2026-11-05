"""Validation for field navigation coordinates."""
import math


def validate_navigation_pose(pose):
    """Return a numeric pose or reject values unsafe for move_base."""
    if not isinstance(pose, (list, tuple)) or len(pose) != 7:
        raise ValueError("navigation pose needs x,y,z,qx,qy,qz,qw")
    try:
        values = tuple(float(value) for value in pose)
    except (TypeError, ValueError) as exc:
        raise ValueError("navigation pose values must be numeric") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("navigation pose values must be finite")
    if any(abs(value) > 1e-3 for value in (values[2], values[3], values[4])):
        raise ValueError("navigation pose must be planar with z,qx,qy near zero")

    quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
    if not math.isclose(quaternion_norm, 1.0, rel_tol=0.0, abs_tol=1e-3):
        raise ValueError("navigation quaternion must be normalized")
    return values


def validate_navigation_config(config):
    """Validate every pose that a real patrol can dispatch."""
    stations = config.get("stations")
    if not isinstance(stations, list) or len(stations) < 2:
        raise ValueError("at least two navigation stations are required")
    pickup_station_count = 0
    for index, station in enumerate(stations):
        if not isinstance(station, dict) or not station.get("id"):
            raise ValueError("station {} needs a non-empty id".format(index))
        if type(station.get("pickup_enabled")) is not bool:
            raise ValueError(
                "station {}: pickup_enabled must be boolean".format(station["id"]))
        pickup_station_count += int(station["pickup_enabled"])
        try:
            validate_navigation_pose(station.get("pose"))
        except ValueError as exc:
            raise ValueError("station {}: {}".format(station["id"], exc)) from exc
    if pickup_station_count != 1:
        raise ValueError("exactly one pickup-enabled station is required")
    for key in ("drop_pose", "home_pose"):
        try:
            validate_navigation_pose(config.get(key))
        except ValueError as exc:
            raise ValueError("{}: {}".format(key, exc)) from exc
    return True
