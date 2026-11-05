"""Calibrated SwiftPro target planning and cancellation-safe step execution."""
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple
import math


class CalibrationError(ValueError):
    pass


class ArmBoundsError(ValueError):
    pass


class DriverUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True)
class ArmStep:
    phase: str
    x_mm: Optional[float] = None
    y_mm: Optional[float] = None
    z_mm: Optional[float] = None
    pump: Optional[bool] = None
    dwell_s: float = 0.0


@dataclass(frozen=True)
class ArmOutcome:
    code: str
    carrying: bool

    @property
    def success(self) -> bool:
        return self.code == "COMMANDS_SENT_UNVERIFIED"


def _matrix(calibration):
    if not calibration.get("valid"):
        raise CalibrationError("arm calibration is not marked valid")
    matrix = calibration.get("matrix")
    if not isinstance(matrix, (list, tuple)) or len(matrix) != 2:
        raise CalibrationError("expected a 2x3 pixel-to-arm matrix")
    try:
        rows = [tuple(float(value) for value in row) for row in matrix]
    except (TypeError, ValueError) as exc:
        raise CalibrationError("invalid pixel-to-arm matrix") from exc
    if any(len(row) != 3 or not all(math.isfinite(v) for v in row)
           for row in rows):
        raise CalibrationError("invalid pixel-to-arm matrix")
    return rows


def _pixel_roi(calibration):
    roi = calibration["pixel_roi"]
    limits = []
    for axis in ("u", "v"):
        low, high = (float(value) for value in roi[axis])
        if not all(math.isfinite(value) for value in (low, high)) or low < 0 or low >= high:
            raise CalibrationError("invalid calibrated pixel ROI")
        limits.append((low, high))
    return limits


def pixel_to_arm(u: float, v: float, calibration) -> Tuple[float, float]:
    rows = _matrix(calibration)
    if calibration.get("calibration_mode") == "field":
        (min_u, max_u), (min_v, max_v) = _pixel_roi(calibration)
        if not min_u <= u <= max_u or not min_v <= v <= max_v:
            raise ArmBoundsError("pixel outside calibrated ROI")
    return (
        rows[0][0] * u + rows[0][1] * v + rows[0][2],
        rows[1][0] * u + rows[1][1] * v + rows[1][2],
    )


def validate_real_ready(calibration, port_exists: bool) -> bool:
    if not port_exists or calibration.get("calibration_mode") != "field":
        return False
    try:
        _matrix(calibration)
        bounds = calibration["bounds_mm"]
        heights = calibration["heights_mm"]
        drop = calibration["drop_mm"]
        transport = calibration["transport_mm"]
        for axis in ("x", "y", "z"):
            low, high = bounds[axis]
            if not math.isfinite(float(low)) or not math.isfinite(float(high)) or low >= high:
                return False
        radius_low, radius_high = bounds["radius"]
        if (not math.isfinite(float(radius_low)) or
                not math.isfinite(float(radius_high)) or
                not 50.0 <= float(radius_low) < float(radius_high) <= 320.0):
            return False
        _pixel_roi(calibration)
        for key in ("approach", "pick", "transport"):
            if not math.isfinite(float(heights[key])):
                return False
        if not (float(heights["pick"]) < float(heights["approach"])
                <= float(heights["transport"])):
            return False
        if not float(drop["z"]) < float(heights["approach"]):
            return False
        if abs(float(transport["z"]) - float(heights["transport"])) > 1e-6:
            return False
        _check_target(drop["x"], drop["y"], drop["z"], bounds)
        _check_target(drop["x"], drop["y"], heights["approach"], bounds)
        _check_target(drop["x"], drop["y"], heights["transport"], bounds)
        _check_target(transport["x"], transport["y"], transport["z"], bounds)
    except (CalibrationError, ArmBoundsError, KeyError, TypeError, ValueError):
        return False
    return True


def _check_target(x_mm: float, y_mm: float, z_mm: float, bounds):
    for axis, value in (("x", x_mm), ("y", y_mm), ("z", z_mm)):
        low, high = bounds[axis]
        if not math.isfinite(float(value)) or not float(low) <= float(value) <= float(high):
            raise ArmBoundsError("{} target outside calibrated workspace".format(axis))
    if "radius" in bounds:
        low, high = bounds["radius"]
        radius = math.hypot(float(x_mm), float(y_mm))
        if not float(low) <= radius <= float(high):
            raise ArmBoundsError("radial target outside calibrated workspace")


def grasp_verdict(u: float, v: float, calibration,
                  scan_step_px: float = 0.5, u_limits=None):
    """Classify a detected pixel for the calibrated pick workspace.

    Returns ("ok", None) when the pixel maps inside the same x/y/radius
    bounds the adapter enforces, ("aim", target_u) when this image row has
    a reachable pixel column and target_u is the middle of the nearest one,
    or ("impossible", None) when no u at this v is inside the workspace.
    u_limits optionally restricts the reachable columns to the ALIGN window.
    """
    rows = _matrix(calibration)
    bounds = calibration["bounds_mm"]
    heights = calibration["heights_mm"]
    pick_height = float(heights["pick"])
    (min_u, max_u), (min_v, max_v) = _pixel_roi(calibration)
    u_value, v_value = float(u), float(v)
    if not (math.isfinite(u_value) and math.isfinite(v_value)
            and min_v <= v_value <= max_v):
        return "impossible", None
    step = float(scan_step_px)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("scan_step_px must be positive")

    def reachable(candidate_u):
        x_mm = rows[0][0] * candidate_u + rows[0][1] * v_value + rows[0][2]
        y_mm = rows[1][0] * candidate_u + rows[1][1] * v_value + rows[1][2]
        try:
            _check_target(x_mm, y_mm, pick_height, bounds)
        except ArmBoundsError:
            return False
        return True

    if reachable(u_value):
        return "ok", None

    scan_low, scan_high = min_u, max_u
    if u_limits is not None:
        try:
            limit_low, limit_high = (float(value) for value in u_limits)
        except (TypeError, ValueError) as exc:
            raise ValueError("u_limits must be two numeric values") from exc
        if not (math.isfinite(limit_low) and math.isfinite(limit_high)):
            raise ValueError("u_limits must be finite")
        scan_low = max(min_u, min(limit_low, limit_high))
        scan_high = min(max_u, max(limit_low, limit_high))
        if scan_low > scan_high:
            return "impossible", None

    intervals = []
    start = None
    previous = scan_low
    index = 0
    while True:
        candidate = min(scan_low + index * step, scan_high)
        if reachable(candidate):
            if start is None:
                start = candidate
        elif start is not None:
            intervals.append((start, previous))
            start = None
        previous = candidate
        if candidate >= scan_high:
            break
        index += 1
    if start is not None:
        intervals.append((start, scan_high))
    if not intervals:
        return "impossible", None

    def distance_to(interval):
        low, high = interval
        if u_value < low:
            return low - u_value
        if u_value > high:
            return u_value - high
        return 0.0

    low, high = min(intervals, key=distance_to)
    return "aim", 0.5 * (low + high)


def plan_pick(x_mm: float, y_mm: float, heights, bounds,
              transport_pose) -> List[ArmStep]:
    approach = float(heights["approach"])
    pick = float(heights["pick"])
    transport_height = float(heights["transport"])
    if not pick < approach <= transport_height:
        raise CalibrationError("pick, approach and transport heights must ascend")
    for z_mm in (approach, pick, transport_height):
        _check_target(x_mm, y_mm, z_mm, bounds)
    transport_x = float(transport_pose["x"])
    transport_y = float(transport_pose["y"])
    transport_z = float(transport_pose["z"])
    if abs(transport_z - transport_height) > 1e-6:
        raise CalibrationError("transport pose height must equal transport clearance")
    _check_target(transport_x, transport_y, transport_z, bounds)
    move_wait = float(heights.get("move_wait_s", 2.0))
    pump_wait = float(heights.get("pump_wait_s", 1.0))
    return [
        ArmStep("PICK_TRANSFER", x_mm, y_mm, transport_height, dwell_s=move_wait),
        ArmStep("APPROACH", x_mm, y_mm, approach, dwell_s=move_wait),
        ArmStep("LOWER", x_mm, y_mm, pick, dwell_s=move_wait),
        ArmStep("SUCTION_ON", pump=True, dwell_s=pump_wait),
        ArmStep("LIFT", x_mm, y_mm, approach, dwell_s=move_wait),
        ArmStep("CLEARANCE", x_mm, y_mm, transport_height, dwell_s=move_wait),
        ArmStep("TRANSPORT", transport_x, transport_y, transport_z,
                dwell_s=move_wait),
    ]


def plan_place(drop, heights, bounds, transport_pose) -> List[ArmStep]:
    x_mm, y_mm, z_mm = float(drop["x"]), float(drop["y"]), float(drop["z"])
    approach = float(heights["approach"])
    transport_height = float(heights["transport"])
    if not z_mm < approach <= transport_height:
        raise CalibrationError("drop, approach and transport heights must ascend")
    transport_x = float(transport_pose["x"])
    transport_y = float(transport_pose["y"])
    transport_z = float(transport_pose["z"])
    if abs(transport_z - transport_height) > 1e-6:
        raise CalibrationError("transport pose height must equal transport clearance")
    _check_target(transport_x, transport_y, transport_z, bounds)
    for target_z in (transport_height, approach, z_mm):
        _check_target(x_mm, y_mm, target_z, bounds)
    move_wait = float(heights.get("move_wait_s", 2.0))
    pump_wait = float(heights.get("pump_wait_s", 1.0))
    return [
        ArmStep("PLACE_TRANSFER", x_mm, y_mm, transport_height,
                dwell_s=move_wait),
        ArmStep("PLACE_APPROACH", x_mm, y_mm, approach, dwell_s=move_wait),
        ArmStep("PLACE_LOWER", x_mm, y_mm, z_mm, dwell_s=move_wait),
        ArmStep("SUCTION_OFF", pump=False, dwell_s=pump_wait),
        ArmStep("PLACE_RETRACT", x_mm, y_mm, approach, dwell_s=move_wait),
        ArmStep("PLACE_CLEARANCE", x_mm, y_mm, transport_height, dwell_s=move_wait),
        ArmStep("PLACE_STOW", transport_x, transport_y, transport_z,
                dwell_s=move_wait),
    ]


def run_steps(steps: List[ArmStep], send: Callable[[ArmStep], None],
              should_cancel: Callable[[], bool], wait: Callable[[float], None],
              initial_carrying: bool = False) -> ArmOutcome:
    carrying = initial_carrying
    for step in steps:
        if should_cancel():
            return ArmOutcome("CANCELED_CARRYING" if carrying else "CANCELED", carrying)
        send(step)
        if step.pump is not None:
            carrying = step.pump
        wait(step.dwell_s)
    return ArmOutcome("COMMANDS_SENT_UNVERIFIED", carrying)
