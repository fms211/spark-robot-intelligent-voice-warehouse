"""Pure helpers for the pickup-station alignment window."""
import math

DEFAULT_INSET_RATIO = 0.15
DIRECTION_FLIP_MARGIN_PX = 5.0
MAX_INSET_RATIO = 0.49


def _axis_bounds(roi, axis, inset_ratio):
    if not isinstance(roi, dict):
        raise ValueError("pixel_roi must be a dict")
    bounds = roi.get(axis)
    if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
        raise ValueError("pixel_roi.{} needs two values".format(axis))
    try:
        low, high = float(bounds[0]), float(bounds[1])
    except (TypeError, ValueError) as exc:
        raise ValueError("pixel_roi.{} must be numeric".format(axis)) from exc
    if not (math.isfinite(low) and math.isfinite(high) and low < high):
        raise ValueError("pixel_roi.{} must be finite and increasing".format(axis))
    margin = (high - low) * inset_ratio
    low_inset, high_inset = low + margin, high - margin
    if not low_inset < high_inset:
        raise ValueError("pixel_roi.{} collapses after inset".format(axis))
    return [low_inset, high_inset]


def build_window(pixel_roi, inset_ratio=DEFAULT_INSET_RATIO):
    """Return the inset grasp window or raise ValueError for unsafe input."""
    try:
        inset = float(inset_ratio)
    except (TypeError, ValueError) as exc:
        raise ValueError("inset_ratio must be numeric") from exc
    if not math.isfinite(inset) or not 0.0 <= inset <= MAX_INSET_RATIO:
        raise ValueError("inset_ratio out of range")
    return {"u": _axis_bounds(pixel_roi, "u", inset),
            "v": _axis_bounds(pixel_roi, "v", inset)}


def inside(window, u, v):
    """True when the pixel is inside the window, boundaries included."""
    if not isinstance(window, dict):
        return False
    try:
        u_low, u_high = window["u"]
        v_low, v_high = window["v"]
        u_value, v_value = float(u), float(v)
    except (KeyError, TypeError, ValueError):
        return False
    return u_low <= u_value <= u_high and v_low <= v_value <= v_high


def horizontal_error(window, u):
    """Horizontal distance from the window center, in pixels."""
    u_low, u_high = window["u"]
    center = 0.5 * (float(u_low) + float(u_high))
    return abs(float(u) - center)
