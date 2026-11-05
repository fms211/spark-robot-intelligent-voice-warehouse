"""Translate the existing move2grasp calibration without enabling hardware."""
import math


def convert_legacy_calibration(text):
    """Convert k1 b1 k2 b2 to a disabled [u,v,1] calibration template."""
    try:
        values = [float(value) for value in text.split()]
    except (TypeError, ValueError, AttributeError) as exc:
        raise ValueError('expected four finite calibration coefficients') from exc
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError('expected four finite calibration coefficients')
    k1, b1, k2, b2 = values
    return {
        'valid': False,
        'calibration_mode': 'uncalibrated',
        'matrix': [[0.0, k1, b1], [k2, 0.0, b2]],
        'pixel_roi': {},
        'bounds_mm': {},
        'heights_mm': {},
        'drop_mm': {},
        'transport_mm': {},
    }
