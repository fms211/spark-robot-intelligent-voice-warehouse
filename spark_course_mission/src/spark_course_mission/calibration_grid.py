"""2D pixel-to-arm calibration fitting and candidate generation.

This module is deliberately hardware-free.  The ROS script collects samples,
while this module validates the grid, rejects collinear designs, fits a 2x3
affine mapping, removes gross outliers, and builds a disabled candidate YAML.

The candidate is never valid for field motion by itself: z bounds, pick/place
heights, drop pose and transport pose must still be measured and reviewed.
"""
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple
import math

import numpy as np

MATRIX_SHAPE = (2, 3)


class CalibrationGridError(ValueError):
    """Raised when the grid or fit cannot support a 2D calibration."""


@dataclass(frozen=True)
class GridPoint:
    x_mm: float
    y_mm: float
    z_mm: float


@dataclass(frozen=True)
class Sample:
    x_mm: float
    y_mm: float
    u_px: float
    v_px: float
    area_px: float = 0.0


@dataclass(frozen=True)
class FitResult:
    matrix: Tuple[Tuple[float, float, float], Tuple[float, float, float]]
    residuals_mm: Tuple[float, ...]
    inlier_indices: Tuple[int, ...]
    outlier_indices: Tuple[int, ...]
    rank: int
    condition: float
    mean_residual_mm: float
    max_residual_mm: float
    passed: bool
    reason: str


# The 2026-11-02 legacy sweep ran along x=180+5i, y=200-10i, z=-110.
# The 2D grid uses small perpendicular offsets around a subset of that
# line.  The SwiftPro service still verifies every point against encoder
# feedback; this module only generates candidate coordinates.
LEGACY_LINE_START = (180.0, 200.0, -110.0)
LEGACY_LINE_STEP = (5.0, -10.0, 0.0)
GRID_BASE_INDICES = (2, 6, 10, 14, 17)
GRID_OFFSETS_MM = (-10.0, 0.0, 10.0)
GRID_Z_MM = -110.0
DEFAULT_SAFE_ENVELOPE_MM = {
    "x": (170.0, 285.0),
    "y": (0.0, 210.0),
    "z": (-120.0, 40.0),
}


def _perpendicular(dx: float, dy: float) -> Tuple[float, float]:
    norm = math.hypot(dx, dy)
    if norm <= 0.0:
        raise CalibrationGridError("line direction must be non-zero")
    return (-dy / norm, dx / norm)


def default_grid_points() -> List[GridPoint]:
    """Return a warm-up point plus a serpentine 2D grid around the legacy line."""
    dx, dy, _ = LEGACY_LINE_STEP
    px, py = _perpendicular(dx, dy)
    points = [GridPoint(*LEGACY_LINE_START)]
    for order, index in enumerate(GRID_BASE_INDICES):
        offsets = GRID_OFFSETS_MM if order % 2 == 0 else tuple(reversed(GRID_OFFSETS_MM))
        base_x = LEGACY_LINE_START[0] + index * dx
        base_y = LEGACY_LINE_START[1] + index * dy
        for offset in offsets:
            points.append(GridPoint(base_x + offset * px,
                                    base_y + offset * py,
                                    GRID_Z_MM))
    return points


def validate_grid_points(points: Sequence[GridPoint],
                         envelope: Optional[dict] = None) -> None:
    """Reject non-finite, out-of-envelope, or collinear commanded points."""
    if len(points) < 4:
        raise CalibrationGridError("at least four grid points are required")
    limits = DEFAULT_SAFE_ENVELOPE_MM if envelope is None else envelope
    for index, point in enumerate(points):
        values = (point.x_mm, point.y_mm, point.z_mm)
        if not all(math.isfinite(value) for value in values):
            raise CalibrationGridError("grid point {} is not finite".format(index))
        for axis, value in zip(("x", "y", "z"), values):
            try:
                low, high = limits[axis]
                low, high = float(low), float(high)
            except (KeyError, TypeError, ValueError) as exc:
                raise CalibrationGridError("invalid safe envelope for {}".format(axis)) from exc
            if not low <= float(value) <= high:
                raise CalibrationGridError(
                    "grid point {} {}={} outside safe envelope".format(index, axis, value))
    design = np.array([[p.x_mm, p.y_mm, 1.0] for p in points], dtype=float)
    if np.linalg.matrix_rank(design) < 3:
        raise CalibrationGridError("grid points are collinear in x/y; 2D offsets are required")


def _design(samples: Sequence[Sample]) -> np.ndarray:
    return np.array([[s.u_px, s.v_px, 1.0] for s in samples], dtype=float)


def _targets(samples: Sequence[Sample]) -> np.ndarray:
    return np.array([[s.x_mm, s.y_mm] for s in samples], dtype=float)


def _matrix_from_coefficients(coefficients: np.ndarray) -> Tuple[Tuple[float, float, float],
                                                                 Tuple[float, float, float]]:
    return (
        (float(coefficients[0, 0]), float(coefficients[1, 0]), float(coefficients[2, 0])),
        (float(coefficients[0, 1]), float(coefficients[1, 1]), float(coefficients[2, 1])),
    )


def predict_xy(matrix: Sequence[Sequence[float]], u_px: float, v_px: float) -> Tuple[float, float]:
    """Apply a 2x3 pixel-to-arm matrix to one pixel center."""
    return (
        float(matrix[0][0]) * float(u_px) + float(matrix[0][1]) * float(v_px) + float(matrix[0][2]),
        float(matrix[1][0]) * float(u_px) + float(matrix[1][1]) * float(v_px) + float(matrix[1][2]),
    )


def _residuals(samples: Sequence[Sample],
               matrix: Sequence[Sequence[float]]) -> Tuple[float, ...]:
    values = []
    for sample in samples:
        x_mm, y_mm = predict_xy(matrix, sample.u_px, sample.v_px)
        values.append(math.hypot(x_mm - sample.x_mm, y_mm - sample.y_mm))
    return tuple(values)


def fit_pixel_to_arm(samples: Sequence[Sample],
                     min_inliers: int = 8,
                     max_residual_mm: float = 8.0,
                     condition_limit: float = 1e7) -> FitResult:
    """Fit x,y = M * [u,v,1] and reject collinear or grossly inconsistent data.

    A collinear pixel sweep (the legacy 20-point line) raises
    CalibrationGridError.  Gross outliers are removed iteratively with a
    median/MAD threshold; the final fit uses only inliers.
    """
    if len(samples) < min_inliers:
        raise CalibrationGridError("need at least {} samples".format(min_inliers))
    for index, sample in enumerate(samples):
        values = (sample.x_mm, sample.y_mm, sample.u_px, sample.v_px)
        if not all(math.isfinite(value) for value in values):
            raise CalibrationGridError("sample {} is not finite".format(index))
    design = _design(samples)
    if np.linalg.matrix_rank(design) < 3:
        raise CalibrationGridError("pixel samples are collinear; a 2D grid is required")
    condition = float(np.linalg.cond(design))
    if not math.isfinite(condition) or condition > condition_limit:
        raise CalibrationGridError("sample design is ill-conditioned: {:.3g}".format(condition))

    targets = _targets(samples)
    coefficients, _, _, _ = np.linalg.lstsq(design, targets, rcond=None)
    matrix = _matrix_from_coefficients(coefficients)
    residuals = _residuals(samples, matrix)
    inliers = set(range(len(samples)))
    outliers = set()

    for _ in range(3):
        current = [residuals[index] for index in sorted(inliers)]
        if len(current) < min_inliers:
            break
        median = float(np.median(current))
        mad = float(np.median([abs(value - median) for value in current]))
        threshold = max(float(max_residual_mm), median + 3.0 * 1.4826 * mad)
        rejected = {index for index in inliers if residuals[index] > threshold}
        if not rejected:
            break
        if len(inliers) - len(rejected) < min_inliers:
            break
        inliers -= rejected
        outliers |= rejected
        keep = sorted(inliers)
        design = _design([samples[index] for index in keep])
        if np.linalg.matrix_rank(design) < 3:
            raise CalibrationGridError("inlier design became collinear after outlier removal")
        coefficients, _, _, _ = np.linalg.lstsq(
            design, _targets([samples[index] for index in keep]), rcond=None)
        matrix = _matrix_from_coefficients(coefficients)
        residuals = _residuals(samples, matrix)

    inlier_indices = tuple(sorted(inliers))
    outlier_indices = tuple(sorted(outliers))
    if len(inlier_indices) < min_inliers:
        raise CalibrationGridError("too few inliers: {}".format(len(inlier_indices)))
    design = _design([samples[index] for index in inlier_indices])
    rank = int(np.linalg.matrix_rank(design))
    condition = float(np.linalg.cond(design))
    if rank < 3 or not math.isfinite(condition) or condition > condition_limit:
        raise CalibrationGridError("inlier design is not a usable 2D fit")
    inlier_residuals = [residuals[index] for index in inlier_indices]
    mean_residual = float(np.mean(inlier_residuals))
    max_residual = float(np.max(inlier_residuals))
    passed = max_residual <= float(max_residual_mm)
    reason = ("ok" if passed else
              "max inlier residual {:.3f} mm exceeds {:.3f} mm".format(
                  max_residual, float(max_residual_mm)))
    return FitResult(
        matrix=matrix,
        residuals_mm=residuals,
        inlier_indices=inlier_indices,
        outlier_indices=outlier_indices,
        rank=rank,
        condition=condition,
        mean_residual_mm=mean_residual,
        max_residual_mm=max_residual,
        passed=passed,
        reason=reason,
    )


def candidate_calibration(fit: FitResult,
                          samples: Sequence[Sample],
                          image_size: Tuple[int, int] = (640, 480),
                          margin_px: float = 5.0,
                          margin_mm: float = 5.0,
                          record: Optional[dict] = None) -> dict:
    """Build a disabled course candidate from a successful 2D fit.

    ``valid`` and ``calibration_mode`` are intentionally disabled.  The
    candidate still lacks z bounds, heights, drop pose and transport pose,
    so ``arm.validate_real_ready`` must reject it.
    """
    if fit.rank < 3:
        raise CalibrationGridError("cannot build a candidate from a rank-deficient fit")
    inliers = [samples[index] for index in fit.inlier_indices]
    if not inliers:
        raise CalibrationGridError("fit has no inliers")
    width, height = int(image_size[0]), int(image_size[1])
    us = [sample.u_px for sample in inliers]
    vs = [sample.v_px for sample in inliers]
    xs = [sample.x_mm for sample in inliers]
    ys = [sample.y_mm for sample in inliers]
    radii = [math.hypot(sample.x_mm, sample.y_mm) for sample in inliers]
    candidate = {
        "valid": False,
        "calibration_mode": "uncalibrated",
        "matrix": [[float(value) for value in row] for row in fit.matrix],
        "pixel_roi": {
            "u": [max(0.0, min(us) - margin_px), min(float(width - 1), max(us) + margin_px)],
            "v": [max(0.0, min(vs) - margin_px), min(float(height - 1), max(vs) + margin_px)],
        },
        "bounds_mm": {
            "x": [min(xs) - margin_mm, max(xs) + margin_mm],
            "y": [min(ys) - margin_mm, max(ys) + margin_mm],
            "radius": [min(radii) - margin_mm, max(radii) + margin_mm],
        },
        "heights_mm": {},
        "drop_mm": {},
        "transport_mm": {},
        "fit_record": {
            "sample_count": len(samples),
            "inlier_count": len(fit.inlier_indices),
            "outlier_indices": list(fit.outlier_indices),
            "rank": fit.rank,
            "condition": fit.condition,
            "mean_residual_mm": fit.mean_residual_mm,
            "max_residual_mm": fit.max_residual_mm,
            "fit_passed": fit.passed,
            "reason": fit.reason,
            "notes": [
                "Candidate only; valid=false.",
                "z bounds, pick/place heights, drop pose and transport pose are unset.",
                "Do not use for field motion until all parameters are measured and validated.",
            ],
        },
    }
    if record:
        candidate["fit_record"].update(record)
    return candidate


def candidate_yaml_text(candidate: dict) -> str:
    """Serialize a candidate with a prominent disabled-candidate header."""
    import yaml

    header = (
        "# Candidate only: 2D grid fit. valid=false.\n"
        "# z bounds, heights, drop pose and transport pose are intentionally unset.\n"
        "# Do not use this file for real arm motion.\n"
    )
    return header + yaml.safe_dump(candidate, default_flow_style=False, sort_keys=False)
