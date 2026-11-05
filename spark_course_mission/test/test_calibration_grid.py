"""2D calibration grid must reject line-only data and emit disabled candidates."""
import math

import numpy as np
import pytest
import yaml

from spark_course_mission.arm import validate_real_ready
from spark_course_mission.calibration_grid import (
    CalibrationGridError, GridPoint, Sample,
    candidate_calibration, candidate_yaml_text, default_grid_points,
    fit_pixel_to_arm, predict_xy, validate_grid_points,
)

TRUTH = (
    (0.02, -0.958, 489.86),
    (-0.821, -0.010, 290.02),
)


def _xy(u_px, v_px, matrix=TRUTH):
    return predict_xy(matrix, u_px, v_px)


def _samples(pixels, noise_mm=0.0, seed=0):
    rng = np.random.default_rng(seed)
    samples = []
    for u_px, v_px in pixels:
        x_mm, y_mm = _xy(u_px, v_px)
        if noise_mm:
            x_mm += float(rng.normal(0.0, noise_mm))
            y_mm += float(rng.normal(0.0, noise_mm))
        samples.append(Sample(x_mm, y_mm, float(u_px), float(v_px), area_px=500.0))
    return samples


def _grid_pixels():
    return [(u, v) for u in (120.0, 160.0, 200.0, 240.0, 280.0)
            for v in (240.0, 260.0, 280.0, 300.0, 320.0)]


def test_default_grid_is_2d_and_inside_envelope():
    points = default_grid_points()
    assert len(points) == 16
    validate_grid_points(points)
    design = np.array([[p.x_mm, p.y_mm, 1.0] for p in points], dtype=float)
    assert np.linalg.matrix_rank(design) == 3
    assert points[0] == GridPoint(180.0, 200.0, -110.0)


def test_default_grid_steps_stay_short():
    points = default_grid_points()
    for first, second in zip(points, points[1:]):
        distance = math.sqrt(
            (second.x_mm - first.x_mm) ** 2 +
            (second.y_mm - first.y_mm) ** 2 +
            (second.z_mm - first.z_mm) ** 2)
        assert distance <= 60.0


def test_collinear_grid_is_rejected():
    points = [GridPoint(180.0 + 5.0 * index, 200.0 - 10.0 * index, -110.0)
              for index in range(12)]
    with pytest.raises(CalibrationGridError):
        validate_grid_points(points)


def test_out_of_envelope_grid_is_rejected():
    points = [GridPoint(100.0, 100.0, -110.0), GridPoint(190.0, 180.0, -110.0),
              GridPoint(200.0, 190.0, -110.0), GridPoint(210.0, 200.0, -110.0)]
    with pytest.raises(CalibrationGridError):
        validate_grid_points(points)


def test_fit_recovers_a_known_affine_mapping():
    samples = _samples(_grid_pixels(), noise_mm=0.15, seed=3)
    fit = fit_pixel_to_arm(samples)
    assert fit.rank == 3
    assert fit.passed is True
    assert fit.max_residual_mm < 0.8
    for sample in samples:
        x_mm, y_mm = predict_xy(fit.matrix, sample.u_px, sample.v_px)
        assert math.hypot(x_mm - sample.x_mm, y_mm - sample.y_mm) <= fit.max_residual_mm + 1e-9


def test_collinear_pixel_samples_are_rejected():
    samples = _samples([(100.0 + 5.0 * index, 200.0 - 10.0 * index)
                        for index in range(12)])
    with pytest.raises(CalibrationGridError):
        fit_pixel_to_arm(samples)


def test_gross_outlier_is_removed():
    samples = list(_samples(_grid_pixels()[:12]))
    samples[5] = Sample(samples[5].x_mm + 50.0, samples[5].y_mm,
                        samples[5].u_px, samples[5].v_px, samples[5].area_px)
    fit = fit_pixel_to_arm(samples)
    assert 5 in fit.outlier_indices
    assert fit.passed is True
    assert fit.max_residual_mm < 0.5


def test_too_few_samples_are_rejected():
    with pytest.raises(CalibrationGridError):
        fit_pixel_to_arm(_samples(_grid_pixels()[:4]))


def test_candidate_is_disabled_and_rejected_by_arm():
    samples = _samples(_grid_pixels())
    fit = fit_pixel_to_arm(samples)
    candidate = candidate_calibration(fit, samples)
    assert candidate["valid"] is False
    assert candidate["calibration_mode"] == "uncalibrated"
    assert candidate["heights_mm"] == {}
    assert candidate["drop_mm"] == {}
    assert candidate["transport_mm"] == {}
    assert len(candidate["matrix"]) == 2
    assert all(len(row) == 3 for row in candidate["matrix"])
    assert validate_real_ready(candidate, port_exists=True) is False


def test_candidate_predictions_match_inliers():
    samples = _samples(_grid_pixels(), noise_mm=0.1, seed=7)
    fit = fit_pixel_to_arm(samples)
    candidate = candidate_calibration(fit, samples)
    for index in fit.inlier_indices:
        sample = samples[index]
        x_mm, y_mm = predict_xy(candidate["matrix"], sample.u_px, sample.v_px)
        assert math.hypot(x_mm - sample.x_mm, y_mm - sample.y_mm) <= fit.max_residual_mm + 1e-9


def test_candidate_yaml_round_trip():
    samples = _samples(_grid_pixels())
    fit = fit_pixel_to_arm(samples)
    candidate = candidate_calibration(fit, samples, record={"date": "2026-11-02"})
    loaded = yaml.safe_load(candidate_yaml_text(candidate))
    assert loaded["valid"] is False
    assert loaded["calibration_mode"] == "uncalibrated"
    assert loaded["fit_record"]["date"] == "2026-11-02"
    assert loaded["fit_record"]["rank"] == 3
    assert loaded["pixel_roi"]["u"][0] < loaded["pixel_roi"]["u"][1]
    assert loaded["bounds_mm"]["radius"][0] < loaded["bounds_mm"]["radius"][1]
