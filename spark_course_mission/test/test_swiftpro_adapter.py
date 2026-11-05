"""SwiftPro plans stay inside calibrated bounds and preserve suction on stop."""
import importlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


def arm_module():
    try:
        return importlib.import_module("spark_course_mission.arm")
    except ImportError as exc:
        pytest.fail("SwiftPro planning module is missing: {}".format(exc))


CALIBRATION = {
    "valid": True,
    "calibration_mode": "field",
    "matrix": [[0.0, -0.5, 200.0], [0.5, 0.0, -100.0]],
    "pixel_roi": {"u": [0.0, 640.0], "v": [0.0, 480.0]},
    "heights_mm": {"approach": 80.0, "pick": -30.0,
                   "transport": 110.0, "place": -20.0},
    "drop_mm": {"x": 180.0, "y": 0.0, "z": -20.0},
    "transport_mm": {"x": 160.0, "y": 0.0, "z": 110.0},
    "bounds_mm": {"x": [100.0, 250.0], "y": [-150.0, 150.0],
                  "z": [-50.0, 130.0], "radius": [50.0, 300.0]},
    "dwell_s": 0.1,
}


def test_calibrated_pixel_maps_to_arm_millimeters():
    assert arm_module().pixel_to_arm(100.0, 200.0, CALIBRATION) == (100.0, -50.0)


def test_grasp_verdict_accepts_a_pixel_inside_the_workspace():
    assert arm_module().grasp_verdict(100.0, 200.0, CALIBRATION) == ("ok", None)


def test_grasp_verdict_aims_at_the_nearest_reachable_column():
    status, target_u = arm_module().grasp_verdict(600.0, 40.0, CALIBRATION)
    assert status == "aim"
    assert 0.0 <= target_u <= 500.0
    assert arm_module().grasp_verdict(target_u, 40.0, CALIBRATION) == ("ok", None)


def test_grasp_verdict_limits_aim_to_the_align_window():
    status, target_u = arm_module().grasp_verdict(
        600.0, 40.0, CALIBRATION, u_limits=(480.0, 640.0))
    assert status == "aim"
    assert target_u == pytest.approx(490.0)


def test_grasp_verdict_reports_an_unreachable_row_or_roi():
    arm = arm_module()
    assert arm.grasp_verdict(100.0, 480.0, CALIBRATION) == ("impossible", None)
    assert arm.grasp_verdict(100.0, 500.0, CALIBRATION) == ("impossible", None)


def test_missing_calibration_or_port_rejects_real_mode():
    arm = arm_module()
    assert arm.validate_real_ready(CALIBRATION, port_exists=True) is True
    assert arm.validate_real_ready(CALIBRATION, port_exists=False) is False
    assert arm.validate_real_ready(dict(CALIBRATION, valid=False), port_exists=True) is False
    assert arm.validate_real_ready(dict(CALIBRATION, calibration_mode="mock"),
                                   port_exists=True) is False
    assert arm.validate_real_ready(dict(CALIBRATION, pixel_roi={}),
                                   port_exists=True) is False
    bad_bounds = dict(CALIBRATION["bounds_mm"], radius=[50.0, 321.0])
    assert arm.validate_real_ready(dict(CALIBRATION, bounds_mm=bad_bounds),
                                   port_exists=True) is False
    with pytest.raises(arm.CalibrationError):
        arm.pixel_to_arm(100, 200, dict(CALIBRATION, valid=False))


def test_real_pixel_mapping_rejects_extrapolation_outside_roi():
    arm = arm_module()
    with pytest.raises(arm.ArmBoundsError, match="pixel outside calibrated ROI"):
        arm.pixel_to_arm(641.0, 200.0, CALIBRATION)


def test_real_target_rejects_radial_overshoot_even_inside_axis_bounds():
    arm = arm_module()
    bounds = dict(CALIBRATION["bounds_mm"], x=[100.0, 300.0],
                  y=[-300.0, 300.0], radius=[50.0, 250.0])
    with pytest.raises(arm.ArmBoundsError, match="radial target"):
        arm.plan_pick(230.0, 150.0, CALIBRATION["heights_mm"], bounds,
                      CALIBRATION["transport_mm"])


def test_out_of_range_target_is_rejected_before_any_step():
    arm = arm_module()
    with pytest.raises(arm.ArmBoundsError):
        arm.plan_pick(300.0, 0.0, CALIBRATION["heights_mm"],
                      CALIBRATION["bounds_mm"], CALIBRATION["transport_mm"])


def test_pick_sequence_approaches_suctions_and_retracts():
    arm = arm_module()
    steps = arm.plan_pick(150.0, -20.0, CALIBRATION["heights_mm"],
                          CALIBRATION["bounds_mm"], CALIBRATION["transport_mm"])
    assert [(step.phase, step.x_mm, step.y_mm, step.z_mm, step.pump)
            for step in steps] == [
                ("PICK_TRANSFER", 150.0, -20.0, 110.0, None),
                ("APPROACH", 150.0, -20.0, 80.0, None),
                ("LOWER", 150.0, -20.0, -30.0, None),
                ("SUCTION_ON", None, None, None, True),
                ("LIFT", 150.0, -20.0, 80.0, None),
                ("CLEARANCE", 150.0, -20.0, 110.0, None),
                ("TRANSPORT", 160.0, 0.0, 110.0, None),
            ]


def test_place_sequence_releases_only_at_drop_pose():
    arm = arm_module()
    steps = arm.plan_place(CALIBRATION["drop_mm"], CALIBRATION["heights_mm"],
                           CALIBRATION["bounds_mm"], CALIBRATION["transport_mm"])
    assert [step.phase for step in steps] == [
        "PLACE_TRANSFER", "PLACE_APPROACH", "PLACE_LOWER",
        "SUCTION_OFF", "PLACE_RETRACT", "PLACE_CLEARANCE", "PLACE_STOW"
    ]
    assert steps[0].z_mm == 110.0
    assert steps[2].z_mm == -20.0
    assert steps[3].pump is False
    assert (steps[-2].x_mm, steps[-2].y_mm, steps[-2].z_mm) == (180.0, 0.0, 110.0)
    assert (steps[-1].x_mm, steps[-1].y_mm, steps[-1].z_mm) == (160.0, 0.0, 110.0)


def test_cancel_before_suction_does_not_activate_pump():
    arm = arm_module()
    steps = arm.plan_pick(150, -20, CALIBRATION["heights_mm"],
                          CALIBRATION["bounds_mm"], CALIBRATION["transport_mm"])
    sent = []
    outcome = arm.run_steps(steps, sent.append, lambda: len(sent) >= 2,
                            lambda seconds: None)
    assert outcome.code == "CANCELED"
    assert outcome.carrying is False
    assert [step.phase for step in sent] == ["PICK_TRANSFER", "APPROACH"]


def test_cancel_while_carrying_keeps_pump_without_new_motion():
    arm = arm_module()
    steps = arm.plan_pick(150, -20, CALIBRATION["heights_mm"],
                          CALIBRATION["bounds_mm"], CALIBRATION["transport_mm"])
    sent = []
    outcome = arm.run_steps(
        steps, sent.append,
        lambda: any(step.pump is True for step in sent),
        lambda seconds: None,
    )
    assert outcome.code == "CANCELED_CARRYING"
    assert outcome.carrying is True
    assert [step.phase for step in sent] == ["PICK_TRANSFER", "APPROACH", "LOWER", "SUCTION_ON"]
    assert not any(step.pump is False for step in sent)


def test_real_calibration_rejects_transport_below_approach():
    arm = arm_module()
    heights = dict(CALIBRATION["heights_mm"], transport=60.0)
    assert arm.validate_real_ready(dict(CALIBRATION, heights_mm=heights), True) is False
    with pytest.raises(arm.CalibrationError, match="heights must ascend"):
        arm.plan_pick(150, -20, heights, CALIBRATION["bounds_mm"],
                      CALIBRATION["transport_mm"])


def adapter_module():
    spec = importlib.util.spec_from_file_location(
        "swiftpro_adapter_script",
        Path(__file__).parents[1] / "scripts" / "swiftpro_adapter.py",
    )
    script = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(script)
    return script


def test_real_adapter_rejects_writer_disconnect_before_position_command():
    arm = arm_module()
    script = adapter_module()

    class Server:
        def publish_feedback(self, feedback):
            pass

    adapter = script.SwiftproAdapter.__new__(script.SwiftproAdapter)
    adapter.mode = "real"
    adapter.writer_connected = lambda: False
    adapter.move_client = lambda *args: pytest.fail("disconnected writer called")
    adapter.server = Server()
    adapter.sent_steps = []
    with pytest.raises(arm.DriverUnavailableError):
        adapter.send_step(arm.ArmStep("APPROACH", 150, 0, 80))


def test_real_adapter_rejects_firmware_e22_before_pump():
    arm = arm_module()
    script = adapter_module()

    class Server:
        def publish_feedback(self, feedback):
            pass

    adapter = script.SwiftproAdapter.__new__(script.SwiftproAdapter)
    adapter.mode = "real"
    adapter.writer_connected = lambda: True
    adapter.move_client = lambda *args: SimpleNamespace(
        success=False, code="E22", detail="address out of range")
    adapter.pump_client = lambda *args: pytest.fail("pump called after rejected move")
    adapter.server = Server()
    adapter.sent_steps = []
    adapter.carrying = False
    with pytest.raises(arm.DriverUnavailableError, match="E22"):
        adapter.send_step(arm.ArmStep("APPROACH", 91.48, 7.03, 23.95))
    assert adapter.carrying is False


def test_lost_pump_on_reply_keeps_carrying_state_conservative():
    arm = arm_module()
    script = adapter_module()

    class Server:
        def publish_feedback(self, feedback):
            pass

    adapter = script.SwiftproAdapter.__new__(script.SwiftproAdapter)
    adapter.mode = "real"
    adapter.writer_connected = lambda: True
    adapter.pump_client = lambda *args: SimpleNamespace(
        success=False, code="SERIAL_ERROR", detail="USB disconnected")
    adapter.server = Server()
    adapter.sent_steps = []
    adapter.carrying = False
    with pytest.raises(arm.DriverUnavailableError, match="SERIAL_ERROR"):
        adapter.send_step(arm.ArmStep("SUCTION_ON", pump=True))
    assert adapter.carrying is True


def test_place_rejects_low_stow_before_releasing_object():
    arm = arm_module()
    with pytest.raises(arm.CalibrationError, match="transport pose height"):
        arm.plan_place(CALIBRATION["drop_mm"], CALIBRATION["heights_mm"],
                       CALIBRATION["bounds_mm"], {"x": 160, "y": 0, "z": 70})
