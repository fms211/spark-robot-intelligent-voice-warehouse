"""Mission transitions must not dispatch unsafe or stale work."""
import importlib
import math
from types import SimpleNamespace

import pytest


def mission_module():
    try:
        return importlib.import_module("spark_course_mission.mission")
    except ImportError as exc:
        pytest.fail("mission state machine is missing: {}".format(exc))


class Clock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class Nav:
    def __init__(self):
        self.goals = []
        self.rotations = []
        self.cancels = 0
        self.stops = 0

    def send_goal(self, pose):
        goal_id = "nav-{}".format(len(self.goals) + 1)
        self.goals.append((goal_id, pose))
        return goal_id

    def rotate_by(self, delta_rad):
        goal_id = "rot-{}".format(len(self.rotations) + 1)
        self.rotations.append((goal_id, delta_rad))
        return goal_id

    def cancel_goal(self):
        self.cancels += 1

    def stop(self):
        self.stops += 1


class Arm:
    def __init__(self):
        self.goals = []
        self.cancels = 0

    def send_goal(self, mode, station_id, center_u, center_v):
        self.goals.append((mode, station_id, center_u, center_v))

    def cancel_goal(self):
        self.cancels += 1


def inspection(station_id, found, stamp, center_u=320.0, center_v=240.0):
    return SimpleNamespace(station_id=station_id, found=found, stamp=stamp,
                           center_u=center_u, center_v=center_v)


def make_controller(ready=True, align=None, grasp_probe=None):
    module = mission_module()
    clock, nav, arm = Clock(), Nav(), Arm()
    statuses, announcements, stations = [], [], []
    config = {
        "valid_waypoints": True,
        "stations": [
            {"id": "patrol_a", "pose": "pose_a", "pickup_enabled": False},
            {"id": "pickup_b", "pose": "pose_b", "pickup_enabled": True},
        ],
        "drop_pose": "drop_pose",
        "home_pose": "home_pose",
        "nav_timeout_s": 10.0,
        "inspect_timeout_s": 3.0,
        "arm_timeout_s": 5.0,
        "stop_timeout_s": 2.0,
        "still_s": 0.5,
        "max_detection_age_s": 0.5,
    }
    if align is not None:
        config["align"] = align
    controller = module.MissionController(
        config, nav, arm, lambda: ready, statuses.append,
        announcements.append, stations.append, clock,
        grasp_probe=grasp_probe,
    )
    controller.on_base_velocity(0.0, 0.0)
    return controller, clock, nav, arm, statuses, announcements, stations


ALIGN = {
    "enabled": True,
    "window": {"u": [220.0, 420.0], "v": [140.0, 340.0]},
    "step_deg": 30.0,
    "fine_step_deg": 10.0,
    "max_rotation_deg": 360.0,
    "max_steps": 30,
    "step_timeout_s": 10.0,
    "total_timeout_s": 60.0,
}


def make_align_controller():
    return make_controller(align=dict(ALIGN))


def arrive_at_pickup(controller, clock, nav):
    controller.on_intent("START")
    controller.on_nav_done(nav.goals[-1][0], 3)
    controller.on_detection(inspection("patrol_a", False, clock()))
    controller.on_nav_done(nav.goals[-1][0], 3)
    controller.on_base_velocity(0.0, 0.0)
    clock.advance(0.6)


def to_pickup(controller, clock, nav):
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock()))


def test_two_station_pick_place_and_return_sequence():
    controller, clock, nav, arm, statuses, _, stations = make_controller()
    to_pickup(controller, clock, nav)
    assert arm.goals == [("PICK", "pickup_b", 320.0, 240.0)]
    assert stations[:3] == ["patrol_a", "", "pickup_b"]

    controller.on_arm_done(True, "COMMANDS_SENT_UNVERIFIED")
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", False, clock()))
    assert nav.goals[-1][1] == "drop_pose"

    controller.on_nav_done(nav.goals[-1][0], 3)
    controller.on_base_velocity(0.0, 0.0)
    clock.advance(0.6)
    controller.tick(clock())
    assert arm.goals[-1][0] == "PLACE"
    controller.on_arm_done(True, "COMMANDS_SENT_UNVERIFIED")
    assert nav.goals[-1][1] == "home_pose"
    controller.on_nav_done(nav.goals[-1][0], 3)
    assert controller.state == "COMPLETE"
    assert "COMPLETE" in statuses


def test_no_target_returns_home_without_arm_action():
    controller, clock, nav, arm, _, announcements, _ = make_controller()
    controller.on_intent("START")
    controller.on_nav_done(nav.goals[-1][0], 3)
    controller.on_detection(inspection("patrol_a", False, clock()))
    controller.on_nav_done(nav.goals[-1][0], 3)
    controller.on_detection(inspection("pickup_b", False, clock()))
    assert nav.goals[-1][1] == "home_pose"
    controller.on_nav_done(nav.goals[-1][0], 3)
    assert controller.state == "COMPLETE"
    assert arm.goals == []
    assert any("未发现" in text for text in announcements)


def test_navigation_abort_retries_once_then_fails_without_pick():
    controller, clock, nav, arm, _, _, _ = make_controller()
    controller.on_intent("START")
    first_goal = nav.goals[-1][0]
    controller.on_nav_done(first_goal, 4)
    assert len(nav.goals) == 2
    controller.on_nav_done(nav.goals[-1][0], 4)
    assert controller.state == "SAFE_STOP"
    clock.advance(0.6)
    controller.on_base_velocity(0.0, 0.0)
    controller.tick(clock())
    assert controller.state == "FAILED"
    assert arm.goals == []


def test_late_result_from_retried_goal_is_ignored():
    controller, _, nav, _, _, _, _ = make_controller()
    controller.on_intent("START")
    first_goal = nav.goals[-1][0]
    controller.on_nav_done(first_goal, 4)
    controller.on_nav_done(first_goal, 3)
    assert controller.state == "NAV_PATROL"
    assert len(nav.goals) == 2


@pytest.mark.parametrize("status", [2, 8])
def test_external_preempt_or_recall_never_reissues_navigation_goal(status):
    controller, _, nav, _, _, _, _ = make_controller()
    controller.on_intent("START")
    controller.on_nav_done(nav.goals[-1][0], status)
    assert len(nav.goals) == 1
    assert controller.state == "SAFE_STOP"


def test_navigation_timeout_retries_once_then_stops():
    controller, clock, nav, arm, _, _, _ = make_controller()
    controller.on_intent("START")
    clock.advance(10.1)
    controller.tick(clock())
    assert len(nav.goals) == 2
    clock.advance(10.1)
    controller.on_base_velocity(0.0, 0.0)
    controller.tick(clock())
    assert controller.state == "SAFE_STOP"
    assert arm.goals == []


def test_inspection_timeout_does_not_trigger_pick():
    controller, clock, nav, arm, _, _, _ = make_controller()
    controller.on_intent("START")
    controller.on_nav_done(nav.goals[-1][0], 3)
    controller.on_detection(inspection("wrong_station", True, clock()))
    clock.advance(3.1)
    controller.tick(clock())
    assert controller.state in ("SAFE_STOP", "FAILED")
    assert arm.goals == []


def test_arm_pick_failure_never_starts_delivery():
    controller, clock, nav, _, _, _, _ = make_controller()
    to_pickup(controller, clock, nav)
    controller.on_arm_done(False, "NO_PORT")
    assert controller.state == "SAFE_STOP"
    clock.advance(0.6)
    controller.on_base_velocity(0.0, 0.0)
    controller.tick(clock())
    assert controller.state == "FAILED"
    assert all(pose != "drop_pose" for _, pose in nav.goals)


def test_failed_precheck_and_repeated_start_do_not_dispatch_extra_goal():
    failed, _, nav, arm, _, _, _ = make_controller(ready=False)
    failed.on_intent("START")
    assert failed.state == "FAILED"
    assert nav.goals == [] and arm.goals == []

    running, _, running_nav, _, _, _, _ = make_controller()
    running.on_intent("START")
    running.on_intent("START")
    assert len(running_nav.goals) == 1


def test_stop_before_pick_waits_for_zero_base_velocity():
    controller, clock, nav, arm, _, _, _ = make_controller()
    controller.on_intent("START")
    controller.on_base_velocity(0.2, 0.0)
    controller.on_intent("STOP")
    assert controller.state == "SAFE_STOP"
    assert nav.cancels >= 1
    assert nav.stops >= 1
    controller.tick(clock())
    assert controller.state == "SAFE_STOP"
    assert nav.stops >= 2
    controller.on_base_velocity(0.0, 0.0)
    clock.advance(0.6)
    controller.tick(clock())
    assert controller.state == "CANCELED"
    assert arm.goals == []


def test_stop_while_carrying_blocks_new_mission():
    controller, clock, nav, arm, _, _, _ = make_controller()
    to_pickup(controller, clock, nav)
    controller.on_arm_done(True, "COMMANDS_SENT_UNVERIFIED")
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", False, clock()))
    controller.on_intent("STOP")
    assert arm.cancels >= 1
    clock.advance(0.6)
    controller.on_base_velocity(0.0, 0.0)
    controller.tick(clock())
    assert controller.state == "CANCELED"
    before = len(nav.goals)
    controller.on_intent("START")
    assert len(nav.goals) == before


def test_stop_during_pick_waits_for_arm_result_and_blocks_restarting_if_pump_on():
    controller, clock, nav, _, _, announcements, _ = make_controller()
    to_pickup(controller, clock, nav)
    controller.on_intent("STOP")
    clock.advance(0.6)
    controller.on_base_velocity(0.0, 0.0)
    controller.tick(clock())
    assert controller.state == "SAFE_STOP"
    controller.on_arm_done(False, "CANCELED_CARRYING", True)
    controller.tick(clock())
    assert controller.state == "CANCELED"
    assert controller.carrying is True
    before = len(nav.goals)
    controller.on_intent("START")
    assert len(nav.goals) == before
    assert any("人工" in text for text in announcements)


def test_missing_arm_cancel_result_fails_closed_and_blocks_restarting():
    controller, clock, nav, _, _, _, _ = make_controller()
    to_pickup(controller, clock, nav)
    controller.on_intent("STOP")
    clock.advance(2.1)
    controller.tick(clock())
    assert controller.state == "FAILED"
    assert controller.arm_state_uncertain is True
    before = len(nav.goals)
    controller.on_intent("START")
    assert len(nav.goals) == before


def test_real_pick_requires_stowed_arm_confirmation_before_driving():
    controller, clock, nav, _, _, _, _ = make_controller()
    controller.config["require_transport_confirmation"] = True
    to_pickup(controller, clock, nav)
    controller.on_arm_done(True, "COMMANDS_SENT_UNVERIFIED", True)
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", False, clock()))
    assert controller.state == "WAIT_TRANSPORT_CONFIRM"
    assert all(pose != "drop_pose" for _, pose in nav.goals)
    controller.confirm_transport()
    assert nav.goals[-1][1] == "drop_pose"


def test_stale_or_repeated_zero_odom_cannot_make_base_stationary():
    controller, clock, _, _, _, _, _ = make_controller()
    clock.advance(0.1)
    controller.on_base_velocity(0.2, 0.0, stamp=clock())
    clock.advance(0.1)
    assert controller.on_base_velocity(0.0, 0.0, stamp=clock() - 10) is False
    assert controller.stationary_since is None
    assert controller.on_base_velocity(0.0, 0.0, stamp=clock()) is True
    assert controller.on_base_velocity(0.0, 0.0, stamp=clock()) is False


def test_object_still_visible_after_pick_is_not_delivered():
    controller, clock, nav, arm, _, _, _ = make_controller()
    to_pickup(controller, clock, nav)
    controller.on_arm_done(True, "COMMANDS_SENT_UNVERIFIED")
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", True, clock()))
    controller.tick(clock())
    assert controller.state == "FAILED"
    assert all(pose != "drop_pose" for _, pose in nav.goals)


def test_object_moved_with_arm_after_pick_is_delivered():
    controller, clock, nav, arm, _, _, _ = make_controller()
    to_pickup(controller, clock, nav)
    controller.on_arm_done(True, "COMMANDS_SENT_UNVERIFIED")
    clock.advance(0.1)
    # The lifted block follows the arm to the transport pose, far from the
    # station pixel recorded before the pick.
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=240.0, center_v=230.0))
    assert controller.state == "NAV_DROP"
    assert nav.goals[-1][1] == "drop_pose"


def test_return_from_idle_uses_home_goal():
    controller, _, nav, _, _, _, _ = make_controller()
    controller.on_intent("RETURN")
    assert controller.state == "RETURN_HOME"
    assert nav.goals == [("nav-1", "home_pose")]
    controller.on_nav_done("nav-1", 3)
    assert controller.state == "COMPLETE"


@pytest.mark.parametrize("phase, elapsed_s", [("PICK", 44.0), ("PLACE", 47.0)])
def test_shipped_timeout_allows_measured_f600_arm_duration(phase, elapsed_s):
    from pathlib import Path
    import yaml
    field = yaml.safe_load((Path(__file__).parents[1] / "config" / "mission.yaml").read_text())
    controller, clock, nav, arm, _, _, _ = make_controller()
    controller.config["arm_timeout_s"] = field["arm_timeout_s"]
    to_pickup(controller, clock, nav)
    if phase == "PLACE":
        controller.on_arm_done(True, "COMMANDS_SENT_UNVERIFIED", True)
        clock.advance(0.1)
        controller.on_detection(inspection("pickup_b", False, clock()))
        controller.on_nav_done(nav.goals[-1][0], 3)
        clock.advance(0.6)
        controller.on_base_velocity(0.0, 0.0)
        controller.tick(clock())
    clock.advance(elapsed_s)
    controller.on_base_velocity(0.0, 0.0)
    controller.tick(clock())
    assert controller.state == phase
    assert arm.cancels == 0
    controller.on_arm_done(True, "COMMANDS_SENT_UNVERIFIED", phase == "PICK")
    assert controller.state == ("PICK" if phase == "PICK" else "RETURN_HOME")


def test_shipped_arm_deadline_still_cancels_an_overrun():
    from pathlib import Path
    import yaml
    field = yaml.safe_load((Path(__file__).parents[1] / "config" / "mission.yaml").read_text())
    controller, clock, nav, arm, _, _, _ = make_controller()
    controller.config["arm_timeout_s"] = field["arm_timeout_s"]
    to_pickup(controller, clock, nav)
    clock.advance(95.0)
    controller.on_base_velocity(0.0, 0.0)
    controller.tick(clock())
    assert controller.state == "SAFE_STOP"
    assert arm.cancels == 1
    assert controller.arm_state_uncertain is True


@pytest.mark.parametrize("key", ["home_pose", "drop_pose"])
@pytest.mark.parametrize("intent", ["START", "RETURN"])
def test_empty_home_or_drop_pose_never_dispatches_navigation(key, intent):
    controller, _, nav, arm, _, _, _ = make_controller()
    controller.config[key] = []
    controller.on_intent(intent)
    assert nav.goals == []
    assert arm.goals == []


def test_align_window_inside_picks_without_rotation():
    controller, clock, nav, arm, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=320.0, center_v=240.0))
    assert controller.state == "PICK"
    assert nav.rotations == []
    assert arm.goals == [("PICK", "pickup_b", 320.0, 240.0)]


def test_align_fine_step_when_target_outside_horizontal_window():
    controller, clock, nav, _, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=150.0, center_v=240.0))
    assert controller.state == "ALIGN"
    assert len(nav.rotations) == 1
    assert nav.rotations[0][1] == pytest.approx(math.radians(2.0))


def test_align_keeps_direction_when_error_shrinks():
    controller, clock, nav, _, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=150.0, center_v=240.0))
    controller.on_nav_done(nav.rotations[-1][0], 3)
    assert controller.state == "INSPECT"
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=170.0, center_v=240.0))
    assert controller.state == "ALIGN"
    assert nav.rotations[-1][1] == pytest.approx(math.radians(10.0))


def test_align_reverses_direction_when_error_grows():
    controller, clock, nav, _, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=150.0, center_v=240.0))
    controller.on_nav_done(nav.rotations[-1][0], 3)
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=130.0, center_v=240.0))
    assert controller.state == "ALIGN"
    assert nav.rotations[-1][1] == pytest.approx(math.radians(-10.0))


def test_align_picks_after_target_enters_window():
    controller, clock, nav, arm, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=150.0, center_v=240.0))
    controller.on_nav_done(nav.rotations[-1][0], 3)
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=300.0, center_v=240.0))
    assert controller.state == "PICK"
    assert arm.goals == [("PICK", "pickup_b", 300.0, 240.0)]


def test_align_inside_window_but_ungraspable_aims_at_grasp_column():
    calls = []

    def probe(u, v):
        calls.append((u, v))
        return ("ok", None) if u <= 300.0 else ("aim", 280.0)

    controller, clock, nav, arm, _, _, _ = make_controller(
        align=dict(ALIGN), grasp_probe=probe)
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=320.0, center_v=240.0))
    assert controller.state == "ALIGN"
    assert calls == [(320.0, 240.0)]
    assert len(nav.rotations) == 1
    assert nav.rotations[0][1] == pytest.approx(math.radians(2.0))
    assert arm.goals == []

    controller.on_nav_done(nav.rotations[-1][0], 3)
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=280.0, center_v=240.0))
    assert controller.state == "PICK"
    assert arm.goals == [("PICK", "pickup_b", 280.0, 240.0)]


def test_align_grasp_aims_proportionally_after_measuring_the_scale():
    def probe(u, v):
        return ("ok", None) if u <= 290.0 else ("aim", 250.0)

    controller, clock, nav, arm, _, _, _ = make_controller(
        align=dict(ALIGN), grasp_probe=probe)
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=320.0, center_v=240.0))
    assert nav.rotations[-1][1] == pytest.approx(math.radians(2.0))

    controller.on_nav_done(nav.rotations[-1][0], 3)
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=295.0, center_v=240.0))
    # 25 px over 2 deg = 12.5 px/deg, so 45 px to the target calls for 3.6 deg.
    assert nav.rotations[-1][1] == pytest.approx(math.radians(3.6))

    controller.on_nav_done(nav.rotations[-1][0], 3)
    clock.advance(0.1)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=250.0, center_v=240.0))
    assert controller.state == "PICK"
    assert arm.goals == [("PICK", "pickup_b", 250.0, 240.0)]


def test_align_ignores_tracker_decisions_before_the_settle_window():
    controller, clock, nav, arm, _, _, _ = make_controller(
        align=dict(ALIGN), grasp_probe=lambda u, v: ("ok", None))
    controller.config["align_settle_s"] = 0.8
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock()))
    assert controller.state == "INSPECT"
    assert arm.goals == []
    clock.advance(0.5)
    controller.on_base_velocity(0.0, 0.0)
    controller.on_detection(inspection("pickup_b", True, clock()))
    assert controller.state == "PICK"
    assert arm.goals == [("PICK", "pickup_b", 320.0, 240.0)]


def test_align_ungraspable_with_no_reachable_column_skips_station():
    controller, clock, nav, arm, _, announcements, _ = make_controller(
        align=dict(ALIGN), grasp_probe=lambda u, v: ("impossible", None))
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=320.0, center_v=240.0))
    assert controller.state == "RETURN_HOME"
    assert nav.rotations == []
    assert arm.goals == []
    assert any("可抓范围" in message for message in announcements)


def test_align_search_step_when_target_not_found():
    controller, clock, nav, _, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", False, clock()))
    assert controller.state == "ALIGN"
    assert nav.rotations[-1][1] == pytest.approx(math.radians(30.0))


def test_align_skips_station_when_only_vertical_window_violated():
    controller, clock, nav, _, _, announcements, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=320.0, center_v=100.0))
    assert controller.state == "RETURN_HOME"
    assert nav.rotations == []
    assert any("垂直" in message for message in announcements)


def test_align_full_rotation_gives_up_and_returns_home():
    controller, clock, nav, arm, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    for _ in range(12):
        controller.on_detection(inspection("pickup_b", False, clock()))
        assert controller.state == "ALIGN"
        controller.on_nav_done(nav.rotations[-1][0], 3)
        assert controller.state == "INSPECT"
    controller.on_detection(inspection("pickup_b", False, clock()))
    assert controller.state == "RETURN_HOME"
    assert arm.goals == []


def test_align_stop_cancels_rotation_and_stops_base():
    controller, clock, nav, _, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=150.0, center_v=240.0))
    assert controller.state == "ALIGN"
    controller.on_intent("STOP")
    assert controller.state == "SAFE_STOP"
    assert nav.cancels >= 1
    assert nav.stops >= 1
    controller.on_base_velocity(0.0, 0.0)
    clock.advance(0.6)
    controller.tick(clock())
    assert controller.state == "CANCELED"


def test_align_rotation_failure_retries_once_then_fails():
    controller, clock, nav, _, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=150.0, center_v=240.0))
    first = nav.rotations[-1]
    controller.on_nav_done(first[0], 4)
    assert controller.state == "ALIGN"
    assert len(nav.rotations) == 2
    assert nav.rotations[-1][1] == pytest.approx(first[1])
    controller.on_nav_done(nav.rotations[-1][0], 4)
    assert controller.state == "SAFE_STOP"


def test_align_total_timeout_gives_up():
    controller, clock, nav, _, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", True, clock(),
                                       center_u=150.0, center_v=240.0))
    controller.on_nav_done(nav.rotations[-1][0], 3)
    assert controller.state == "INSPECT"
    clock.advance(61.0)
    controller.tick(clock())
    assert controller.state == "RETURN_HOME"


def test_alignment_not_used_at_patrol_station():
    controller, clock, nav, _, _, _, _ = make_align_controller()
    controller.on_intent("START")
    controller.on_nav_done(nav.goals[-1][0], 3)
    controller.on_detection(inspection("patrol_a", False, clock()))
    assert nav.rotations == []
    assert controller.state == "NAV_PATROL"


def test_alignment_disabled_skips_search_on_no_target():
    controller, clock, nav, _, _, _, _ = make_controller()
    arrive_at_pickup(controller, clock, nav)
    controller.on_detection(inspection("pickup_b", False, clock()))
    assert nav.rotations == []
    assert controller.state == "RETURN_HOME"

@pytest.mark.parametrize('linear,angular', [(-36.04, 0.0), (0.0, float('nan')), (float('inf'), 0.0)])
def test_invalid_base_feedback_during_pick_cancels_and_latches(linear, angular):
    controller, clock, nav, arm, _, _, _ = make_controller()
    to_pickup(controller, clock, nav)
    before = (nav.cancels, nav.stops, arm.cancels)
    assert controller.on_base_velocity(linear, angular, stamp=clock()) is False
    assert controller.state == 'SAFE_STOP'
    assert (nav.cancels, nav.stops, arm.cancels) == tuple(x + 1 for x in before)
    assert controller.arm_state_uncertain
    controller.on_arm_done(False, 'CANCELED', False)
    clock.advance(0.1)
    controller.on_base_velocity(0.0, 0.0, stamp=clock())
    clock.advance(0.6)
    controller.on_base_velocity(0.0, 0.0, stamp=clock())
    controller.tick(clock())
    assert controller.state == 'FAILED'
    goals = len(nav.goals)
    controller.on_intent('START')
    assert len(nav.goals) == goals


def test_invalid_feedback_while_idle_blocks_start_and_return():
    controller, clock, nav, _, _, _, _ = make_controller()
    assert not controller.on_base_velocity(float('nan'), 0.0, stamp=clock())
    clock.advance(0.1)
    controller.on_base_velocity(0.0, 0.0, stamp=clock())
    controller.on_intent('START')
    controller.on_intent('RETURN')
    assert controller.state == 'IDLE'
    assert nav.goals == []


def test_normal_feedback_does_not_trip_odom_guard():
    controller, clock, nav, _, _, _, _ = make_controller()
    assert controller.on_base_velocity(0.2, 0.5, stamp=clock())
    controller.on_intent('START')
    assert controller.state == 'NAV_PATROL'
    assert len(nav.goals) == 1


def test_driver_fault_stops_active_mission_and_blocks_restart():
    controller, clock, nav, arm, _, _, _ = make_controller()
    to_pickup(controller, clock, nav)
    controller.on_odom_fault('ENCODER_CATASTROPHIC_DELTA')
    assert controller.state == 'SAFE_STOP'
    assert controller.odom_fault_latched
    assert (nav.cancels, nav.stops, arm.cancels) == (1, 1, 1)
    assert controller.arm_state_uncertain


def test_alignment_waits_for_fresh_pose_without_dispatching_or_consuming_step():
    controller, clock, nav, _, _, _, _ = make_align_controller()
    arrive_at_pickup(controller, clock, nav)
    ready = [False]
    nav.ensure_pose_ready = lambda: ready[0]
    controller.on_detection(inspection("pickup_b", False, clock()))
    assert controller.state == "INSPECT"
    assert nav.rotations == []
    assert controller.align_steps == 0
    ready[0] = True
    controller.on_detection(inspection("pickup_b", False, clock()))
    assert controller.state == "ALIGN"
    assert len(nav.rotations) == 1
    assert controller.align_steps == 1
