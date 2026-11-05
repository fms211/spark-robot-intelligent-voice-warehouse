"""Hardware-independent state machine for patrol and delivery."""
import math

from spark_course_mission.alignment import (
    DIRECTION_FLIP_MARGIN_PX, horizontal_error, inside,
)


NAV_SUCCESS = 3  # actionlib_msgs/GoalStatus.SUCCEEDED
NAV_PREEMPTED = 2
NAV_RECALLED = 8
NAV_ABORTED = 4


class MissionController:
    def __init__(self, config, nav, arm, precheck, status_sink,
                 announcement_sink, station_sink, clock, grasp_probe=None):
        self.config = config
        self.nav = nav
        self.arm = arm
        self.precheck = precheck
        self.status_sink = status_sink
        self.announcement_sink = announcement_sink
        self.station_sink = station_sink
        self.clock = clock
        self.state = "IDLE"
        self.station_index = 0
        self.nav_goal_id = None
        self.nav_pose = None
        self.nav_retries = 0
        self.deadline = None
        self.inspect_started = None
        self.pick_done_at = None
        self.pick_verify = False
        self.pick_verify_u = None
        self.pick_verify_v = None
        self.pick_verify_motion_px = float(
            config.get("pick_verify_motion_px", 25.0))
        self.grasp_probe = grasp_probe
        self.place_pending = False
        self.carrying = False
        self.no_target = False
        self.stop_target = None
        self.stop_deadline = None
        self.stop_reason = ""
        self.stationary_since = None
        self.last_velocity_at = None
        self.last_odom_stamp = None
        self.arm_in_flight = False
        self.arm_cancel_pending = False
        self.arm_state_uncertain = False
        self.odom_fault_latched = False
        align = config.get("align") or {}
        self.align = {
            "enabled": bool(align.get("enabled", False)),
            "window": align.get("window"),
            "step_deg": float(align.get("step_deg", 30.0)),
            "fine_step_deg": float(align.get("fine_step_deg", 10.0)),
            "max_rotation_deg": float(align.get("max_rotation_deg", 360.0)),
            "max_steps": int(align.get("max_steps", 30)),
            "step_timeout_s": float(align.get("step_timeout_s", 10.0)),
            "total_timeout_s": float(align.get("total_timeout_s", 60.0)),
        }
        self._reset_alignment()

    def _set_state(self, state):
        self.state = state
        self.status_sink(state)

    def _announce(self, message):
        self.announcement_sink(message)

    def _stationary(self, now):
        return (self.stationary_since is not None
                and self.last_velocity_at is not None
                and now - self.last_velocity_at <= float(
                    self.config.get("max_odom_age_s", 1.0))
                and now - self.stationary_since >= float(self.config["still_s"]))

    def on_base_velocity(self, linear: float, angular: float, stamp=None):
        now = self.clock()
        source_stamp = now if stamp is None else float(stamp)
        if (not math.isfinite(source_stamp) or source_stamp <= 0
                or now - source_stamp > float(self.config.get("max_odom_age_s", 1.0))
                or source_stamp - now > 0.1
                or (stamp is not None and self.last_odom_stamp is not None
                    and source_stamp <= self.last_odom_stamp)):
            return False
        # Grossly impossible feedback indicates that odometry cannot be used
        # to prove that the base is stationary. This is a last-resort guard,
        # not a replacement for checks before the driver publishes odometry.
        if (not math.isfinite(linear) or not math.isfinite(angular)
                or abs(linear) > 5.0 or abs(angular) > 10.0):
            self.stationary_since = None
            self.last_velocity_at = None
            self.odom_fault_latched = True
            if self.state not in ("IDLE", "COMPLETE", "FAILED", "CANCELED", "SAFE_STOP"):
                self._safe_stop("FAILED", "底盘里程计速度异常，需人工排查")
            return False
        if stamp is not None:
            self.last_odom_stamp = source_stamp
        self.last_velocity_at = source_stamp
        if abs(linear) <= 0.01 and abs(angular) <= 0.01:
            if self.stationary_since is None:
                self.stationary_since = source_stamp
        else:
            self.stationary_since = None
        return True

    def on_odom_fault(self, reason):
        self.stationary_since = None
        self.last_velocity_at = None
        self.odom_fault_latched = True
        if self.state not in ("IDLE", "COMPLETE", "FAILED", "CANCELED", "SAFE_STOP"):
            self._safe_stop("FAILED", "底盘里程计故障：{}".format(reason))

    def _align_active(self):
        return self.align["enabled"] and self.align["window"] is not None

    def _reset_alignment(self):
        self.align_steps = 0
        self.align_rotation_deg = 0.0
        self.align_direction = 1.0
        self.align_last_error_px = None
        self.align_px_per_deg = None
        self.align_last_u = None
        self.align_last_delta_deg = 0.0
        self.align_started_at = None
        self.align_exhausted = False
        self.align_retry_used = False
        self.align_last_delta_deg = 0.0

    def _enter_inspect(self):
        self._set_state("INSPECT")
        self.inspect_started = self.clock()
        self.deadline = self.clock() + float(self.config["inspect_timeout_s"])

    def _advance_station(self, reason):
        self._announce(reason)
        self.station_sink("")
        self.station_index += 1
        self._start_patrol()

    def _start_rotate(self, delta_deg, count=True):
        now = self.clock()
        ensure_pose = getattr(self.nav, "ensure_pose_ready", None)
        if ensure_pose is not None and not ensure_pose():
            # Remain in INSPECT and retry on a new image, bounded by its deadline.
            # A failed rotation retry also returns here after canceling its goal.
            if self.state == "ALIGN":
                self._set_state("INSPECT")
                self.inspect_started = now
                self.deadline = now + float(self.config["inspect_timeout_s"])
            return
        if count:
            self.align_steps += 1
            self.align_rotation_deg += abs(float(delta_deg))
        self.align_last_delta_deg = float(delta_deg)
        self._set_state("ALIGN")
        self.deadline = now + float(self.align["step_timeout_s"])
        try:
            goal_id = self.nav.rotate_by(math.radians(float(delta_deg)))
        except Exception:
            goal_id = None
        if not goal_id:
            self._safe_stop("FAILED", "对准旋转不可用（定位无效）")
            return
        self.nav_goal_id = goal_id

    def _learn_align_scale(self, u):
        """Estimate the signed pixel shift per degree from the last rotation."""
        if self.align_last_u is None or abs(self.align_last_delta_deg) < 1.0:
            return
        measured = ((float(u) - self.align_last_u) / self.align_last_delta_deg)
        if not math.isfinite(measured) or abs(measured) < 0.5:
            return
        if self.align_px_per_deg is None:
            self.align_px_per_deg = measured
        else:
            self.align_px_per_deg = 0.5 * (self.align_px_per_deg + measured)

    def _align_delta(self, u, target_u, probe_step_deg):
        """Aim one rotation at target_u, proportional once the scale is known."""
        self._learn_align_scale(u)
        limit = float(self.align["fine_step_deg"])
        if self.align_px_per_deg is not None and abs(self.align_px_per_deg) >= 0.5:
            delta = (float(target_u) - float(u)) / self.align_px_per_deg
            delta = max(-limit, min(limit, delta))
            smallest = min(2.0, limit)
            if abs(delta) < smallest:
                delta = math.copysign(smallest, delta)
            self.align_direction = 1.0 if self.align_px_per_deg >= 0.0 else -1.0
            return delta
        return self.align_direction * float(probe_step_deg)

    def _start_align_fine(self, result):
        if self.align_started_at is None:
            self.align_started_at = self.clock()
        u = float(result.center_u)
        error = horizontal_error(self.align["window"], u)
        if self.align_px_per_deg is None:
            # Discover the rotation direction with the original heuristic
            # until a pixel-per-degree measurement exists.
            if (self.align_last_error_px is not None
                    and error > self.align_last_error_px + DIRECTION_FLIP_MARGIN_PX):
                self.align_direction = -self.align_direction
            self.align_last_error_px = error
        u_low, u_high = self.align["window"]["u"]
        # A full fine step can jump a close target across the whole ROI, so
        # probe with 2 deg and let the measured scale aim the next step.
        probe_step = min(float(self.align["fine_step_deg"]), 2.0)
        delta = self._align_delta(
            u, 0.5 * (float(u_low) + float(u_high)), probe_step)
        self.align_last_u = u
        self.align_last_delta_deg = delta
        self._start_rotate(delta)

    def _start_align_grasp(self, result, target_u):
        if self.align_started_at is None:
            self.align_started_at = self.clock()
        u = float(result.center_u)
        # The graspable pixel band can be narrower than a full fine step, so
        # probe with 2 deg and let the measured scale aim the next one.
        probe_step = min(float(self.align["fine_step_deg"]), 2.0)
        delta = self._align_delta(u, float(target_u), probe_step)
        self.align_last_u = u
        self.align_last_delta_deg = delta
        self._start_rotate(delta)

    def _start_align_search(self):
        if self.align_started_at is None:
            self.align_started_at = self.clock()
        self.align_last_error_px = None
        self.align_last_u = None
        self._start_rotate(self.align_direction * float(self.align["step_deg"]))

    def _on_alignment_detection(self, result):
        window = self.align["window"]
        if not result.found:
            if self.align_exhausted:
                self._advance_station("对准未成功，前往下一站")
            else:
                self._start_align_search()
            return True
        if inside(window, result.center_u, result.center_v):
            if self.grasp_probe is None:
                return False
            status, target_u = self.grasp_probe(
                float(result.center_u), float(result.center_v))
            if status == "ok":
                return False
            if status != "aim" or target_u is None:
                self._advance_station("目标不在机械臂可抓范围，跳过本站")
                return True
            if self.align_exhausted:
                self._advance_station("对准未成功，前往下一站")
                return True
            self._start_align_grasp(result, float(target_u))
            return True
        u_low, u_high = window["u"]
        if u_low <= float(result.center_u) <= u_high:
            self._advance_station("目标超出可抓窗口的垂直范围，跳过本站")
            return True
        if self.align_exhausted:
            self._advance_station("对准未成功，前往下一站")
            return True
        self._start_align_fine(result)
        return True

    def _precheck_ok(self):
        return (self.config.get("valid_waypoints") is True
                and bool(self.config.get("stations"))
                and bool(self.config.get("drop_pose"))
                and bool(self.config.get("home_pose"))
                and bool(self.precheck()))

    def on_intent(self, intent: str):
        if intent == "STATUS":
            self._announce("当前任务状态：{}".format(self.state))
            return
        if intent == "STOP":
            if self.state not in ("IDLE", "COMPLETE", "FAILED", "CANCELED", "SAFE_STOP"):
                self._safe_stop("CANCELED", "用户停止任务")
            return
        if intent == "START":
            if (self.state not in ("IDLE", "COMPLETE", "FAILED", "CANCELED")
                    or self.carrying or self.arm_state_uncertain or self.odom_fault_latched):
                return
            self.station_index = 0
            self.no_target = False
            self.pick_verify = False
            self.place_pending = False
            self._set_state("PRECHECK")
            if not self._precheck_ok():
                self._set_state("FAILED")
                self._announce("巡检条件未满足，任务未启动")
                return
            self._start_patrol()
            return
        if intent == "RETURN" and self.state in ("IDLE", "COMPLETE", "FAILED", "CANCELED"):
            if self.carrying or self.arm_state_uncertain or self.odom_fault_latched:
                return
            if not self._precheck_ok():
                self._set_state("FAILED")
                self._announce("返航条件未满足")
                return
            self._start_nav(self.config["home_pose"], "RETURN_HOME")

    def _start_nav(self, pose, state):
        self.nav_pose = pose
        self.nav_retries = 0
        self._set_state(state)
        self.nav_goal_id = self.nav.send_goal(pose)
        self.deadline = self.clock() + float(self.config["nav_timeout_s"])

    def _retry_nav_or_stop(self, reason):
        if self.nav_retries < 1:
            self.nav_retries += 1
            self.nav.cancel_goal()
            self.nav_goal_id = self.nav.send_goal(self.nav_pose)
            self.deadline = self.clock() + float(self.config["nav_timeout_s"])
        else:
            self._safe_stop("FAILED", reason)

    def _start_patrol(self):
        if self.station_index >= len(self.config["stations"]):
            self.no_target = True
            self._announce("巡检结束，未发现目标")
            self._start_nav(self.config["home_pose"], "RETURN_HOME")
            return
        self._start_nav(self.config["stations"][self.station_index]["pose"],
                        "NAV_PATROL")

    def on_nav_done(self, goal_id: str, status: int):
        if self.state == "ALIGN":
            if goal_id != self.nav_goal_id:
                return
            if status == NAV_SUCCESS:
                self.nav_goal_id = None
                self.deadline = None
                self.align_retry_used = False
                if (self.align_steps >= int(self.align["max_steps"])
                        or self.align_rotation_deg >= float(
                            self.align["max_rotation_deg"])):
                    self.align_exhausted = True
                self._enter_inspect()
            elif status in (NAV_PREEMPTED, NAV_RECALLED):
                self._safe_stop("FAILED", "对准旋转被取消或抢占")
            elif not self.align_retry_used:
                self.align_retry_used = True
                self._start_rotate(self.align_last_delta_deg, count=False)
            else:
                self._safe_stop("FAILED", "对准旋转失败")
            return
        if self.state not in ("NAV_PATROL", "NAV_DROP", "RETURN_HOME"):
            return
        if goal_id != self.nav_goal_id:
            return
        if status != NAV_SUCCESS:
            if status in (NAV_PREEMPTED, NAV_RECALLED):
                self._safe_stop("FAILED", "导航目标被外部取消或抢占")
            elif status == NAV_ABORTED:
                self._retry_nav_or_stop("导航目标失败")
            else:
                self._safe_stop("FAILED", "导航目标状态异常：{}".format(status))
            return
        self.nav_goal_id = None
        self.deadline = None
        if self.state == "NAV_PATROL":
            station = self.config["stations"][self.station_index]
            self._reset_alignment()
            self._set_state("INSPECT")
            self.station_sink(station["id"])
            self.inspect_started = self.clock()
            self.deadline = self.clock() + float(self.config["inspect_timeout_s"])
        elif self.state == "NAV_DROP":
            self._set_state("PLACE")
            self.place_pending = True
            self.deadline = self.clock() + float(self.config["arm_timeout_s"])
        elif self.state == "RETURN_HOME":
            self._set_state("COMPLETE")
            if self.no_target:
                self._announce("巡检完成，未发现目标")
            else:
                self._announce("任务流程完成，请核对实际取放结果")

    def _fresh_detection(self, result, now):
        try:
            stamp = float(result.stamp)
        except (AttributeError, TypeError, ValueError):
            return False
        return (stamp <= now
                and now - stamp <= float(self.config["max_detection_age_s"]))

    def on_detection(self, result):
        now = self.clock()
        if not self._fresh_detection(result, now):
            return
        if self.state == "INSPECT":
            station = self.config["stations"][self.station_index]
            if result.station_id != station["id"] or result.stamp < self.inspect_started:
                return
            if station.get("pickup_enabled"):
                settle = float(self.config.get("align_settle_s", 0.0))
                if result.stamp < self.inspect_started + settle:
                    return
            if station.get("pickup_enabled") and self._align_active():
                if self._on_alignment_detection(result):
                    return
            if result.found and station.get("pickup_enabled"):
                if not self._stationary(now):
                    return
                self.station_sink("")
                self.pick_verify_u = float(result.center_u)
                self.pick_verify_v = float(result.center_v)
                self.arm.send_goal("PICK", station["id"], result.center_u,
                                   result.center_v)
                self.arm_in_flight = True
                self._set_state("PICK")
                self.pick_verify = False
                self.deadline = now + float(self.config["arm_timeout_s"])
                return
            self.station_sink("")
            self.station_index += 1
            self._start_patrol()
        elif self.state == "PICK" and self.pick_verify:
            station = self.config["stations"][self.station_index]
            if result.station_id != station["id"] or result.stamp <= self.pick_done_at:
                return
            if result.found:
                if self.pick_verify_u is None or self.pick_verify_v is None:
                    moved_px = 0.0
                else:
                    moved_px = math.hypot(
                        float(result.center_u) - self.pick_verify_u,
                        float(result.center_v) - self.pick_verify_v)
                if moved_px <= self.pick_verify_motion_px:
                    self._safe_stop("FAILED", "取物后目标仍在原位，结果不确定")
                    return
                # The target followed the arm away from its station pixel:
                # treat the pick as visually confirmed enough to continue.
            self.station_sink("")
            if self.config.get("require_transport_confirmation", False):
                self._set_state("WAIT_TRANSPORT_CONFIRM")
                self.deadline = now + float(self.config.get(
                    "transport_confirmation_timeout_s", 30.0))
                self._announce("请现场确认机械臂已收拢，再允许底盘移动")
            else:
                self._start_nav(self.config["drop_pose"], "NAV_DROP")

    def confirm_transport(self):
        if (self.state != "WAIT_TRANSPORT_CONFIRM" or not self.carrying
                or self.arm_state_uncertain or not self._stationary(self.clock())):
            return False
        self._start_nav(self.config["drop_pose"], "NAV_DROP")
        return True

    def on_arm_done(self, success: bool, code: str, carrying=None):
        if self.state == "SAFE_STOP" and self.arm_cancel_pending:
            self.arm_in_flight = False
            self.arm_cancel_pending = False
            self.arm_state_uncertain = carrying is None
            self.carrying = bool(carrying) if carrying is not None else True
            if self.carrying:
                self._announce("吸泵可能仍开启，请人工检查后重启任务")
            return
        if self.state == "PICK" and not self.pick_verify:
            self.arm_in_flight = False
            if not success:
                self.carrying = bool(carrying) if carrying is not None else True
                self.arm_state_uncertain = carrying is None
                self._safe_stop("FAILED", "机械臂取物指令失败：{}".format(code))
                return
            self.carrying = bool(carrying) if carrying is not None else True
            self.pick_verify = True
            self.pick_done_at = self.clock()
            self.station_sink(self.config["stations"][self.station_index]["id"])
            self.deadline = self.clock() + float(self.config["inspect_timeout_s"])
        elif self.state == "PLACE" and not self.place_pending:
            self.arm_in_flight = False
            if not success:
                if carrying is not None:
                    self.carrying = bool(carrying)
                self._safe_stop("FAILED", "机械臂放置指令失败：{}".format(code))
                return
            self.carrying = bool(carrying) if carrying is not None else False
            self._start_nav(self.config["home_pose"], "RETURN_HOME")

    def _safe_stop(self, target, reason):
        if self.state == "SAFE_STOP":
            return
        self.station_sink("")
        self.arm_cancel_pending = self.arm_in_flight
        if self.arm_cancel_pending:
            self.arm_state_uncertain = True
        self.nav.cancel_goal()
        self.nav.stop()
        self.arm.cancel_goal()
        self.stop_target = target
        self.stop_reason = reason
        self.stop_deadline = self.clock() + float(self.config["stop_timeout_s"])
        self.deadline = None
        self._set_state("SAFE_STOP")
        if self.carrying or self.arm_state_uncertain:
            self._announce("任务停止，吸泵状态需确认，请人工接管")

    def tick(self, now: float):
        if self.state == "SAFE_STOP":
            self.nav.stop()
            if not self.arm_cancel_pending and self._stationary(now):
                self._set_state(self.stop_target)
                self._announce(self.stop_reason)
            elif now >= self.stop_deadline:
                self._set_state("FAILED")
                self._announce("停止超时，底盘速度未确认归零")
            return
        if self.state == "PLACE" and self.place_pending:
            if self._stationary(now):
                station = self.config["stations"][self.station_index]
                self.arm.send_goal("PLACE", station["id"], 0.0, 0.0)
                self.arm_in_flight = True
                self.place_pending = False
                self.deadline = now + float(self.config["arm_timeout_s"])
            elif now >= self.deadline:
                self._safe_stop("FAILED", "放置前底盘未静止")
            return
        if (self._align_active()
                and self.align_started_at is not None
                and self.state in ("INSPECT", "ALIGN")
                and now - self.align_started_at > float(
                    self.align["total_timeout_s"])):
            self.nav.cancel_goal()
            self._advance_station("对准超时，前往下一站")
            return
        if self.deadline is None or now < self.deadline:
            return
        if self.state == "ALIGN":
            if not self.align_retry_used:
                self.align_retry_used = True
                self.nav.cancel_goal()
                self._start_rotate(self.align_last_delta_deg, count=False)
            else:
                self._safe_stop("FAILED", "对准旋转超时")
            return
        if self.state in ("NAV_PATROL", "NAV_DROP", "RETURN_HOME"):
            self._retry_nav_or_stop("导航目标超时")
        elif self.state == "INSPECT":
            self._safe_stop("FAILED", "巡检图像超时")
        elif self.state == "PICK":
            message = "取物视觉复核超时" if self.pick_verify else "机械臂取物超时"
            self._safe_stop("FAILED", message)
        elif self.state == "PLACE":
            self._safe_stop("FAILED", "机械臂放置超时")
        elif self.state == "WAIT_TRANSPORT_CONFIRM":
            self._safe_stop("FAILED", "机械臂收拢姿态未确认")
