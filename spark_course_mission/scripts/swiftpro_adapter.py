#!/usr/bin/env python3
"""Action server using acknowledged SwiftPro command services."""
import os

import actionlib
import rospy
from swiftpro.srv import Move, SetPump

from spark_course_mission.arm import (
    ArmBoundsError, CalibrationError, DriverUnavailableError,
    pixel_to_arm, plan_pick, plan_place, run_steps, validate_real_ready,
)
from spark_course_mission.msg import (
    PickPlaceAction, PickPlaceFeedback, PickPlaceGoal, PickPlaceResult,
)


class SwiftproAdapter:
    def __init__(self):
        self.mode = rospy.get_param("~arm_mode", "mock")
        self.calibration = rospy.get_param("~calibration", {})
        self.port = rospy.get_param("~port", "/dev/uarm")
        if self.mode not in ("mock", "real"):
            raise ValueError("arm_mode must be mock or real")
        if self.mode == "real" and not validate_real_ready(
                self.calibration, os.path.exists(self.port)):
            raise CalibrationError("real arm requires connected port and field calibration")
        self.carrying = False
        self.sent_steps = []
        self.move_client = None
        self.pump_client = None
        if self.mode == "real":
            self.move_client = rospy.ServiceProxy("swiftpro_move", Move)
            self.pump_client = rospy.ServiceProxy("swiftpro_set_pump", SetPump)
            if not self.writer_connected():
                raise DriverUnavailableError("SwiftPro acknowledged command services unavailable")
        self.server = actionlib.SimpleActionServer(
            "/mission/pick_place", PickPlaceAction, execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()

    def writer_connected(self):
        try:
            rospy.wait_for_service("swiftpro_move", timeout=5.0)
            rospy.wait_for_service("swiftpro_set_pump", timeout=5.0)
            return True
        except (rospy.ROSException, rospy.ROSInterruptException):
            return False

    def send_step(self, step):
        if self.mode == "real" and not self.writer_connected():
            raise DriverUnavailableError("SwiftPro writer disconnected during action")
        self.server.publish_feedback(PickPlaceFeedback(phase=step.phase))
        self.sent_steps.append(step)
        if self.mode == "mock":
            return
        if step.x_mm is not None:
            response = self.move_client(step.x_mm, step.y_mm, step.z_mm)
        elif step.pump is not None:
            if step.pump:
                # A lost reply cannot prove whether the pump switched on.
                self.carrying = True
            response = self.pump_client(step.pump)
        else:
            return
        if not response.success:
            raise DriverUnavailableError(
                "SwiftPro {}: {}".format(response.code, response.detail))
        if step.pump is not None:
            self.carrying = step.pump

    def wait_step(self, seconds):
        if self.mode == "mock":
            return
        end = rospy.Time.now() + rospy.Duration.from_sec(seconds)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if self.server.is_preempt_requested():
                return
            rospy.sleep(0.05)

    def execute(self, goal):
        result = PickPlaceResult()
        try:
            bounds = self.calibration["bounds_mm"]
            heights = self.calibration["heights_mm"]
            if goal.mode == PickPlaceGoal.PICK:
                x_mm, y_mm = pixel_to_arm(goal.center_u, goal.center_v,
                                            self.calibration)
                steps = plan_pick(x_mm, y_mm, heights, bounds,
                                  self.calibration["transport_mm"])
            elif goal.mode == PickPlaceGoal.PLACE:
                if not self.carrying:
                    raise CalibrationError("cannot place without a prior pickup")
                steps = plan_place(self.calibration["drop_mm"], heights, bounds,
                                   self.calibration["transport_mm"])
            else:
                raise CalibrationError("unsupported pick/place mode")
            outcome = run_steps(
                steps, self.send_step, self.server.is_preempt_requested,
                self.wait_step, initial_carrying=self.carrying,
            )
            self.carrying = outcome.carrying
            result.success = outcome.success
            result.carrying = outcome.carrying
            result.code = outcome.code
            result.detail = ("Commands acknowledged; physical grasp unverified"
                             if self.mode == "real" else
                             "Mock command sequence completed")
            if outcome.success:
                self.server.set_succeeded(result)
            else:
                self.server.set_preempted(result)
        except (CalibrationError, ArmBoundsError, KeyError, TypeError, ValueError) as exc:
            result.success = False
            result.carrying = self.carrying
            result.code = "INVALID_TARGET"
            result.detail = str(exc)
            self.server.set_aborted(result)
        except Exception as exc:
            rospy.logerr("SwiftPro adapter failed: %s", exc)
            result.success = False
            result.carrying = self.carrying
            result.code = "DRIVER_ERROR"
            result.detail = str(exc)
            self.server.set_aborted(result)


if __name__ == "__main__":
    rospy.init_node("spark_mission_swiftpro_adapter")
    try:
        SwiftproAdapter()
        rospy.spin()
    except (CalibrationError, DriverUnavailableError) as exc:
        rospy.logfatal("SwiftPro adapter not ready: %s", exc)
