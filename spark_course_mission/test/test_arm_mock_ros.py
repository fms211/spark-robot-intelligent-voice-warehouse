#!/usr/bin/env python3
"""The actual mock arm action server must preserve carried-state results."""
import unittest

import actionlib
import rospy
import rostest

from spark_course_mission.msg import PickPlaceAction, PickPlaceGoal


class ArmMockRosTest(unittest.TestCase):
    __test__ = False  # rostest launches the action server and ROS master.

    def test_pick_then_place_reports_carrying_state(self):
        rospy.init_node("spark_course_mission_arm_mock_test", anonymous=True)
        client = actionlib.SimpleActionClient("/mission/pick_place", PickPlaceAction)
        self.assertTrue(client.wait_for_server(rospy.Duration.from_sec(3)),
                        "mock SwiftPro action server did not start")

        pick = PickPlaceGoal(mode=PickPlaceGoal.PICK, station_id="pickup_b",
                             center_u=320.0, center_v=240.0)
        client.send_goal(pick)
        self.assertTrue(client.wait_for_result(rospy.Duration.from_sec(4)))
        picked = client.get_result()
        self.assertTrue(picked.success)
        self.assertTrue(picked.carrying)
        self.assertEqual(picked.code, "COMMANDS_SENT_UNVERIFIED")

        place = PickPlaceGoal(mode=PickPlaceGoal.PLACE, station_id="drop",
                              center_u=0.0, center_v=0.0)
        client.send_goal(place)
        self.assertTrue(client.wait_for_result(rospy.Duration.from_sec(4)))
        placed = client.get_result()
        self.assertTrue(placed.success)
        self.assertFalse(placed.carrying)


if __name__ == "__main__":
    rostest.rosrun("spark_course_mission", "arm_mock_ros_test", ArmMockRosTest)
