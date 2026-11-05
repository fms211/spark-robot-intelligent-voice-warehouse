#!/usr/bin/env python3
"""ROS integration: /spark_base/odom_fault must fail the active mission safely."""
import threading
import time
import unittest

import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import String
import rostest

from spark_course_mission.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult


class MissionFaultRosTest(unittest.TestCase):
    __test__ = False  # rostest owns the ROS master and launches this suite.

    @classmethod
    def setUpClass(cls):
        rospy.init_node("spark_course_mission_fault_test", anonymous=True)
        cls.statuses = []
        cls.announcements = []
        cls.nav_goals = []
        cls.preempts = 0
        cls.zero_cmds = []
        cls.pause_nav = True
        cls.stop_publisher = threading.Event()
        cls.nav_server = actionlib.SimpleActionServer(
            "move_base", MoveBaseAction, execute_cb=cls.on_nav_goal,
            auto_start=False,
        )
        cls.arm_server = actionlib.SimpleActionServer(
            "/mission/pick_place", PickPlaceAction, execute_cb=cls.on_arm_goal,
            auto_start=False,
        )
        cls.nav_server.start()
        cls.arm_server.start()
        cls.command_pub = rospy.Publisher("/mission/command", String, queue_size=10)
        cls.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        cls.fault_pub = rospy.Publisher("/spark_base/odom_fault", String, queue_size=1)
        rospy.Subscriber("/mission/status", String,
                         lambda msg: cls.statuses.append(msg.data), queue_size=20)
        rospy.Subscriber("/mission/announcement", String,
                         lambda msg: cls.announcements.append(msg.data), queue_size=20)
        rospy.Subscriber("/navigation_velocity_smoother/raw_cmd_vel", Twist,
                         lambda msg: cls.zero_cmds.append((msg.linear.x, msg.angular.z)),
                         queue_size=50)
        cls.publisher_thread = threading.Thread(target=cls.publish_odom, daemon=True)
        cls.publisher_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.stop_publisher.set()
        cls.publisher_thread.join(timeout=1)
        cls.nav_server.action_server.stop()
        cls.arm_server.action_server.stop()

    @classmethod
    def on_nav_goal(cls, goal):
        cls.nav_goals.append(goal.target_pose.pose.position.x)
        limit = time.time() + 5
        while time.time() < limit and not rospy.is_shutdown():
            if cls.nav_server.is_preempt_requested():
                cls.preempts += 1
                cls.nav_server.set_preempted()
                return
            time.sleep(0.03)
        cls.nav_server.set_succeeded(MoveBaseResult())

    @classmethod
    def on_arm_goal(cls, goal):
        result = PickPlaceResult(success=True,
                                 carrying=goal.mode == PickPlaceGoal.PICK,
                                 code="COMMANDS_SENT_UNVERIFIED",
                                 detail="mock command")
        cls.arm_server.set_succeeded(result)

    @classmethod
    def publish_odom(cls):
        while not cls.stop_publisher.is_set() and not rospy.is_shutdown():
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.twist.twist = Twist()
            cls.odom_pub.publish(odom)
            time.sleep(0.05)

    @staticmethod
    def wait_for(predicate, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def test_odom_fault_cancels_mission_and_blocks_restart(self):
        self.assertTrue(self.wait_for(
            lambda: self.command_pub.get_num_connections() > 0, timeout=4.0),
            "mission manager did not subscribe")
        type(self).pause_nav = True
        start_nav = len(self.nav_goals)
        start_status = len(self.statuses)
        start_zero = len(self.zero_cmds)
        start_ann = len(self.announcements)
        self.command_pub.publish(String(data="START"))
        self.assertTrue(self.wait_for(lambda: len(self.nav_goals) > start_nav),
                        "mission did not dispatch the first navigation goal")
        self.assertTrue(self.wait_for(lambda: "NAV_PATROL" in self.statuses[start_status:]),
                        "mission did not enter NAV_PATROL")

        self.fault_pub.publish(String(data="ENCODER_CATASTROPHIC_DELTA"))
        self.assertTrue(self.wait_for(lambda: "SAFE_STOP" in self.statuses[start_status:]),
                        "fault did not trigger SAFE_STOP")
        self.assertTrue(self.wait_for(lambda: self.preempts >= 1),
                        "active navigation goal was not canceled")
        self.assertTrue(self.wait_for(
            lambda: any(abs(linear) <= 0.01 and abs(angular) <= 0.01
                        for linear, angular in self.zero_cmds[start_zero:])),
            "no zero-velocity command was published on fault")
        self.assertTrue(self.wait_for(
            lambda: "FAILED" in self.statuses[start_status:], timeout=10.0),
            "mission did not finish in FAILED")
        self.assertTrue(any("底盘里程计故障" in text
                            for text in self.announcements[start_ann:]),
                        "fault reason was not announced")

        before = len(self.nav_goals)
        self.command_pub.publish(String(data="START"))
        time.sleep(0.6)
        self.assertEqual(len(self.nav_goals), before,
                         "START was accepted after the odometry fault latch")


if __name__ == "__main__":
    rostest.rosrun("spark_course_mission", "mission_fault_ros_test",
                   MissionFaultRosTest)
