#!/usr/bin/env python3
"""Exercise the ROS mission boundary with synthetic images and action servers."""
import math
import threading
import time
import unittest

import actionlib
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from nav_msgs.msg import Odometry
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rostest

from spark_course_mission.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult


class MissionMockRosTest(unittest.TestCase):
    __test__ = False  # rostest owns the ROS master and launches this suite.

    @classmethod
    def setUpClass(cls):
        rospy.init_node("spark_course_mission_mock_test", anonymous=True)
        cls.bridge = CvBridge()
        cls.statuses = []
        cls.nav_goals = []
        cls.arm_modes = []
        cls.station = ""
        cls.pause_nav = False
        cls.stop_publisher = threading.Event()
        cls.nav_poses = []
        cls.rotations = []
        cls.current_pose = (0.0, 0.0, 0.0)
        cls.target_u = 50.0
        cls.draw_target = True
        cls.amcl_pub = rospy.Publisher(
            "/amcl_pose", PoseWithCovarianceStamped, queue_size=1)
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
        cls.image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
        cls.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        rospy.Subscriber("/mission/status", String,
                         lambda msg: cls.statuses.append(msg.data), queue_size=20)
        rospy.Subscriber("/mission/inspection_station", String,
                         lambda msg: setattr(cls, "station", msg.data), queue_size=10)
        cls.publisher_thread = threading.Thread(target=cls.publish_sensors, daemon=True)
        cls.publisher_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.stop_publisher.set()
        cls.publisher_thread.join(timeout=1)
        cls.nav_server.action_server.stop()
        cls.arm_server.action_server.stop()

    @classmethod
    def on_nav_goal(cls, goal):
        pose = goal.target_pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z
                   + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2))
        cls.nav_poses.append((pose.position.x, pose.position.y, yaw))
        cls.current_pose = (pose.position.x, pose.position.y, yaw)
        if len(cls.nav_poses) > 1:
            previous = cls.nav_poses[-2]
            if (abs(pose.position.x - previous[0]) < 1e-6
                    and abs(pose.position.y - previous[1]) < 1e-6
                    and abs(yaw - previous[2]) > 1e-6):
                cls.rotations.append(len(cls.nav_poses) - 1)
        cls.nav_goals.append(goal.target_pose.pose.position.x)
        if cls.pause_nav:
            limit = time.time() + 3
            while time.time() < limit and not rospy.is_shutdown():
                if cls.nav_server.is_preempt_requested():
                    cls.nav_server.set_preempted()
                    return
                time.sleep(0.03)
        cls.nav_server.set_succeeded(MoveBaseResult())

    @classmethod
    def on_arm_goal(cls, goal):
        if goal.mode == PickPlaceGoal.PICK:
            cls.draw_target = False
        cls.arm_modes.append(goal.mode)
        result = PickPlaceResult(success=True,
                                 carrying=goal.mode == PickPlaceGoal.PICK,
                                 code="COMMANDS_SENT_UNVERIFIED",
                                 detail="mock command")
        cls.arm_server.set_succeeded(result)

    @classmethod
    def publish_sensors(cls):
        while not cls.stop_publisher.is_set() and not rospy.is_shutdown():
            frame = np.zeros((100, 100, 3), dtype=np.uint8)
            if cls.draw_target and cls.station == "pickup_b":
                center = int(cls.target_u)
                cv2.rectangle(frame, (max(0, center - 15), 35),
                              (min(99, center + 15), 65), (255, 0, 0), -1)
            image = cls.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image.header.stamp = rospy.Time.now()
            image.header.frame_id = "camera_color_optical_frame"
            cls.image_pub.publish(image)
            amcl = PoseWithCovarianceStamped()
            amcl.header.stamp = rospy.Time.now()
            amcl.header.frame_id = "map"
            amcl.pose.pose.position.x = cls.current_pose[0]
            amcl.pose.pose.position.y = cls.current_pose[1]
            amcl.pose.pose.orientation.z = math.sin(cls.current_pose[2] / 2.0)
            amcl.pose.pose.orientation.w = math.cos(cls.current_pose[2] / 2.0)
            amcl.pose.covariance[0] = 0.01
            amcl.pose.covariance[7] = 0.01
            amcl.pose.covariance[35] = 0.01
            cls.amcl_pub.publish(amcl)
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

    def test_voice_started_patrol_pick_place_and_stop(self):
        self.assertTrue(self.wait_for(lambda: self.command_pub.get_num_connections() > 0,
                                      timeout=4.0), "mission manager did not subscribe")
        type(self).pause_nav = False
        type(self).draw_target = True
        type(self).target_u = 50.0
        start_arm = len(self.arm_modes)
        start_nav = len(self.nav_goals)
        start_status = len(self.statuses)
        self.command_pub.publish(String(data="START"))
        self.assertTrue(self.wait_for(
            lambda: "COMPLETE" in self.statuses[start_status:]),
            "mission did not complete through fake action servers")
        self.assertEqual(self.arm_modes[start_arm:start_arm + 2],
                         [PickPlaceGoal.PICK, PickPlaceGoal.PLACE])
        self.assertGreaterEqual(len(self.nav_goals) - start_nav, 4)

        self.pause_nav = True
        before = len(self.nav_goals)
        self.command_pub.publish(String(data="START"))
        self.assertTrue(self.wait_for(lambda: len(self.nav_goals) > before),
                        "second patrol did not start")
        self.command_pub.publish(String(data="STOP"))
        self.assertTrue(self.wait_for(lambda: "CANCELED" in self.statuses),
                        "STOP did not cancel the second patrol")
        stopped_at = len(self.nav_goals)
        time.sleep(0.4)
        self.assertEqual(len(self.nav_goals), stopped_at)

    def test_alignment_rotation_then_pick(self):
        self.assertTrue(self.wait_for(lambda: self.command_pub.get_num_connections() > 0,
                                      timeout=4.0), "mission manager did not subscribe")
        type(self).pause_nav = False
        type(self).draw_target = True
        type(self).target_u = 85.0
        start_arm = len(self.arm_modes)
        start_nav = len(self.nav_poses)
        start_status = len(self.statuses)
        self.command_pub.publish(String(data="START"))
        self.assertTrue(self.wait_for(
            lambda: any(index >= start_nav for index in self.rotations),
            timeout=15.0), "mission did not issue a rotation goal")
        type(self).target_u = 50.0
        self.assertTrue(self.wait_for(
            lambda: PickPlaceGoal.PICK in self.arm_modes[start_arm:],
            timeout=15.0), "mission did not pick after alignment")
        self.assertTrue(self.wait_for(
            lambda: "COMPLETE" in self.statuses[start_status:], timeout=25.0),
            "mission did not complete after alignment")


if __name__ == "__main__":
    rostest.rosrun("spark_course_mission", "mission_mock_ros_test", MissionMockRosTest)
