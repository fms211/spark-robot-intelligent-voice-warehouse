#!/usr/bin/env python3
"""Exercise candidate capture through the real ROS service, without motion."""
import time
import unittest

from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import rosservice
import rostest


class WaypointCaptureRosTest(unittest.TestCase):
    __test__ = False

    def test_service_returns_candidate_only_after_fresh_map_localization(self):
        rospy.init_node("spark_waypoint_capture_test", anonymous=True)
        service_name = "/mission/capture_waypoint"
        try:
            rospy.wait_for_service(service_name, timeout=3)
        except rospy.ROSException as exc:
            self.fail("waypoint capture service unavailable: {}".format(exc))

        service_class = rosservice.get_service_class_by_name(service_name)
        self.assertIsNotNone(service_class)
        capture = rospy.ServiceProxy(service_name, service_class)
        request = service_class._request_class(
            kind="station", station_id="pickup_b", pickup_enabled=True)

        missing = capture(request)
        self.assertFalse(missing.success)
        self.assertIn("no AMCL", missing.reason)

        pose_pub = rospy.Publisher(
            "/amcl_pose", PoseWithCovarianceStamped, queue_size=1)
        deadline = time.time() + 3
        while time.time() < deadline and pose_pub.get_num_connections() == 0:
            time.sleep(0.05)
        self.assertGreater(pose_pub.get_num_connections(), 0)

        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = 1.2
        message.pose.pose.position.y = -0.3
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.04

        result = None
        deadline = time.time() + 3
        while time.time() < deadline:
            message.header.stamp = rospy.Time.now()
            pose_pub.publish(message)
            time.sleep(0.05)
            result = capture(request)
            if result.success:
                break
        self.assertIsNotNone(result)
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.reason, "")
        self.assertIn("- id: pickup_b", result.yaml_snippet)
        self.assertIn("pickup_enabled: true", result.yaml_snippet)
        self.assertIn("1.200000, -0.300000", result.yaml_snippet)


if __name__ == "__main__":
    rostest.rosrun(
        "spark_course_mission", "waypoint_capture_ros_test",
        WaypointCaptureRosTest)
