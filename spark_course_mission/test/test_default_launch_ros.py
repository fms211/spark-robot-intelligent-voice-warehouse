#!/usr/bin/env python3
"""Default mission launch starts in safe mock mode with unset field poses."""
import threading
import unittest

import rospy
import rostest
from std_msgs.msg import String


class DefaultLaunchTest(unittest.TestCase):
    __test__ = False  # rostest owns the isolated ROS master.

    def test_default_launch_starts_and_rejects_unconfigured_start(self):
        prefix = '/spark_mission_manager/mission_config/'
        self.assertFalse(rospy.get_param(prefix + 'valid_waypoints'))
        self.assertEqual(rospy.get_param(prefix + 'home_pose'), [])
        self.assertEqual(rospy.get_param(prefix + 'drop_pose'), [])
        seen = []
        failed = threading.Event()
        def on_status(msg):
            seen.append(msg.data)
            if msg.data == 'FAILED':
                failed.set()
        sub = rospy.Subscriber('/mission/status', String, on_status, queue_size=10)
        pub = rospy.Publisher('/mission/command', String, queue_size=1)
        end = rospy.Time.now() + rospy.Duration(5)
        while pub.get_num_connections() == 0 and rospy.Time.now() < end:
            rospy.sleep(.05)
        self.assertGreater(pub.get_num_connections(), 0)
        pub.publish(String(data='START'))
        self.assertTrue(failed.wait(5), seen)
        self.assertNotIn('NAV_PATROL', seen)
        sub.unregister()


if __name__ == '__main__':
    rospy.init_node('default_launch_test')
    rostest.rosrun('spark_course_mission', 'default_launch_test', DefaultLaunchTest)
