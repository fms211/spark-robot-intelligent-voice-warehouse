#!/usr/bin/env python3
"""Mission announcements must reach voice_fms TTS without opening audio."""
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String


class VoiceFmsBridgeRosTest(unittest.TestCase):
    __test__ = False  # rostest owns the isolated ROS master.

    def test_announcement_is_forwarded_to_voice_tts(self):
        rospy.init_node("spark_course_mission_voice_fms_bridge_test", anonymous=True)
        heard = []
        tts_sub = rospy.Subscriber("/voice/tts", String,
                                   lambda msg: heard.append(msg.data), queue_size=10)
        announce_pub = rospy.Publisher("/mission/announcement", String,
                                        queue_size=10)

        deadline = time.time() + 4
        while time.time() < deadline and (
                announce_pub.get_num_connections() == 0
                or tts_sub.get_num_connections() == 0):
            time.sleep(0.05)
        self.assertGreater(announce_pub.get_num_connections(), 0,
                           "mission status bridge did not subscribe")
        self.assertGreater(tts_sub.get_num_connections(), 0,
                           "mission status bridge did not publish /voice/tts")

        announce_pub.publish(String(data="巡检到达站点"))
        deadline = time.time() + 3
        while time.time() < deadline and not heard:
            time.sleep(0.05)
        self.assertEqual(heard, ["巡检到达站点"])


if __name__ == "__main__":
    rostest.rosrun("spark_course_mission", "voice_fms_bridge_ros_test",
                   VoiceFmsBridgeRosTest)
