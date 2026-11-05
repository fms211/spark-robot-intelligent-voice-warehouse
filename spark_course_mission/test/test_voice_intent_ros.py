#!/usr/bin/env python3
"""Exercise the real voice-intent ROS node without using a microphone."""
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String


class VoiceIntentRosTest(unittest.TestCase):
    __test__ = False  # rostest owns this ROS integration test.

    def test_chinese_whitelist_publishes_only_mission_commands(self):
        rospy.init_node("spark_course_mission_voice_intent_test", anonymous=True)
        received = []
        command_sub = rospy.Subscriber("/mission/command", String,
                                       lambda msg: received.append(msg.data),
                                       queue_size=10)
        speech_pub = rospy.Publisher("/voice/stt", String, queue_size=10)

        deadline = time.time() + 3
        while time.time() < deadline and (
                speech_pub.get_num_connections() == 0
                or command_sub.get_num_connections() == 0):
            time.sleep(0.05)
        self.assertGreater(speech_pub.get_num_connections(), 0,
                           "voice_intent did not subscribe to /voice/stt")
        self.assertGreater(command_sub.get_num_connections(), 0,
                           "voice_intent did not publish /mission/command")

        speech_pub.publish(String(data="请 开 始 巡 检。"))
        deadline = time.time() + 3
        while time.time() < deadline and not received:
            time.sleep(0.05)
        self.assertEqual(received, ["START"])

        speech_pub.publish(String(data="不要开始巡检"))
        time.sleep(0.35)
        self.assertEqual(received, ["START"])

        speech_pub.publish(String(data="停止任务"))
        deadline = time.time() + 3
        while time.time() < deadline and len(received) < 2:
            time.sleep(0.05)
        self.assertEqual(received, ["START", "STOP"])

        speech_pub.publish(String(data="嗯停止任务"))
        deadline = time.time() + 3
        while time.time() < deadline and len(received) < 3:
            time.sleep(0.05)
        self.assertEqual(received, ["START", "STOP", "STOP"])

        speech_pub.publish(String(data="嗯不要停止任务"))
        time.sleep(0.35)
        self.assertEqual(received, ["START", "STOP", "STOP"])


if __name__ == "__main__":
    rostest.rosrun("spark_course_mission", "voice_intent_ros_test", VoiceIntentRosTest)
