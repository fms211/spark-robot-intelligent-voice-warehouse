#!/usr/bin/env python3
"""Turn voice_fms ASR text into mission-level ROS commands."""
import rospy
from std_msgs.msg import String

from spark_course_mission.intent import parse_intent


def main():
    rospy.init_node("spark_mission_voice_intent")
    publisher = rospy.Publisher("/mission/command", String, queue_size=10)

    def on_speech(message):
        command = parse_intent(message.data)
        if command is not None:
            publisher.publish(String(data=command))
            rospy.loginfo("Mission voice command: %s", command)

    rospy.Subscriber("/voice/stt", String, on_speech, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
