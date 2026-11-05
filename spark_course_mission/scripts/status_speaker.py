#!/usr/bin/env python3
"""Forward mission announcements to voice_fms' shared TTS topic."""

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("spark_mission_status_bridge")
    publisher = rospy.Publisher("/voice/tts", String, queue_size=10)

    def on_announcement(message):
        if not message.data:
            return
        if publisher.get_num_connections() == 0:
            rospy.logwarn_throttle(10, "voice_fms TTS has no /voice/tts subscriber")
        publisher.publish(String(data=message.data))

    rospy.Subscriber("/mission/announcement", String, on_announcement, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
