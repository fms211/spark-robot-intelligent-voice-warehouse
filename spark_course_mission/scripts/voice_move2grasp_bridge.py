#!/usr/bin/env python3
"""Bridge /mission/command to the original move2grasp /grasp topic.

The original move2grasp grasp_d435.py already owns detection, suction and
release.  This node only translates the course voice whitelist:

    START          -> /grasp "1"  (start detection/grasp)
    STOP, RETURN   -> /grasp "0"  (release and return home)
    STATUS, other  -> no arm action

Use ``~dry_run:=true`` to verify the mapping without publishing /grasp.
"""
import rospy
from std_msgs.msg import String

from spark_course_mission.voice_grasp import command_to_grasp


def main():
    rospy.init_node("spark_voice_move2grasp_bridge")
    dry_run = bool(rospy.get_param("~dry_run", False))
    grasp_pub = rospy.Publisher("/grasp", String, queue_size=1)
    status_pub = rospy.Publisher("/spark_voice_move2grasp/status", String,
                                 queue_size=10, latch=True)

    def on_command(message):
        command = message.data
        grasp = command_to_grasp(command)
        if grasp is None:
            text = "{} -> no grasp action".format(command)
        else:
            text = "{} -> /grasp {}".format(command, grasp)
            if dry_run:
                text += " (dry-run)"
            else:
                grasp_pub.publish(String(data=grasp))
        status_pub.publish(String(data=text))
        rospy.loginfo("voice-to-grasp: %s", text)

    rospy.Subscriber("/mission/command", String, on_command, queue_size=10)
    rospy.loginfo("voice->move2grasp bridge ready (dry_run=%s)", dry_run)
    rospy.spin()


if __name__ == "__main__":
    main()
