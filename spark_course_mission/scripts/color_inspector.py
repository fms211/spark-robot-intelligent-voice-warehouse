#!/usr/bin/env python3
"""Publish fresh and stable color-inspection results for the active station."""
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from spark_course_mission.detection import StableTracker, detect_hsv
from spark_course_mission.msg import InspectionResult


class ColorInspector:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_name = rospy.get_param("~color_name", "blue")
        self.lower = rospy.get_param("~hsv_lower", [100, 90, 80])
        self.upper = rospy.get_param("~hsv_upper", [130, 255, 255])
        self.roi = rospy.get_param("~roi", [0, 0, 640, 480])
        self.min_area = float(rospy.get_param("~min_area_px", 400))
        self.tracker = StableTracker(
            max_jitter_px=float(rospy.get_param("~max_jitter_px", 12)),
            max_age_s=float(rospy.get_param("~max_age_s", 0.5)),
        )
        self.station_id = ""
        self.publisher = rospy.Publisher("/mission/inspection_result", InspectionResult,
                                         queue_size=10)
        rospy.Subscriber("/mission/inspection_station", String, self.on_station,
                         queue_size=1)
        rospy.Subscriber("/camera/color/image_raw", Image, self.on_image,
                         queue_size=1, buff_size=2 ** 24)

    def on_station(self, message):
        if message.data != self.station_id:
            self.station_id = message.data
            self.tracker.reset()

    def on_image(self, message):
        if not self.station_id or message.header.stamp == rospy.Time():
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            candidate = detect_hsv(frame, self.lower, self.upper, self.roi,
                                   self.min_area)
        except (CvBridgeError, ValueError) as exc:
            rospy.logwarn_throttle(5, "Color inspection frame rejected: %s", exc)
            return
        decision = self.tracker.add(candidate, message.header.stamp.to_sec(),
                                    rospy.Time.now().to_sec())
        if decision is None:
            return
        result = InspectionResult()
        result.header = message.header
        result.station_id = self.station_id
        result.found = decision.found
        result.color_name = self.color_name
        result.center_u = decision.center_u
        result.center_v = decision.center_v
        result.area_px = decision.area_px
        result.confidence = decision.confidence
        self.publisher.publish(result)


if __name__ == "__main__":
    rospy.init_node("spark_mission_color_inspector")
    ColorInspector()
    rospy.spin()
