#!/usr/bin/env python3
"""Read AMCL and return candidate YAML; never command or configure motion."""
import threading

from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy

from spark_course_mission.srv import CaptureWaypoint, CaptureWaypointResponse
from spark_course_mission.waypoint_capture import capture_candidate


class WaypointCaptureNode:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest_amcl = None
        self._max_age_s = float(rospy.get_param("~max_amcl_age_s", 1.0))
        self._max_xy_sigma_m = float(rospy.get_param("~max_xy_sigma_m", 0.25))
        self._max_yaw_sigma_rad = float(rospy.get_param("~max_yaw_sigma_rad", 0.35))
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                         self.on_amcl_pose, queue_size=1)
        rospy.Service("/mission/capture_waypoint", CaptureWaypoint,
                      self.on_capture)

    def on_amcl_pose(self, message):
        with self._lock:
            self._latest_amcl = message

    def on_capture(self, request):
        with self._lock:
            message = self._latest_amcl
        try:
            snippet = capture_candidate(
                message, rospy.Time.now().to_sec(),
                request.kind, request.station_id, request.pickup_enabled,
                self._max_age_s, self._max_xy_sigma_m,
                self._max_yaw_sigma_rad)
        except ValueError as exc:
            return CaptureWaypointResponse(False, "", str(exc))
        return CaptureWaypointResponse(True, snippet, "")


if __name__ == "__main__":
    rospy.init_node("spark_waypoint_capture")
    WaypointCaptureNode()
    rospy.spin()
