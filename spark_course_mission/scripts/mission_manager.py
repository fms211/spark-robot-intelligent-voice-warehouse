#!/usr/bin/env python3
"""Bind the patrol state machine to existing ROS navigation and arm actions."""
from types import SimpleNamespace
import math
import threading
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
import rosgraph
import rospy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, TriggerResponse
import tf2_ros

from spark_course_mission.alignment import DEFAULT_INSET_RATIO, build_window
from spark_course_mission.arm import (
    ArmBoundsError, CalibrationError, grasp_verdict,
)
from spark_course_mission.mission import MissionController
from spark_course_mission.msg import InspectionResult, PickPlaceAction, PickPlaceGoal
from spark_course_mission.navigation import (
    validate_navigation_config, validate_navigation_pose,
)
from spark_course_mission.waypoint_capture import capture_candidate


class NavigationClient:
    def __init__(self, pose_source=None, refresh_source=None):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.stop_pub = rospy.Publisher(
            "/navigation_velocity_smoother/raw_cmd_vel", Twist, queue_size=1)
        self.on_done = None
        self.sequence = 0
        self.pose_source = pose_source
        self.refresh_source = refresh_source

    def ready(self):
        return self.client.wait_for_server(rospy.Duration.from_sec(0.2))

    def send_goal(self, pose):
        pose = validate_navigation_pose(pose)
        self.sequence += 1
        goal_id = "nav-{}".format(self.sequence)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(pose[0])
        goal.target_pose.pose.position.y = float(pose[1])
        goal.target_pose.pose.position.z = float(pose[2])
        goal.target_pose.pose.orientation.x = float(pose[3])
        goal.target_pose.pose.orientation.y = float(pose[4])
        goal.target_pose.pose.orientation.z = float(pose[5])
        goal.target_pose.pose.orientation.w = float(pose[6])
        self.client.send_goal(goal, done_cb=lambda status, result: self.on_done(goal_id, status))
        return goal_id

    def ensure_pose_ready(self):
        message = self.pose_source() if self.pose_source is not None else None
        if message is not None:
            stamp = message.header.stamp.to_sec()
            age = rospy.Time.now().to_sec() - stamp
            if math.isfinite(stamp) and -0.1 <= age <= 2.0:
                return True
        refresh = getattr(self, "refresh_source", None)
        if refresh is not None:
            refresh()
        return False

    def rotate_by(self, delta_rad):
        """Send a move_base goal with the same x/y and a rotated yaw."""
        if self.pose_source is None:
            return None
        message = self.pose_source()
        if message is None:
            return None
        stamp = message.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if not math.isfinite(stamp) or age < -0.1 or age > 2.0:
            return None
        pose = message.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z
                   + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2))
        target_yaw = yaw + float(delta_rad)
        return self.send_goal([
            pose.position.x, pose.position.y, 0.0,
            0.0, 0.0, math.sin(target_yaw / 2.0), math.cos(target_yaw / 2.0),
        ])

    def cancel_goal(self):
        if self.client.get_state() in (
                GoalStatus.PENDING, GoalStatus.ACTIVE,
                GoalStatus.PREEMPTING, GoalStatus.RECALLING):
            self.client.cancel_goal()

    def stop(self):
        self.stop_pub.publish(Twist())


class ArmClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("/mission/pick_place", PickPlaceAction)
        self.on_done = None

    def ready(self):
        return self.client.wait_for_server(rospy.Duration.from_sec(0.2))

    def send_goal(self, mode, station_id, center_u, center_v):
        goal = PickPlaceGoal()
        goal.mode = PickPlaceGoal.PICK if mode == "PICK" else PickPlaceGoal.PLACE
        goal.station_id = station_id
        goal.center_u = center_u
        goal.center_v = center_v

        def done(status, result):
            success = status == GoalStatus.SUCCEEDED and result is not None and result.success
            code = result.code if result is not None else "NO_RESULT"
            carrying = result.carrying if result is not None else None
            self.on_done(success, code, carrying)

        self.client.send_goal(goal, done_cb=done)

    def cancel_goal(self):
        if self.client.get_state() in (
                GoalStatus.PENDING, GoalStatus.ACTIVE,
                GoalStatus.PREEMPTING, GoalStatus.RECALLING):
            self.client.cancel_goal()


class MissionManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.config = rospy.get_param("~mission_config", {})
        self.config["require_transport_confirmation"] = bool(
            rospy.get_param("~require_transport_confirmation", False))
        self.simulation_mode = bool(rospy.get_param("~simulation_mode", False))
        self.node_name = rospy.get_name()
        self.ros_master = rosgraph.Master(self.node_name)
        self.last_image = None
        self.last_scan = None
        self.last_scan_message = None
        self.last_odom = None
        self.last_amcl = None
        self.last_amcl_message = None
        self.map_seen = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.status_pub = rospy.Publisher("/mission/status", String, queue_size=10,
                                           latch=True)
        self.announcement_pub = rospy.Publisher("/mission/announcement", String,
                                                 queue_size=10)
        self.station_pub = rospy.Publisher("/mission/inspection_station", String,
                                            queue_size=1, latch=True)
        self._load_alignment()
        self.grasp_probe = self._load_grasp_probe()
        self._amcl_refresh_lock = threading.Lock()
        self._last_amcl_refresh_attempt = 0.0
        self.nav = NavigationClient(lambda: self.last_amcl_message,
                                    self._queue_amcl_refresh)
        self.arm = ArmClient()
        self.controller = MissionController(
            self.config, self.nav, self.arm, self.precheck,
            self.publish_status, self.publish_announcement,
            self.publish_station, lambda: rospy.Time.now().to_sec(),
            grasp_probe=self.grasp_probe,
        )
        self.nav.on_done = self.on_nav_done
        self.arm.on_done = self.on_arm_done
        rospy.Subscriber("/mission/command", String, self.on_command, queue_size=10)
        rospy.Subscriber("/mission/inspection_result", InspectionResult,
                         self.on_inspection, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=10)
        rospy.Subscriber("/spark_base/odom_fault", String, self.on_odom_fault, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw", Image, self.on_image,
                         queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.on_amcl,
                         queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.on_map, queue_size=1)
        rospy.Service("/mission/confirm_arm_stowed", Trigger,
                      self.on_confirm_arm_stowed)
        rospy.Timer(rospy.Duration.from_sec(0.1), self.on_timer)
        self.publish_status("IDLE")

    @staticmethod
    def fresh(stamp, limit):
        return stamp is not None and 0 <= rospy.Time.now().to_sec() - stamp <= limit

    @staticmethod
    def usable_scan(message):
        if message is None or not message.header.frame_id:
            return False
        ranges = message.ranges
        if len(ranges) < 60 or not (
                math.isfinite(message.range_min)
                and math.isfinite(message.range_max)
                and 0 <= message.range_min < message.range_max):
            return False
        valid = sum(
            math.isfinite(distance)
            and message.range_min <= distance <= message.range_max
            for distance in ranges)
        return valid >= 60 and valid / len(ranges) >= 0.2

    def velocity_channels_ready(self):
        try:
            publishers = dict(self.ros_master.getSystemState()[0])
        except Exception as exc:
            rospy.logerr_throttle(10, "Cannot inspect velocity publishers: %s", exc)
            return False
        cmd_vel = set(publishers.get("/cmd_vel", []))
        raw_cmd_vel = set(publishers.get(
            "/navigation_velocity_smoother/raw_cmd_vel", []))
        expected_cmd_vel = {"/spark_base_nodelet_manager"}
        expected_raw_cmd_vel = {"/move_base", self.node_name}
        if cmd_vel != expected_cmd_vel or raw_cmd_vel != expected_raw_cmd_vel:
            rospy.logerr_throttle(
                10, "Unsafe velocity publishers: /cmd_vel=%s, raw_cmd_vel=%s",
                sorted(cmd_vel), sorted(raw_cmd_vel))
            return False
        return True

    @staticmethod
    def _align_number(value, default):
        try:
            number = float(value)
        except (TypeError, ValueError):
            return default
        return number if math.isfinite(number) and number > 0 else default

    @staticmethod
    def _align_inset(value):
        try:
            inset = float(value)
        except (TypeError, ValueError):
            return DEFAULT_INSET_RATIO
        if not math.isfinite(inset):
            return DEFAULT_INSET_RATIO
        return inset

    def _load_alignment(self):
        align = dict(self.config.get("align") or {})
        align["enabled"] = bool(align.get("enabled", False))
        align["step_deg"] = self._align_number(align.get("step_deg"), 30.0)
        align["fine_step_deg"] = self._align_number(
            align.get("fine_step_deg"), 10.0)
        align["max_rotation_deg"] = self._align_number(
            align.get("max_rotation_deg"), 360.0)
        align["max_steps"] = max(
            1, int(self._align_number(align.get("max_steps"), 30.0)))
        align["step_timeout_s"] = self._align_number(
            align.get("step_timeout_s"), 10.0)
        align["total_timeout_s"] = self._align_number(
            align.get("total_timeout_s"), 60.0)
        window = None
        if align["enabled"]:
            roi = rospy.get_param("~alignment_calibration/pixel_roi", None)
            try:
                window = build_window(
                    roi, self._align_inset(align.get("roi_inset_ratio")))
            except ValueError as exc:
                rospy.logwarn("alignment disabled: %s", exc)
                align["enabled"] = False
        align["window"] = window
        self.config["align"] = align

    def _load_grasp_probe(self):
        """Use the adapter's own calibration to keep ALIGN inside its bounds."""
        calibration = rospy.get_param("~alignment_calibration", None)
        if (not isinstance(calibration, dict)
                or calibration.get("calibration_mode") != "field"
                or not calibration.get("valid")):
            return None
        try:
            grasp_verdict(0.0, 0.0, calibration)
        except (CalibrationError, ArmBoundsError, KeyError, TypeError,
                ValueError) as exc:
            rospy.logwarn("grasp-aware alignment disabled: %s", exc)
            return None

        window = self.config.get("align", {}).get("window")
        u_limits = window.get("u") if isinstance(window, dict) else None

        def probe(u, v):
            try:
                status, target_u = grasp_verdict(
                    u, v, calibration, u_limits=u_limits)
                rospy.loginfo_throttle(
                    1.0, "grasp probe u=%.1f v=%.1f -> %s target=%s",
                    float(u), float(v), status,
                    "%.1f" % target_u if target_u is not None else "-")
                return status, target_u
            except (CalibrationError, ArmBoundsError, KeyError, TypeError,
                    ValueError) as exc:
                rospy.logwarn_throttle(10.0, "grasp check failed: %s", exc)
                return "impossible", None

        return probe

    def precheck(self):
        try:
            validate_navigation_config(self.config)
        except ValueError as exc:
            rospy.logerr_throttle(10, "Invalid mission waypoint configuration: %s", exc)
            return False
        if not self.nav.ready() or not self.arm.ready():
            return False
        if self.simulation_mode:
            return True
        if not self.velocity_channels_ready():
            return False
        if not self.map_seen or not self.tf_buffer.can_transform(
                "map", "base_footprint", rospy.Time(0), rospy.Duration.from_sec(0.2)):
            return False
        if not (self.fresh(self.last_image, 2.0)
                and self.fresh(self.last_odom, 2.0)
                and self.usable_scan(self.last_scan_message)
                and self.fresh(
                    self.last_scan_message.header.stamp.to_sec(), 2.0)
                and self.tf_buffer.can_transform(
                    "base_footprint", self.last_scan_message.header.frame_id,
                    rospy.Time(0), rospy.Duration.from_sec(0.2))):
            return False
        try:
            capture_candidate(
                self.last_amcl_message, rospy.Time.now().to_sec(),
                "home", "", False, max_age_s=2.0)
        except ValueError:
            return False
        return True

    def _queue_amcl_refresh(self):
        # This is called under the controller lock. Never block STOP or odometry
        # callbacks while contacting the service; only one worker may run.
        now = rospy.Time.now().to_sec()
        if (not self.controller._stationary(now)
                or not self.fresh(self.last_scan, 0.5)
                or self.controller.odom_fault_latched):
            return
        monotonic_now = time.monotonic()
        if (monotonic_now - self._last_amcl_refresh_attempt < 1.0
                or not self._amcl_refresh_lock.acquire(False)):
            return
        self._last_amcl_refresh_attempt = monotonic_now

        def request():
            try:
                rospy.wait_for_service("/request_nomotion_update", timeout=0.5)
                rospy.ServiceProxy("/request_nomotion_update", Empty)()
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn("AMCL refresh unavailable: %s", exc)
            finally:
                self._amcl_refresh_lock.release()

        worker = threading.Thread(target=request)
        worker.daemon = True
        worker.start()

    def publish_status(self, value):
        rospy.loginfo("mission status: %s", value)
        self.status_pub.publish(String(data=value))

    def publish_announcement(self, value):
        rospy.loginfo("mission announcement: %s", value)
        self.announcement_pub.publish(String(data=value))

    def publish_station(self, value):
        self.station_pub.publish(String(data=value))

    def on_command(self, message):
        with self.lock:
            self.controller.on_intent(message.data)

    def on_nav_done(self, goal_id, status):
        with self.lock:
            self.controller.on_nav_done(goal_id, status)

    def on_arm_done(self, success, code, carrying):
        with self.lock:
            self.controller.on_arm_done(success, code, carrying)

    def on_confirm_arm_stowed(self, request):
        with self.lock:
            confirmed = self.controller.confirm_transport()
        return TriggerResponse(
            success=confirmed,
            message="transport confirmed" if confirmed else "not waiting for safe stow",
        )

    def on_inspection(self, message):
        event = SimpleNamespace(
            station_id=message.station_id,
            found=message.found,
            center_u=message.center_u,
            center_v=message.center_v,
            stamp=message.header.stamp.to_sec(),
        )
        with self.lock:
            self.controller.on_detection(event)
            state = self.controller.state
            index = self.controller.station_index
        if (state in ("INSPECT", "ALIGN")
                and 0 <= index < len(self.config["stations"])
                and self.config["stations"][index].get("pickup_enabled")):
            rospy.loginfo_throttle(
                1.0, "inspection found=%s u=%.1f v=%.1f state=%s",
                message.found, message.center_u, message.center_v, state)

    def on_odom(self, message):
        stamp = message.header.stamp.to_sec()
        with self.lock:
            accepted = self.controller.on_base_velocity(
                message.twist.twist.linear.x,
                message.twist.twist.angular.z,
                stamp,
            )
        if accepted:
            self.last_odom = stamp

    def on_odom_fault(self, message):
        with self.lock:
            self.controller.on_odom_fault(message.data)

    def on_scan(self, message):
        self.last_scan_message = message
        self.last_scan = message.header.stamp.to_sec()

    def on_image(self, message):
        self.last_image = message.header.stamp.to_sec()

    def on_amcl(self, message):
        self.last_amcl_message = message
        self.last_amcl = message.header.stamp.to_sec()

    def on_map(self, message):
        self.map_seen = bool(message.info.width and message.info.height)

    def on_timer(self, event):
        with self.lock:
            self.controller.tick(rospy.Time.now().to_sec())


if __name__ == "__main__":
    rospy.init_node("spark_mission_manager")
    MissionManager()
    rospy.spin()
