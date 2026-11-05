#!/usr/bin/env python3
"""Collect a supervised 2D pixel-to-arm calibration grid.

Safety:
* Launch ``swiftpro_write_node`` with ``service_only:=true`` and F600.
* Every move is acknowledged and encoder-verified by ``swiftpro_move``.
* This script never commands the pump.
* On failure it stops at the current pose; it never sends an unverified
  retract path.
"""
import json
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from swiftpro.srv import Move

from spark_course_mission.calibration_grid import (
    CalibrationGridError, GridPoint, Sample,
    candidate_calibration, candidate_yaml_text, default_grid_points,
    fit_pixel_to_arm, validate_grid_points,
)


class CalibrationGridRunner:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = None
        self.start_event = threading.Event()
        self.started = False
        self.aborted = False
        self.display_ok = True
        self.image_size = (640, 480)
        self.previous_center = None
        self.samples = []

        output_yaml = rospy.get_param("~output_yaml", "")
        if not output_yaml:
            raise RuntimeError("~output_yaml is required")
        self.output_yaml = Path(output_yaml).expanduser()
        self.samples_json = Path(rospy.get_param(
            "~samples_json", str(self.output_yaml.with_name(
                self.output_yaml.stem + "_samples.json")))).expanduser()
        for path in (self.output_yaml, self.samples_json):
            if path.exists():
                raise RuntimeError("refusing to overwrite existing file: {}".format(path))
            path.parent.mkdir(parents=True, exist_ok=True)

        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.settle_s = float(rospy.get_param("~settle_s", 1.5))
        self.frames_per_point = int(rospy.get_param("~frames_per_point", 5))
        self.max_jitter_px = float(rospy.get_param("~max_jitter_px", 8.0))
        self.max_center_jump_px = float(rospy.get_param("~max_center_jump_px", 60.0))
        self.min_area_px = float(rospy.get_param("~min_area_px", 50.0))
        self.max_area_px = float(rospy.get_param("~max_area_px", 5000.0))
        self.collect_frames = int(rospy.get_param("~collect_frames", 200))
        self.collect_roi = [int(value) for value in
                            rospy.get_param("~collect_roi", [355, 310, 375, 340])]
        self.max_step_mm = float(rospy.get_param("~max_step_mm", 60.0))
        if not math.isfinite(self.max_step_mm) or self.max_step_mm <= 0.0:
            raise RuntimeError("~max_step_mm must be positive")
        start = rospy.get_param("~start_pose_mm", [120.0, 0.0, 35.0])
        self.start_pose = GridPoint(float(start[0]), float(start[1]), float(start[2]))
        if not all(math.isfinite(value) for value in
                   (self.start_pose.x_mm, self.start_pose.y_mm, self.start_pose.z_mm)):
            raise RuntimeError("~start_pose_mm must be finite")
        lower = rospy.get_param("~patch_hsv_lower", None)
        upper = rospy.get_param("~patch_hsv_upper", None)
        self.hsv_lower = None if lower is None else np.array(lower, dtype=np.uint8)
        self.hsv_upper = None if upper is None else np.array(upper, dtype=np.uint8)

        grid_param = rospy.get_param("~grid_points", None)
        if grid_param:
            self.points = [GridPoint(float(row[0]), float(row[1]), float(row[2]))
                           for row in grid_param]
        else:
            self.points = default_grid_points()
        envelope = rospy.get_param("~safe_envelope_mm", None)
        validate_grid_points(self.points, envelope or None)

        self.move = rospy.ServiceProxy("/swiftpro_move", Move)
        rospy.wait_for_service("/swiftpro_move", timeout=10.0)
        driver_feedrate = rospy.get_param("/swiftpro_write_node/feedrate_mm_min", None)
        if driver_feedrate is not None and int(driver_feedrate) > 600:
            raise RuntimeError("swiftpro feedrate {} exceeds the verified 600 mm/min".format(
                driver_feedrate))
        self.status_pub = rospy.Publisher(
            "/spark_calibration_grid/status", String, queue_size=10, latch=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.on_image,
                         queue_size=1, buff_size=2 ** 24)
        self.start_service = rospy.Service(
            "/spark_calibration_grid/start", Trigger, self.on_start)
        self.publish_status("READY")

    def publish_status(self, value):
        self.status_pub.publish(String(data=value))

    def on_start(self, _request):
        if self.started or self.start_event.is_set():
            return TriggerResponse(success=False, message="calibration already started")
        self.started = True
        self.start_event.set()
        return TriggerResponse(
            success=True,
            message="starting HSV collection and 2D grid; keep the sweep area clear",
        )

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5, "camera frame rejected: %s", exc)
            return
        with self.lock:
            self.latest_frame = frame
            self.latest_stamp = message.header.stamp
            self.image_size = (frame.shape[1], frame.shape[0])

    def get_frame(self, timeout=2.0):
        deadline = time.time() + float(timeout)
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()
                stamp = self.latest_stamp
            if frame is not None and stamp is not None:
                age = (rospy.Time.now() - stamp).to_sec()
                if 0.0 <= age <= 0.5:
                    return frame
            rospy.sleep(0.02)
        return None

    def show_frame(self, frame, rect=None, center=None):
        if not self.display_ok:
            return
        display = frame.copy()
        if rect is not None:
            cv2.rectangle(display, (rect[0], rect[1]), (rect[2], rect[3]),
                          (0, 255, 0), 2)
        if center is not None:
            cv2.circle(display, (int(center[0]), int(center[1])), 6,
                       (0, 0, 255), 2)
        try:
            cv2.imshow("spark_calibration_grid", display)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.display_ok = False
            rospy.logwarn("camera preview disabled: %s", exc)

    def collect_hsv(self):
        if self.hsv_lower is not None and self.hsv_upper is not None:
            rospy.loginfo("using configured patch HSV lower=%s upper=%s",
                          self.hsv_lower.tolist(), self.hsv_upper.tolist())
            return
        rospy.loginfo("collecting patch HSV from ROI %s; keep the patch inside the green box",
                      self.collect_roi)
        x0, y0, x1, y1 = self.collect_roi
        total = np.zeros(3, dtype=np.float64)
        count = 0
        deadline = time.time() + 30.0
        while count < self.collect_frames and time.time() < deadline and not rospy.is_shutdown():
            frame = self.get_frame(timeout=1.0)
            if frame is None:
                continue
            crop = frame[max(0, y0):max(0, y1), max(0, x0):max(0, x1)]
            if crop.size == 0:
                continue
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            total += np.array([hsv[:, :, 0].mean(), hsv[:, :, 1].mean(),
                               hsv[:, :, 2].mean()])
            count += 1
            self.show_frame(frame, rect=(x0, y0, x1, y1))
            rospy.sleep(0.01)
        if count < 20:
            raise RuntimeError("only {} camera frames during HSV collection".format(count))
        mean = total / float(count)
        self.hsv_lower = np.array([
            max(0, int(mean[0] - 6)),
            max(50, int(mean[1] - 120)),
            max(50, int(mean[2] - 120)),
        ], dtype=np.uint8)
        self.hsv_upper = np.array([
            min(180, int(mean[0] + 6)),
            min(255, int(mean[1] + 120)),
            min(255, int(mean[2] + 120)),
        ], dtype=np.uint8)
        rospy.loginfo("patch HSV lower=%s upper=%s",
                      self.hsv_lower.tolist(), self.hsv_upper.tolist())

    def detect_patch(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not self.min_area_px <= area <= self.max_area_px:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            candidates.append((moments["m10"] / moments["m00"],
                               moments["m01"] / moments["m00"], area))
        if not candidates:
            return None
        if self.previous_center is not None:
            nearest = min(candidates, key=lambda item: math.hypot(
                item[0] - self.previous_center[0],
                item[1] - self.previous_center[1]))
            if math.hypot(nearest[0] - self.previous_center[0],
                          nearest[1] - self.previous_center[1]) > self.max_center_jump_px:
                return None
            return nearest
        return max(candidates, key=lambda item: item[2])

    def capture_sample(self, index, point):
        for attempt in range(3):
            centers = []
            areas = []
            deadline = time.time() + 3.0
            while (len(centers) < self.frames_per_point and time.time() < deadline
                   and not rospy.is_shutdown()):
                frame = self.get_frame(timeout=1.0)
                if frame is None:
                    continue
                detected = self.detect_patch(frame)
                if detected is None:
                    continue
                centers.append((detected[0], detected[1]))
                areas.append(detected[2])
                self.show_frame(frame, center=(detected[0], detected[1]))
                rospy.sleep(0.05)
            if len(centers) < max(2, self.frames_per_point // 2):
                rospy.logwarn("point %d attempt %d: only %d patch detections",
                              index, attempt, len(centers))
                rospy.sleep(0.5)
                continue
            u_px = float(np.median([center[0] for center in centers]))
            v_px = float(np.median([center[1] for center in centers]))
            spread = max(math.hypot(center[0] - u_px, center[1] - v_px)
                         for center in centers)
            if spread > self.max_jitter_px:
                rospy.logwarn("point %d attempt %d: patch jitter %.2f px",
                              index, attempt, spread)
                rospy.sleep(0.5)
                continue
            self.previous_center = (u_px, v_px)
            return Sample(point.x_mm, point.y_mm, u_px, v_px,
                          float(np.median(areas)))
        return None

    def move_to(self, point):
        try:
            response = self.move(point.x_mm, point.y_mm, point.z_mm)
        except rospy.ServiceException as exc:
            rospy.logerr("swiftpro_move service failed: %s", exc)
            return False
        if not response.success:
            rospy.logerr("move rejected at (%.2f, %.2f, %.2f): %s %s",
                         point.x_mm, point.y_mm, point.z_mm,
                         response.code, response.detail)
            return False
        rospy.loginfo("moved to (%.2f, %.2f, %.2f); actual (%.2f, %.2f, %.2f)",
                      point.x_mm, point.y_mm, point.z_mm,
                      response.actual_x, response.actual_y, response.actual_z)
        return True

    def move_path(self, start, target):
        """Split a long straight-line move so each service call stays under 15 s."""
        distance = math.sqrt(
            (target.x_mm - start.x_mm) ** 2 +
            (target.y_mm - start.y_mm) ** 2 +
            (target.z_mm - start.z_mm) ** 2)
        steps = max(1, int(math.ceil(distance / self.max_step_mm)))
        for step in range(1, steps + 1):
            fraction = float(step) / float(steps)
            point = GridPoint(
                start.x_mm + (target.x_mm - start.x_mm) * fraction,
                start.y_mm + (target.y_mm - start.y_mm) * fraction,
                start.z_mm + (target.z_mm - start.z_mm) * fraction)
            if not self.move_to(point):
                return False
        return True

    def run(self):
        rospy.loginfo("2D calibration grid: %d points", len(self.points))
        rospy.loginfo("align the patch in the green box, then run: "
                      "rosservice call /spark_calibration_grid/start")
        self.publish_status("WAITING_FOR_START")
        if self.auto_start:
            self.start_event.set()
        while not self.start_event.is_set() and not rospy.is_shutdown():
            frame = self.get_frame(timeout=0.2)
            if frame is not None:
                self.show_frame(frame, rect=tuple(self.collect_roi))
            rospy.sleep(0.02)
        if rospy.is_shutdown():
            return
        self.collect_hsv()
        self.publish_status("RUNNING")
        if not self.move_to(self.start_pose):
            self.abort("move to start pose failed")
            return
        previous = self.start_pose
        for index, point in enumerate(self.points):
            if rospy.is_shutdown():
                self.abort("ROS shutdown during grid")
                return
            rospy.loginfo("point %d/%d target=(%.2f, %.2f, %.2f)",
                          index + 1, len(self.points), point.x_mm,
                          point.y_mm, point.z_mm)
            if not self.move_path(previous, point):
                self.abort("move failed at point {}".format(index + 1))
                return
            previous = point
            rospy.sleep(self.settle_s)
            sample = self.capture_sample(index, point)
            if sample is None:
                self.abort("patch detection failed at point {}".format(index + 1))
                return
            self.samples.append(sample)
            rospy.loginfo("sample %d: u=%.2f v=%.2f area=%.1f",
                          index + 1, sample.u_px, sample.v_px, sample.area_px)
        if len(self.samples) < 8:
            self.abort("too few samples: {}".format(len(self.samples)))
            return
        try:
            fit = fit_pixel_to_arm(self.samples)
        except CalibrationGridError as exc:
            self.write_samples(completed=True, error=str(exc))
            rospy.logerr("2D fit rejected: %s", exc)
            return
        candidate = candidate_calibration(
            fit, self.samples, image_size=self.image_size,
            record={
                "date": time.strftime("%Y-%m-%d %H:%M:%S"),
                "grid_points": [[p.x_mm, p.y_mm, p.z_mm] for p in self.points],
            })
        self.write_samples(completed=True, error="", fit=fit)
        self.write_candidate(candidate)
        rospy.loginfo(
            "fit rank=%d condition=%.3g inliers=%d outliers=%s "
            "mean=%.3f mm max=%.3f mm passed=%s",
            fit.rank, fit.condition, len(fit.inlier_indices),
            fit.outlier_indices, fit.mean_residual_mm,
            fit.max_residual_mm, fit.passed)
        if not fit.passed:
            rospy.logwarn("fit did not meet the residual target: %s", fit.reason)
        rospy.loginfo("wrote candidate %s", self.output_yaml)
        rospy.loginfo("arm is left at the last grid point; stow it manually. "
                      "No retract path was sent.")
        self.publish_status("COMPLETE")

    def abort(self, reason):
        self.aborted = True
        rospy.logerr("ABORT: %s", reason)
        rospy.logerr("arm is left at its current pose; no retract path was sent. "
                     "Take over on site.")
        try:
            self.write_samples(completed=False, error=reason)
        except Exception as exc:
            rospy.logerr("failed to write samples after abort: %s", exc)
        self.publish_status("ABORTED")

    def write_samples(self, completed, error, fit=None):
        payload = {
            "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "completed": bool(completed),
            "error": error,
            "image_size": list(self.image_size),
            "hsv_lower": None if self.hsv_lower is None else self.hsv_lower.tolist(),
            "hsv_upper": None if self.hsv_upper is None else self.hsv_upper.tolist(),
            "grid_points": [[p.x_mm, p.y_mm, p.z_mm] for p in self.points],
            "samples": [{
                "x_mm": sample.x_mm,
                "y_mm": sample.y_mm,
                "u_px": sample.u_px,
                "v_px": sample.v_px,
                "area_px": sample.area_px,
            } for sample in self.samples],
        }
        if fit is not None:
            payload["fit"] = {
                "matrix": [list(row) for row in fit.matrix],
                "residuals_mm": list(fit.residuals_mm),
                "inlier_indices": list(fit.inlier_indices),
                "outlier_indices": list(fit.outlier_indices),
                "rank": fit.rank,
                "condition": fit.condition,
                "mean_residual_mm": fit.mean_residual_mm,
                "max_residual_mm": fit.max_residual_mm,
                "passed": fit.passed,
                "reason": fit.reason,
            }
        with self.samples_json.open("x") as output:
            output.write(json.dumps(payload, ensure_ascii=False, indent=2))
            output.write("\n")

    def write_candidate(self, candidate):
        with self.output_yaml.open("x") as output:
            output.write(candidate_yaml_text(candidate))


def main():
    rospy.init_node("spark_calibration_grid")
    try:
        runner = CalibrationGridRunner()
    except Exception as exc:
        rospy.logfatal("cannot initialize calibration grid: %s", exc)
        return
    try:
        runner.run()
    except rospy.ROSInterruptException:
        runner.abort("ROS shutdown")
    except Exception as exc:
        rospy.logfatal("calibration grid failed: %s", exc)
        runner.abort(str(exc))
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
