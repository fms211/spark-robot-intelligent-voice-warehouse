"""The ROS adapter must reject bad waypoints before action dispatch."""
import importlib.util
import os
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from sensor_msgs.msg import LaserScan


def load_mission_manager():
    default_script = Path(__file__).resolve().parents[1] / "scripts" / "mission_manager.py"
    script = Path(os.environ.get("MISSION_MANAGER_UNDER_TEST", str(default_script)))
    spec = importlib.util.spec_from_file_location("mission_manager_under_test", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ReadyClient:
    def __init__(self):
        self.ready_calls = 0

    def ready(self):
        self.ready_calls += 1
        return True


def test_invalid_waypoints_fail_before_navigation_or_arm_readiness(monkeypatch):
    module = load_mission_manager()
    manager = module.MissionManager.__new__(module.MissionManager)
    manager.config = {
        "stations": [
            {"id": "inspection", "pose": [0, 0, 0, 0, 0, 0, 1],
             "pickup_enabled": "false"},
            {"id": "pickup", "pose": [1, 0, 0, 0, 0, 0, 1],
             "pickup_enabled": True},
        ],
        "drop_pose": [1, 0, 0, 0, 0, 0, 1],
        "home_pose": [0, 0, 0, 0, 0, 0, 1],
    }
    manager.nav = ReadyClient()
    manager.arm = ReadyClient()
    manager.simulation_mode = True
    monkeypatch.setattr(module.rospy, "logerr_throttle", lambda *args: None)

    assert manager.precheck() is False
    assert manager.nav.ready_calls == 0
    assert manager.arm.ready_calls == 0


class ReadyTransform:
    def __init__(self):
        self.scan_frame_available = True

    def can_transform(self, target, source, *args):
        if target == "base_footprint" and source == "laser_frame":
            return self.scan_frame_available
        return True


class VelocityMaster:
    def __init__(self):
        self.publishers = [
            ["/cmd_vel", ["/spark_base_nodelet_manager"]],
            ["/navigation_velocity_smoother/raw_cmd_vel",
             ["/move_base", "/spark_mission_manager"]],
        ]

    def getSystemState(self):
        return [self.publishers, [], []]


def real_precheck_manager(monkeypatch):
    module = load_mission_manager()
    now = rospy.Time.from_sec(time.time())
    monkeypatch.setattr(rospy.Time, "now", staticmethod(lambda: now))
    manager = module.MissionManager.__new__(module.MissionManager)
    valid_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    manager.config = {
        "stations": [
            {"id": "patrol_a", "pose": valid_pose, "pickup_enabled": False},
            {"id": "pickup_b", "pose": valid_pose, "pickup_enabled": True},
        ],
        "drop_pose": valid_pose,
        "home_pose": valid_pose,
    }
    manager.nav = ReadyClient()
    manager.arm = ReadyClient()
    manager.simulation_mode = False
    manager.node_name = "/spark_mission_manager"
    manager.ros_master = VelocityMaster()
    manager.map_seen = True
    manager.tf_buffer = ReadyTransform()
    manager.last_image = now.to_sec()
    manager.last_odom = now.to_sec()

    scan = LaserScan()
    scan.header.stamp = now
    scan.header.frame_id = "laser_frame"
    scan.range_min = 0.1
    scan.range_max = 5.0
    scan.ranges = [1.0] * 360
    manager.on_scan(scan)

    amcl = PoseWithCovarianceStamped()
    amcl.header.stamp = now
    amcl.header.frame_id = "map"
    amcl.pose.pose.orientation.w = 1.0
    amcl.pose.covariance[0] = 0.01
    amcl.pose.covariance[7] = 0.01
    amcl.pose.covariance[35] = 0.04
    manager.on_amcl(amcl)
    return manager, scan, amcl


def test_real_precheck_accepts_fresh_dense_scan_and_confident_amcl(monkeypatch):
    manager, _, _ = real_precheck_manager(monkeypatch)
    assert manager.precheck() is True


def test_real_precheck_rejects_competing_cmd_vel_publisher(monkeypatch):
    manager, _, _ = real_precheck_manager(monkeypatch)
    manager.ros_master.publishers[0][1].append("/voice_nav")
    assert manager.precheck() is False


def test_real_precheck_rejects_missing_navigation_smoother(monkeypatch):
    manager, _, _ = real_precheck_manager(monkeypatch)
    manager.ros_master.publishers[0][1] = ["/voice_nav"]
    assert manager.precheck() is False


def test_real_precheck_rejects_competing_raw_velocity_publisher(monkeypatch):
    manager, _, _ = real_precheck_manager(monkeypatch)
    manager.ros_master.publishers[1][1].append("/other_teleop")
    assert manager.precheck() is False


def test_real_precheck_rejects_missing_move_base_velocity_path(monkeypatch):
    manager, _, _ = real_precheck_manager(monkeypatch)
    manager.ros_master.publishers[1][1].remove("/move_base")
    assert manager.precheck() is False


def test_real_precheck_fails_closed_when_master_state_is_unavailable(monkeypatch):
    manager, _, _ = real_precheck_manager(monkeypatch)
    manager.ros_master.getSystemState = lambda: (_ for _ in ()).throw(RuntimeError("offline"))
    assert manager.precheck() is False


def test_real_precheck_rejects_d435_like_sparse_scan(monkeypatch):
    manager, scan, _ = real_precheck_manager(monkeypatch)
    scan.ranges = [1.0] * 21 + [float("inf")] * 339
    manager.on_scan(scan)
    assert manager.precheck() is False


def test_real_precheck_rejects_scan_without_frame(monkeypatch):
    manager, scan, _ = real_precheck_manager(monkeypatch)
    scan.header.frame_id = ""
    manager.on_scan(scan)
    assert manager.precheck() is False


def test_real_precheck_rejects_scan_without_base_transform(monkeypatch):
    manager, _, _ = real_precheck_manager(monkeypatch)
    manager.tf_buffer.scan_frame_available = False
    assert manager.precheck() is False


def test_real_precheck_uses_scan_message_timestamp_not_cached_other_frame(monkeypatch):
    manager, scan, _ = real_precheck_manager(monkeypatch)
    manager.last_scan = rospy.Time.now().to_sec()
    scan.header.stamp = rospy.Time.from_sec(manager.last_scan - 3.0)
    manager.last_scan_message = scan
    assert manager.precheck() is False


def test_real_precheck_rejects_uninitialized_amcl_covariance(monkeypatch):
    manager, _, amcl = real_precheck_manager(monkeypatch)
    amcl.pose.covariance = [0.0] * 36
    manager.on_amcl(amcl)
    assert manager.precheck() is False


def test_real_precheck_rejects_nonplanar_amcl_pose(monkeypatch):
    manager, _, amcl = real_precheck_manager(monkeypatch)
    amcl.pose.pose.position.z = 0.3
    manager.on_amcl(amcl)
    assert manager.precheck() is False
