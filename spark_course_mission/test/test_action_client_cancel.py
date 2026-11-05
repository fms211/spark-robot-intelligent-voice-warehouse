"""A stop request must not cancel already completed ROS actions."""
import importlib.util
from pathlib import Path
import threading

from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
import rospy
from spark_course_mission.msg import PickPlaceResult


def manager_script():
    path = Path(__file__).parents[1] / "scripts" / "mission_manager.py"
    spec = importlib.util.spec_from_file_location("spark_mission_manager_script", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeActionClient:
    def __init__(self, state):
        self.state = state
        self.cancels = 0

    def get_state(self):
        return self.state

    def cancel_goal(self):
        self.cancels += 1


def test_finished_navigation_is_not_canceled_again():
    wrapper = manager_script().NavigationClient.__new__(manager_script().NavigationClient)
    wrapper.client = FakeActionClient(GoalStatus.SUCCEEDED)
    wrapper.cancel_goal()
    assert wrapper.client.cancels == 0


def test_finished_arm_action_is_not_canceled_again():
    wrapper = manager_script().ArmClient.__new__(manager_script().ArmClient)
    wrapper.client = FakeActionClient(GoalStatus.SUCCEEDED)
    wrapper.cancel_goal()
    assert wrapper.client.cancels == 0


def test_active_navigation_is_canceled():
    wrapper = manager_script().NavigationClient.__new__(manager_script().NavigationClient)
    wrapper.client = FakeActionClient(GoalStatus.ACTIVE)
    wrapper.cancel_goal()
    assert wrapper.client.cancels == 1


def test_stop_sends_zero_to_navigation_smoother_input():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    wrapper = manager_script().NavigationClient.__new__(manager_script().NavigationClient)
    wrapper.stop_pub = Publisher()
    wrapper.stop()
    assert len(wrapper.stop_pub.messages) == 1
    message = wrapper.stop_pub.messages[0]
    assert message.linear.x == 0.0
    assert message.angular.z == 0.0


def test_odom_header_time_is_passed_to_stationarity_gate():
    class Controller:
        def __init__(self):
            self.stamps = []

        def on_base_velocity(self, linear, angular, stamp):
            self.stamps.append(stamp)
            return False

    manager = manager_script().MissionManager.__new__(manager_script().MissionManager)
    manager.lock = threading.RLock()
    manager.controller = Controller()
    manager.last_odom = None
    message = Odometry()
    message.header.stamp = rospy.Time.from_sec(100)
    manager.on_odom(message)
    assert manager.controller.stamps == [message.header.stamp.to_sec()]
    assert manager.last_odom is None


def test_preempted_arm_result_carries_suction_state_to_mission():
    class Client:
        def send_goal(self, goal, done_cb):
            self.done_cb = done_cb

    script = manager_script()
    arm = script.ArmClient.__new__(script.ArmClient)
    arm.client = Client()
    received = []
    arm.on_done = lambda *args: received.append(args)
    arm.send_goal("PICK", "pickup_b", 320.0, 240.0)
    result = PickPlaceResult(success=False, carrying=True,
                             code="CANCELED_CARRYING", detail="pump held")
    arm.client.done_cb(GoalStatus.PREEMPTED, result)
    assert received == [(False, "CANCELED_CARRYING", True)]


def test_ros_odometry_callback_trips_course_controller_during_pick():
    from test_mission_state import make_controller, to_pickup

    controller, clock, nav, arm, _, _, _ = make_controller()
    to_pickup(controller, clock, nav)
    manager = manager_script().MissionManager.__new__(manager_script().MissionManager)
    manager.lock = threading.RLock()
    manager.controller = controller
    manager.last_odom = None
    message = Odometry()
    message.header.stamp = rospy.Time.from_sec(clock())
    message.twist.twist.linear.x = -36.04
    manager.on_odom(message)
    assert controller.state == 'SAFE_STOP'
    assert controller.odom_fault_latched
    assert nav.cancels == nav.stops == arm.cancels == 1
    assert manager.last_odom is None


def test_navigation_requests_refresh_and_waits_for_stale_pose():
    from geometry_msgs.msg import PoseWithCovarianceStamped
    script = manager_script()
    nav = script.NavigationClient.__new__(script.NavigationClient)
    rospy.rostime.set_rostime_initialized(True)
    message = PoseWithCovarianceStamped()
    message.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(3)
    nav.pose_source = lambda: message
    requests = []
    nav.refresh_source = lambda: requests.append(True)
    assert nav.ensure_pose_ready() is False
    assert requests == [True]
    message.header.stamp = rospy.Time.now()
    assert nav.ensure_pose_ready() is True
    assert requests == [True]


def test_refresh_request_requires_still_base_fresh_scan_and_no_fault(monkeypatch):
    script = manager_script()
    rospy.rostime.set_rostime_initialized(True)
    manager = script.MissionManager.__new__(script.MissionManager)
    manager._amcl_refresh_lock = threading.Lock()
    manager._last_amcl_refresh_attempt = 0.0
    manager.last_scan = rospy.Time.now().to_sec()
    manager.controller = type('Controller', (), {
        '_stationary': lambda self, now: False,
        'odom_fault_latched': False,
    })()
    started = []
    class FakeThread:
        def __init__(self, target):
            self.target = target
            self.daemon = False
        def start(self):
            started.append(self.target)
    monkeypatch.setattr(script.threading, 'Thread', FakeThread)
    manager._queue_amcl_refresh()
    assert started == []
    manager.controller._stationary = lambda now: True
    manager.last_scan = rospy.Time.now().to_sec() - 1.0
    manager._queue_amcl_refresh()
    assert started == []
    manager.last_scan = rospy.Time.now().to_sec()
    manager.controller.odom_fault_latched = True
    manager._queue_amcl_refresh()
    assert started == []
    manager.controller.odom_fault_latched = False
    manager._queue_amcl_refresh()
    assert len(started) == 1
    manager._queue_amcl_refresh()
    assert len(started) == 1
