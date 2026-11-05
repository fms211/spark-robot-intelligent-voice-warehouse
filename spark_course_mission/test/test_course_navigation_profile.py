"""Read effective launch parameters, without opening any robot devices."""
import math
import subprocess
import yaml
import pytest


@pytest.fixture(scope='module')
def nav_params():
    p = subprocess.run(
        ['roslaunch', '--dump-params', 'spark_course_mission', 'mission.launch',
         'enable_robot:=true', 'arm_mode:=mock'],
        capture_output=True, text=True, check=True)
    return yaml.safe_load(p.stdout)


def test_course_navigation_limits_both_translation_and_rotation(nav_params):
    p = nav_params
    ns = '/move_base/DWAPlannerROS/'
    assert 0 < p[ns + 'max_vel_x'] <= 0.25
    assert 0 < p[ns + 'max_vel_trans'] <= 0.25
    assert 0 < p[ns + 'max_vel_theta'] <= 0.6
    assert 0 < p[ns + 'min_vel_trans'] < p[ns + 'max_vel_trans']
    assert 0 < p[ns + 'min_vel_theta'] < p[ns + 'max_vel_theta']
    assert p[ns + 'min_vel_x'] >= 0
    # A 180-degree turn at the course rotation limit must fit the timeout.
    assert p['/move_base/oscillation_timeout'] >= math.pi / p[ns + 'max_vel_theta']


def test_course_navigation_does_not_schedule_unobserved_recovery_rotation(nav_params):
    assert nav_params['/move_base/recovery_behavior_enabled'] is False


def test_course_costmap_encloses_stowed_arm_and_block(nav_params):
    # Model arm base is +44 mm forward. Stowed cup X=180 mm;
    # half block width allowance=30 mm, plus 50 mm project clearance.
    required_radius = 0.044 + 0.180 + 0.030 + 0.050
    assert nav_params['/move_base/local_costmap/robot_radius'] >= required_radius
    assert nav_params['/move_base/global_costmap/robot_radius'] >= required_radius
