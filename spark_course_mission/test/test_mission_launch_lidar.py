"""Check the integrated launch resolves the confirmed physical lidar driver."""

import subprocess


def test_real_robot_launch_selects_g6_by_default():
    result = subprocess.run(
        ["roslaunch", "--nodes", "spark_course_mission", "mission.launch", "enable_robot:=true"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "/ydlidar_lidar_publisher" in result.stdout
    assert "/delta_lidar_node" not in result.stdout
