"""The localization probe must not start motion-capable task nodes."""

import subprocess


def launch_nodes(*arguments):
    result = subprocess.run(
        ["roslaunch", "--nodes", "spark_course_mission",
         "localization_probe.launch", *arguments],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    return set(result.stdout.splitlines())


def test_default_probe_uses_map_and_amcl_without_opening_devices():
    nodes = launch_nodes()
    assert "/map_server" in nodes
    assert "/amcl" in nodes
    assert "/robot_state_publisher" in nodes
    assert "/spark_base_server" not in nodes
    assert "/ydlidar_lidar_publisher" not in nodes
    assert "/move_base" not in nodes


def test_explicit_sensor_probe_still_excludes_motion_and_arm_nodes():
    nodes = launch_nodes("start_base:=true", "start_lidar:=true")
    assert "/spark_base_server" in nodes
    assert "/ydlidar_lidar_publisher" in nodes
    assert "/amcl" in nodes
    assert "/move_base" not in nodes
    assert "/spark_mission_manager" not in nodes
    assert "/spark_mission_swiftpro_adapter" not in nodes
    assert not any(node.startswith("/camera/") for node in nodes)
