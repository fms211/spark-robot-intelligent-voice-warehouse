"""The voice-to-grasp bridge must map only the explicit course commands."""
from spark_course_mission.voice_grasp import command_to_grasp


def test_start_maps_to_grasp():
    assert command_to_grasp("START") == "1"


def test_stop_maps_to_release():
    assert command_to_grasp("STOP") == "0"


def test_return_maps_to_release():
    assert command_to_grasp("RETURN") == "0"


def test_status_and_unknown_do_not_move_the_arm():
    assert command_to_grasp("STATUS") is None
    assert command_to_grasp("") is None
    assert command_to_grasp("start") is None
    assert command_to_grasp("任意自然语言") is None
