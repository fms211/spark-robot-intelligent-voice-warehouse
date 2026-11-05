"""Voice recognition text must map only to explicit mission commands."""
import importlib

import pytest


def intent_module():
    try:
        return importlib.import_module("spark_course_mission.intent")
    except ImportError as exc:
        pytest.fail("mission intent parser is missing: {}".format(exc))


@pytest.mark.parametrize("spoken, expected", [
    ("开始巡检", "START"),
    ("开 始 巡 检", "START"),
    ("请开始巡检。", "START"),
    ("启动巡检", "START"),
    ("停止任务", "STOP"),
    ("停 止 巡 检", "STOP"),
    ("取消任务！", "STOP"),
    ("返回起点", "RETURN"),
    ("回到起点", "RETURN"),
    ("查询状态", "STATUS"),
])
def test_explicit_chinese_commands(spoken, expected):
    assert intent_module().parse_intent(spoken) == expected


@pytest.mark.parametrize("spoken", [
    "",
    "开始",
    "不要开始巡检",
    "现在开始巡检吗",
    "巡检结束了吗",
    "停止音乐",
    "放下物体",
    "start patrol",
])
def test_other_speech_cannot_start_or_stop_mission(spoken):
    assert intent_module().parse_intent(spoken) is None


def test_speech_command_is_an_argument_array():
    text = "巡检完成; echo unsafe"
    assert intent_module().speech_argv(text) == ["ekho", text]


@pytest.mark.parametrize('spoken, expected', [
    ('嗯停止任务', 'STOP'),
    ('嗯嗯，请停止任务。', 'STOP'),
    ('嗯不要停止任务', None),
    ('嗯开始巡检', None),
    ('停止任务吗', None),
])
def test_stop_with_leading_hesitation_does_not_expand_other_commands(spoken, expected):
    assert intent_module().parse_intent(spoken) == expected
