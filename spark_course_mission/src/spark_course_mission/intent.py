"""Strict mapping from recognized speech text to mission commands."""
from typing import Optional
import unicodedata


_COMMANDS = {
    "开始巡检": "START",
    "启动巡检": "START",
    "停止任务": "STOP",
    "停止巡检": "STOP",
    "取消任务": "STOP",
    "返回起点": "RETURN",
    "回到起点": "RETURN",
    "查询状态": "STATUS",
}


def parse_intent(text: str) -> Optional[str]:
    if not isinstance(text, str):
        return None
    normalized = unicodedata.normalize("NFKC", text)
    normalized = "".join(
        char for char in normalized
        if not unicodedata.category(char).startswith(("Z", "P"))
        and not char.isspace()
    )
    if normalized.startswith("嗯"):
        candidate = normalized.lstrip("嗯")
        if candidate.startswith("请"):
            candidate = candidate[1:]
        # Only a complete, explicit stop command may follow hesitation.
        if _COMMANDS.get(candidate) == "STOP":
            return "STOP"
    if normalized.startswith("请"):
        normalized = normalized[1:]
    return _COMMANDS.get(normalized)


def speech_argv(text: str):
    """Pass status text as one subprocess argument, never through a shell."""
    return ["ekho", text]
