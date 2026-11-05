"""Translate course mission commands to the original move2grasp /grasp topic."""


def command_to_grasp(command):
    """Return '1' (grasp), '0' (release/return), or None (no arm action)."""
    if command == "START":
        return "1"
    if command in ("STOP", "RETURN"):
        return "0"
    return None
