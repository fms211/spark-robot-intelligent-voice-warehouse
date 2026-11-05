"""Exercise the installed voice_03 VAD callback without a microphone."""
from collections import deque
from queue import Queue
import sys
from pathlib import Path

import pytest
import rospkg

try:
    voice_scripts = Path(rospkg.RosPack().get_path('voice_03')) / 'scripts'
except rospkg.ResourceNotFound:
    pytest.skip('voice_03 workspace is not sourced', allow_module_level=True)
sys.path.insert(0, str(voice_scripts))
import lib.microphone as existing_microphone
from spark_course_mission.voice_vad import make_course_microphone


def test_speech_resets_trailing_silence_timer(monkeypatch):
    cls = make_course_microphone(existing_microphone.Microphone)
    mic = cls.__new__(cls)
    mic.status = mic.listening_mask
    mic.active = False
    mic.listen_history = deque(maxlen=8)
    mic.listen_queue = Queue()
    mic.listen_countdown = [282, 62.5]
    mic._course_silence_frames = 62.5
    active = {'speech': False}
    monkeypatch.setattr(existing_microphone.vad, 'is_speech', lambda _: active['speech'])

    def frame(speech):
        active['speech'] = speech
        mic._callback(b'x', 512, None, None)

    for _ in range(50):
        frame(False)
    frame(True)
    for _ in range(13):
        frame(False)
    assert mic.status & mic.listening_mask
    for _ in range(50):
        frame(False)
    assert not (mic.status & mic.listening_mask)
