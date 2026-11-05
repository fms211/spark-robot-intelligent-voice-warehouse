#!/usr/bin/env python3
"""Run voice_03 ASR with the course silence-timeout correction."""
import os
import sys
import rospkg

voice_scripts = os.path.join(rospkg.RosPack().get_path('voice_03'), 'scripts')
sys.path.insert(0, voice_scripts)
import fms_asr
from spark_course_mission.voice_vad import make_course_microphone

if __name__ == '__main__':
    fms_asr.Microphone = make_course_microphone(fms_asr.Microphone)
    fms_asr.main()
