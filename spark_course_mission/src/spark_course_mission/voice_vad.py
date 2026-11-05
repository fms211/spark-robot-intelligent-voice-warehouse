"""Narrow adaptation of the existing voice_03 silence timeout."""


def make_course_microphone(base_type):
    class CourseMicrophone(base_type):
        def listen(self, duration=9, timeout=2):
            self._course_silence_frames = (
                (timeout * self.sample_rate + self.frames_per_buffer - 1)
                / self.frames_per_buffer
            )
            return super().listen(duration=duration, timeout=timeout)

        def _callback(self, in_data, frame_count, time_info, status):
            result = super()._callback(in_data, frame_count, time_info, status)
            if (self.status & self.listening_mask) and self.active:
                self.listen_countdown[1] = self._course_silence_frames
            return result

    return CourseMicrophone
