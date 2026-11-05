"""Exercise calibration timing with a simulated ROS clock and no hardware."""
import importlib.util
from pathlib import Path
import sys
import types


def test_first_calibration_sample_waits_full_twenty_seconds(monkeypatch):
    script = Path(__file__).resolve().parents[1] / 'scripts' / 'calibration_position.py'
    now = [0.0]
    events = []
    class Sampled(Exception):
        pass
    class Rate:
        def __init__(self, hz):
            self.period = 1.0 / hz
            self.last = now[0]
        def sleep(self):
            now[0] = max(now[0], self.last + self.period)
            self.last += self.period
            if now[0] - self.last > 2 * self.period:
                self.last = now[0]
    class Publisher:
        def __init__(self, topic, *args, **kwargs):
            self.topic = topic
        def publish(self, value):
            if self.topic == 'position_write_topic':
                events.append((value.x, value.y, now[0]))
            if self.topic == 'cali_pix_topic':
                events.append(('sample', now[0]))
                raise Sampled
    ros = types.ModuleType('rospy')
    ros.Rate = Rate
    ros.Publisher = Publisher
    ros.Subscriber = lambda *args, **kwargs: None
    ros.sleep = lambda seconds: now.__setitem__(0, now[0] + seconds)
    ros.is_shutdown = lambda: False
    std = types.ModuleType('std_msgs.msg')
    std.String = type('String', (), {})
    carry = types.ModuleType('spark_carry_object.msg')
    carry.position = type('position', (), {})
    carry.status = type('status', (), {})
    for name, mod in [('rospy', ros), ('std_msgs', types.ModuleType('std_msgs')),
                      ('std_msgs.msg', std), ('spark_carry_object', types.ModuleType('spark_carry_object')),
                      ('spark_carry_object.msg', carry)]:
        monkeypatch.setitem(sys.modules, name, mod)
    spec = importlib.util.spec_from_file_location('calibration_position_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.start_cali = 1
    try:
        module.talker('test', 0)
    except Sampled:
        pass
    else:
        raise AssertionError('first sample was not published')
    first_point = next(event for event in events if event[0] == 180)
    sample = next(event for event in events if event[0] == 'sample')
    assert sample[1] - first_point[2] >= 20.0
