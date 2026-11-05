"""The detached calibration prompt must never publish a repeating start signal."""
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'calibration_confirm.sh'


def prepare(tmp_path, failing_node=''):
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    log = tmp_path / 'calls'
    rosnode = bindir / 'rosnode'
    rosnode.write_text('#!/bin/sh\necho "node $*" >> "$TEST_CALLS"\ncase "$*" in *"$FAIL_NODE"*) [ -z "$FAIL_NODE" ] && exit 0 || exit 1;; esac\nexit 0\n')
    rostopic = bindir / 'rostopic'
    rostopic.write_text('#!/bin/sh\necho "topic $*" >> "$TEST_CALLS"\n')
    rosnode.chmod(0o755)
    rostopic.chmod(0o755)
    return dict(os.environ, PATH=str(bindir) + ':' + os.environ['PATH'],
                TEST_CALLS=str(log), FAIL_NODE=failing_node), log


def test_confirmation_publishes_start_once_after_all_nodes_are_ready(tmp_path):
    env, log = prepare(tmp_path)
    result = subprocess.run(['bash', str(SCRIPT)], input='\n', env=env,
                            text=True, capture_output=True)
    assert result.returncode == 0
    calls = log.read_text().splitlines()
    assert sum(line.startswith('node ') for line in calls) == 3
    assert sum(line.startswith('topic ') for line in calls) == 1
    assert 'pub -1 /start_topic' in calls[-1]


def test_confirmation_refuses_if_arm_driver_is_absent(tmp_path):
    env, log = prepare(tmp_path, '/swiftpro_write_node')
    result = subprocess.run(['bash', str(SCRIPT)], input='\n', env=env,
                            text=True, capture_output=True)
    assert result.returncode != 0
    assert not any(line.startswith('topic ') for line in log.read_text().splitlines())
