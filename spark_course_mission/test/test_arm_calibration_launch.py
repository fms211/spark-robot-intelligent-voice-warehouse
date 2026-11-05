"""A measured calibration can be selected without replacing disabled defaults."""
import subprocess
import yaml


def test_real_launch_loads_explicit_field_calibration_without_starting_nodes(tmp_path):
    calibration = tmp_path / 'measured.yaml'
    calibration.write_text('valid: true\ncalibration_mode: field\nmatrix: [[1, 0, 12], [0, 1, 34]]\n')
    result = subprocess.run(
        ['roslaunch', '--dump-params', 'spark_course_mission', 'mission.launch',
         'arm_mode:=real', 'arm_calibration_file:=' + str(calibration)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    params = yaml.safe_load(result.stdout)
    assert params['/spark_mission_swiftpro_adapter/calibration/valid'] is True
    assert params['/spark_mission_swiftpro_adapter/calibration/matrix'] == [[1, 0, 12], [0, 1, 34]]
