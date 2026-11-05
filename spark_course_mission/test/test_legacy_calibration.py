import math
import pytest
from spark_course_mission.legacy_calibration import convert_legacy_calibration


def test_legacy_equations_are_preserved_and_result_is_disabled():
    result = convert_legacy_calibration('-0.9 481 -0.8 298')
    assert result['valid'] is False
    assert result['calibration_mode'] == 'uncalibrated'
    matrix = result['matrix']
    u, v = 300.0, 260.0
    assert matrix[0][0]*u + matrix[0][1]*v + matrix[0][2] == pytest.approx(-0.9*v + 481)
    assert matrix[1][0]*u + matrix[1][1]*v + matrix[1][2] == pytest.approx(-0.8*u + 298)
    assert result['pixel_roi'] == {}
    assert result['heights_mm'] == {}


@pytest.mark.parametrize('text', ['', '1 2 3', '1 2 3 4 5', '1 nan 3 4', 'inf 2 3 4', '1 x 3 4'])
def test_invalid_legacy_data_is_rejected(text):
    with pytest.raises(ValueError):
        convert_legacy_calibration(text)


def test_cli_refuses_to_overwrite_existing_file(tmp_path):
    import os
    import subprocess
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    source, output = tmp_path / 'thefile.txt', tmp_path / 'candidate.yaml'
    source.write_text('1 2 3 4')
    output.write_text('do not overwrite')
    env = dict(os.environ, PYTHONPATH=str(root / 'src'))
    run = subprocess.run([sys.executable, str(root / 'scripts/import_move2grasp_calibration.py'), str(source), str(output)], env=env, capture_output=True)
    assert run.returncode != 0
    assert output.read_text() == 'do not overwrite'
