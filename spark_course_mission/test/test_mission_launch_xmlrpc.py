"""The real launch parameter set must be accepted by a ROS XML-RPC master."""
import subprocess
import xmlrpc.client

import pytest
import yaml


@pytest.mark.parametrize('robot,arm', [('false','mock'),('true','mock'),('false','real'),('true','real')])
def test_mission_launch_parameters_are_xmlrpc_serializable(robot, arm):
    output = subprocess.run(
        ['roslaunch', '--dump-params', 'spark_course_mission', 'mission.launch',
         'enable_robot:='+robot, 'arm_mode:='+arm],
        text=True, capture_output=True, check=True,
    )
    parameters = yaml.safe_load(output.stdout)
    xmlrpc.client.dumps((parameters,))
    prefix = '/spark_mission_manager/mission_config/'
    assert parameters[prefix + 'valid_waypoints'] is False
    assert parameters[prefix + 'stations'] == []
    assert not parameters.get(prefix + 'home_pose')
    assert not parameters.get(prefix + 'drop_pose')
    assert parameters['/spark_mission_manager/alignment_calibration/valid'] is False
    assert parameters['/spark_mission_color_inspector/roi'] == [0, 0, 640, 480]
    if arm == 'real':
        assert parameters['/spark_mission_swiftpro_adapter/calibration/valid'] is False


def test_mission_launch_inspection_roi_override():
    output = subprocess.run(
        ['roslaunch', '--dump-params', 'spark_course_mission', 'mission.launch',
         'inspection_roi_x0:=102', 'inspection_roi_y0:=186',
         'inspection_roi_x1:=327', 'inspection_roi_y1:=293'],
        text=True, capture_output=True, check=True,
    )
    parameters = yaml.safe_load(output.stdout)
    assert parameters['/spark_mission_color_inspector/roi'] == [102, 186, 327, 293]
