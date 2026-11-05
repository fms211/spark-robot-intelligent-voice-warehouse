"""The calibration entry reuses existing nodes with the stable arm device."""
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml


def test_existing_calibration_nodes_use_stable_port_and_stop_on_driver_exit():
    proc = subprocess.run(['roslaunch', '--dump-params', 'spark_course_mission',
                           'calibrate_warehouse.launch'], check=True,
                          text=True, capture_output=True)
    params = yaml.safe_load(proc.stdout)
    assert params['/swiftpro_write_node/port'] == '/dev/uarm'
    assert params['/swiftpro_write_node/service_only'] is False
    assert params['/swiftpro_write_node/feedrate_mm_min'] == 3000
    path = Path(__file__).resolve().parents[1] / 'launch' / 'calibrate_warehouse.launch'
    root = ET.parse(str(path)).getroot()
    node = next(n for n in root.iter('node') if n.get('name') == 'swiftpro_write_node')
    assert node.get('required') == 'true'
    for name in ('cali_cam_cv3_node', 'cali_pos_node'):
        other = next(n for n in root.iter('node') if n.get('name') == name)
        assert other.get('required') == 'true'
    prompt = next(n for n in root.iter('node') if n.get('name') == 'calibration_prompt')
    assert prompt.get('type') == 'calibration_prompt.sh'
    nodes = subprocess.run(['roslaunch', '--nodes', 'spark_course_mission',
                            'calibrate_warehouse.launch'], check=True,
                           text=True, capture_output=True).stdout
    assert '/cali_cam_cv3_node' in nodes
    assert '/cali_pos_node' in nodes
    assert '/swiftpro_write_node' in nodes


def test_existing_warehouse_uses_stable_arm_device():
    proc = subprocess.run(['roslaunch', '--dump-params', 'move2grasp',
                           'move2grasp.launch', 'camera_type_tel:=d435',
                           'lidar_type_tel:=ydlidar_g6'], check=True,
                          text=True, capture_output=True)
    params = yaml.safe_load(proc.stdout)
    assert params['/swiftpro_write_node/port'] == '/dev/uarm'
