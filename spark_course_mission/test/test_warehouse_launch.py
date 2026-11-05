"""Warehouse entry resolves the existing mission and only one selected ASR."""
import subprocess
import xmlrpc.client
import yaml


def run_launch(option, *args):
    return subprocess.run(['roslaunch', option, 'spark_course_mission',
                           'warehouse.launch', *args], check=True,
                          text=True, capture_output=True).stdout


def test_default_is_mock_and_has_no_microphone_or_hardware():
    nodes = run_launch('--nodes')
    assert '/spark_mission_manager' in nodes
    assert '/spark_mission_voice_intent' in nodes
    assert '/spark_warehouse_asr' not in nodes
    assert '/spark_base_nodelet_manager' not in nodes
    assert '/swiftpro_write_node' not in nodes
    params = yaml.safe_load(run_launch('--dump-params'))
    xmlrpc.client.dumps((params,))
    assert params['/spark_mission_manager/mission_config/valid_waypoints'] is False


def test_optional_asr_and_robot_arguments_resolve_without_starting_nodes():
    nodes = run_launch('--nodes', 'enable_asr:=true', 'enable_robot:=true')
    assert nodes.count('/spark_warehouse_asr') == 1
    assert '/ydlidar_lidar_publisher' in nodes
    assert '/spark_mission_manager' in nodes


def test_asr_uses_system_python_for_existing_voice_dependencies():
    from pathlib import Path
    import xml.etree.ElementTree as ET
    launch = Path(__file__).resolve().parents[1] / 'launch' / 'warehouse.launch'
    root = ET.parse(str(launch)).getroot()
    asr = next(node for node in root.iter('node') if node.get('name') == 'spark_warehouse_asr' and node.get('pkg') == 'spark_course_mission')
    assert asr.get('launch-prefix') == '/usr/bin/python3'
    assert asr.get('type') == 'course_voice_asr.py'
