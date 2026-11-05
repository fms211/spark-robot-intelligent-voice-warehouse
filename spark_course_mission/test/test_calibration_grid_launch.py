"""The 2D calibration launch must stay service-only and avoid legacy motion nodes."""
from pathlib import Path
import xml.etree.ElementTree as ET


def _root():
    launch = Path(__file__).resolve().parents[1] / "launch/calibrate_grid.launch"
    return ET.parse(str(launch)).getroot()


def test_swiftpro_node_is_service_only_and_slow():
    root = _root()
    node = root.find("./node[@name='swiftpro_write_node']")
    assert node is not None
    params = {param.get("name"): param.get("value") for param in node.findall("param")}
    assert params["service_only"] == "true"
    assert params["feedrate_mm_min"] == "$(arg feedrate_mm_min)"
    assert params["position_tolerance_mm"] == "2.5"
    assert params["port"] == "$(arg port)"


def test_grid_script_and_camera_are_launched():
    root = _root()
    node = root.find("./node[@type='calibration_grid_run.py']")
    assert node is not None
    params = {param.get("name"): param.get("value") for param in node.findall("param")}
    assert params["auto_start"] == "false"
    assert params["output_yaml"] == "$(arg output_yaml)"
    assert params["samples_json"] == "$(arg samples_json)"
    assert node.find("./env[@name='DISPLAY']") is not None
    includes = [include.get("file") for include in root.findall("include")]
    assert any("d435.launch" in item for item in includes)


def test_legacy_and_chassis_nodes_are_absent():
    root = _root()
    types = [node.get("type") for node in root.iter("node")]
    includes = [include.get("file") for include in root.findall("include")]
    assert "calibration_position.py" not in types
    assert "cali_cam_cv3.py" not in types
    assert "mission_manager.py" not in types
    assert "swiftpro_adapter.py" not in types
    assert not any("spark_base" in item for item in includes)
    assert not any("move_base" in item for item in includes)
