import importlib.util
from pathlib import Path
import pytest
import yaml
from launch.actions import LogInfo
ROOT = Path(__file__).resolve().parents[1]


def module():
    spec = importlib.util.spec_from_file_location("robot_launch", ROOT / "launch/robot.launch.py")
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def config():
    return yaml.safe_load((ROOT / "config/contracts.yaml").read_text())


def test_startup_does_not_claim_live_robot_is_ready():
    c = config()
    assert c["hardware_ready"] is False and c["model_ready"] is False
    actions = module().inspection_actions("check", c)
    assert actions and all(isinstance(a, LogInfo) for a in actions)


@pytest.mark.parametrize("mode", ["robot", "sim", "unexpected"])
def test_unimplemented_mode_does_not_activate_devices(mode):
    with pytest.raises(RuntimeError):
        module().inspection_actions(mode, config())


def test_input_and_sensor_qos_are_not_confused():
    q = config()["qos"]
    assert q["detections"] == dict(reliability="best_effort", durability="volatile", depth=1)
    for name in ["arm_input", "arm_target"]:
        assert q[name] == dict(reliability="reliable", durability="volatile", depth=1)


def test_bounded_pending_work():
    assert config()["queues"] == dict(waiting_frames=1, waiting_arm_inputs=1)
    assert config()["workers"]["arm_inference"] == 1
