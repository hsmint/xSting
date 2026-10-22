"""Scaffold inspection only. Live robot activation is deliberately unimplemented."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml


def inspection_actions(mode, config):
    if mode != "check":
        raise RuntimeError("실기·시뮬레이션 실행 노드는 아직 미구현입니다. 담당자 TODO와 TEST_PLAN을 완료한 뒤 연결하세요.")
    if config.get("schema_version") != 1:
        raise ValueError("unsupported contracts schema")
    return [LogInfo(msg="xSting 구조 점검: 패키지 5개. 모터·카메라·학습 모델은 실행하지 않습니다.")]


def _setup(context):
    config_path = Path(get_package_share_directory("xsting_bringup")) / "config/contracts.yaml"
    return inspection_actions(LaunchConfiguration("mode").perform(context), yaml.safe_load(config_path.read_text()))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="check", description="현재 check만 지원"),
        OpaqueFunction(function=_setup),
    ])
