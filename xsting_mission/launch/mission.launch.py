"""노트북 통합 미션 실행 런치 파일: mission_bridge + robot_command_server (+ llm_commander)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(context, name):
    val = LaunchConfiguration(name).perform(context).strip().lower()
    return val in ("true", "1", "yes", "on")


def _setup(context):
    share = get_package_share_directory("xsting_mission")
    backend = LaunchConfiguration("backend").perform(context).strip()
    nav_mode = LaunchConfiguration("nav_mode").perform(context).strip()
    nav_action = LaunchConfiguration("nav_action").perform(context).strip()
    base_frame = LaunchConfiguration("base_frame").perform(context).strip()
    decider = LaunchConfiguration("decider").perform(context).strip()
    model = LaunchConfiguration("model").perform(context).strip()
    effort = LaunchConfiguration("effort").perform(context).strip()
    run_commander = _is_true(context, "commander")
    run_bridge = _is_true(context, "bridge")
    guard = _is_true(context, "guard")
    record_media = _is_true(context, "record_media")
    fallback_pose_raw = LaunchConfiguration(
        "fallback_pose_when_no_tf").perform(context).strip().lower()
    if fallback_pose_raw == "auto":
        fallback_pose = nav_mode in ("manual", "stub")
    else:
        fallback_pose = fallback_pose_raw in ("true", "1", "yes", "on")

    bridge_cfg = os.path.join(share, "config", "mission_bridge.yaml")
    arena_cfg = os.path.join(share, "config", "arena.yaml")
    nodes = []

    if backend == "ros" and run_bridge:
        nodes.append(
            Node(
                package="xsting_mission",
                executable="mission_bridge",
                name="mission_bridge",
                parameters=[
                    bridge_cfg,
                    {
                        "nav_mode": nav_mode,
                        "nav_action": nav_action,
                        "base_frame": base_frame,
                    },
                ],
                output="screen",
            )
        )

    nodes.append(
        Node(
            package="xsting_mission",
            executable="robot_command_server",
            name="robot_command_server",
            parameters=[
                {
                    "backend": backend,
                    "arena_file": arena_cfg,
                    "base_frame": base_frame,
                    "fallback_pose_when_no_tf": fallback_pose,
                }
            ],
            output="screen",
        )
    )

    if run_commander:
        cmd_params = {
            "decider": decider,
            "arena_file": arena_cfg,
            "guard": guard,
            "record_media": record_media,
        }
        if model:
            cmd_params["model"] = model
        if effort:
            cmd_params["effort"] = effort
        nodes.append(
            Node(
                package="xsting_mission",
                executable="llm_commander",
                name="llm_commander",
                parameters=[cmd_params],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="ros"),
        DeclareLaunchArgument("bridge", default_value="true"),
        DeclareLaunchArgument("nav_mode", default_value="manual"),
        DeclareLaunchArgument("nav_action", default_value="/nav/run"),
        DeclareLaunchArgument("base_frame", default_value="base"),
        DeclareLaunchArgument("fallback_pose_when_no_tf", default_value="auto"),
        DeclareLaunchArgument("commander", default_value="false"),
        DeclareLaunchArgument("decider", default_value="llm"),
        DeclareLaunchArgument("guard", default_value="true"),
        DeclareLaunchArgument("record_media", default_value="true"),
        DeclareLaunchArgument("model", default_value=""),
        DeclareLaunchArgument("effort", default_value=""),
        OpaqueFunction(function=_setup),
    ])
