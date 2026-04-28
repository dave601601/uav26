"""Launch file for line_tracer.

Modes (selected by `sim` argument):
  sim:=true  (default in this competition repo) — assumes the Gazebo world
             from `world` package is running separately; we only spawn
             line_tracer_node and let the ros_gz bridge supply camera topics.
  sim:=false — also includes realsense2_camera with default params.

Combine with:
  use_sim_time:=true  to consume /clock from Gazebo.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_line_tracer = get_package_share_directory("line_tracer")

    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="true",
        description="If true, expect Gazebo world running; skip RealSense.",
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_line_tracer, "config", "params.yaml"),
        description="YAML parameter file for line_tracer_node",
    )
    target_alt_arg = DeclareLaunchArgument(
        "target_altitude",
        default_value="2.0",
        description="Override target altitude [m] (also flows into FSM stub).",
    )

    line_tracer_node = Node(
        package="line_tracer",
        executable="line_tracer_node",
        name="line_tracer_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "target_altitude": LaunchConfiguration("target_altitude"),
            },
        ],
    )

    # Real RealSense path: pull in the wrapper. Skipped in sim mode.
    try:
        pkg_rs = get_package_share_directory("realsense2_camera")
        rs_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_rs, "launch", "rs_launch.py")
            ),
            launch_arguments={
                "align_depth.enable": "true",
                "enable_color": "true",
                "enable_depth": "true",
            }.items(),
        )
        real_group = GroupAction(
            actions=[rs_launch],
            condition=UnlessCondition(LaunchConfiguration("sim")),
        )
    except Exception as e:                                       # pragma: no cover
        real_group = LogInfo(
            msg=f"realsense2_camera not installed; sim:=false will be a no-op ({e})"
        )

    sim_note = LogInfo(
        msg="sim:=true — expecting world/sim.launch.py to be running already",
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    return LaunchDescription(
        [
            sim_arg,
            use_sim_time_arg,
            params_file_arg,
            target_alt_arg,
            sim_note,
            real_group,
            line_tracer_node,
        ]
    )
