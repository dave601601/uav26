"""Scripted forward/back/roll/yaw demo on top of altitude hold.

Cycles through pitch/roll/yaw commands so each axis is visible in turn.
See scripts/flight_demo_pub.py for the sequence definition.

    ros2 launch fc_sim flight_demo.launch.py
    ros2 launch fc_sim flight_demo.launch.py phase_duration:=8.0 pitch_amp:=0.08
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_world = get_package_share_directory("world")

    target_alt_arg     = DeclareLaunchArgument("target_altitude",  default_value="2.0")
    phase_dur_arg      = DeclareLaunchArgument("phase_duration",   default_value="5.0")
    pitch_amp_arg      = DeclareLaunchArgument("pitch_amp",        default_value="0.05")
    roll_amp_arg       = DeclareLaunchArgument("roll_amp",         default_value="0.05")
    yaw_amp_arg        = DeclareLaunchArgument("yaw_amp",          default_value="0.30")
    hover_thrust_arg   = DeclareLaunchArgument("hover_thrust_norm", default_value="0.333")
    kp_alt_arg         = DeclareLaunchArgument("kp_alt",           default_value="0.02")
    kd_alt_arg         = DeclareLaunchArgument("kd_alt",           default_value="0.067")
    settle_delay_arg   = DeclareLaunchArgument("settle_delay",     default_value="3.0")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_world, "launch", "sim.launch.py")
        ),
    )

    demo_node = TimerAction(
        period=LaunchConfiguration("settle_delay"),
        actions=[
            Node(
                package="fc_sim",
                executable="flight_demo_pub.py",
                name="flight_demo_pub",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "target_altitude":   LaunchConfiguration("target_altitude"),
                    "phase_duration":    LaunchConfiguration("phase_duration"),
                    "pitch_amp":         LaunchConfiguration("pitch_amp"),
                    "roll_amp":          LaunchConfiguration("roll_amp"),
                    "yaw_amp":           LaunchConfiguration("yaw_amp"),
                    "hover_thrust_norm": LaunchConfiguration("hover_thrust_norm"),
                    "kp_alt":            LaunchConfiguration("kp_alt"),
                    "kd_alt":            LaunchConfiguration("kd_alt"),
                }],
            ),
        ],
    )

    return LaunchDescription([
        target_alt_arg, phase_dur_arg, pitch_amp_arg, roll_amp_arg,
        yaw_amp_arg, hover_thrust_arg, kp_alt_arg, kd_alt_arg,
        settle_delay_arg, sim, demo_node,
    ])
