"""Closed-loop hover demo. Brings up gz + bridge + fc_sim_node, then
spawns hover_pub.py which P-controls thrust_norm against /odom_truth.z
so the drone holds a target altitude instead of climbing forever.

    ros2 launch fc_sim hover_demo.launch.py
    ros2 launch fc_sim hover_demo.launch.py target_altitude:=3.0
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

    target_alt_arg = DeclareLaunchArgument(
        "target_altitude",
        default_value="2.0",
        description="Altitude the closed-loop hover_pub holds [m].",
    )
    hover_thrust_arg = DeclareLaunchArgument(
        "hover_thrust_norm",
        default_value="0.500",
        description="Feed-forward hover thrust_norm (0..1).",
    )
    kp_alt_arg = DeclareLaunchArgument(
        "kp_alt",
        default_value="0.04",
        description="P gain on altitude error (thrust_norm per metre).",
    )
    delay_arg = DeclareLaunchArgument(
        "settle_delay",
        default_value="0.5",
        description="Seconds to wait for gz to come up before publishing.",
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_world, "launch", "sim.launch.py")
        ),
    )

    hover_node = TimerAction(
        period=LaunchConfiguration("settle_delay"),
        actions=[
            Node(
                package="fc_sim",
                executable="hover_pub.py",
                name="hover_pub",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "target_altitude": LaunchConfiguration("target_altitude"),
                    "hover_thrust_norm": LaunchConfiguration("hover_thrust_norm"),
                    "kp_alt": LaunchConfiguration("kp_alt"),
                }],
            ),
        ],
    )

    return LaunchDescription([
        target_alt_arg,
        hover_thrust_arg,
        kp_alt_arg,
        delay_arg,
        sim,
        hover_node,
    ])
