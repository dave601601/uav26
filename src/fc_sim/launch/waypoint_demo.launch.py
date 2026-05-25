"""Waypoint demo: sim + fc_sim_node + waypoint_demo_pub in one launch.

Examples:
    ros2 launch fc_sim waypoint_demo.launch.py pattern:=forward_back
    ros2 launch fc_sim waypoint_demo.launch.py pattern:=box headless:=true
    ros2 launch fc_sim waypoint_demo.launch.py pattern:=altitude
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

    pattern_arg = DeclareLaunchArgument(
        "pattern",
        default_value="forward_back",
        description="Waypoint pattern: hover, forward_back, box, altitude.",
    )
    settle_delay_arg = DeclareLaunchArgument(
        "settle_delay",
        default_value="0.5",
        description="Seconds to wait before publishing setpoints.",
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_world, "launch", "sim.launch.py")
        ),
    )

    wp_node = TimerAction(
        period=LaunchConfiguration("settle_delay"),
        actions=[
            Node(
                package="fc_sim",
                executable="waypoint_demo_pub.py",
                name="waypoint_demo_pub",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "pattern": LaunchConfiguration("pattern"),
                }],
            ),
        ],
    )

    return LaunchDescription([
        pattern_arg,
        settle_delay_arg,
        sim,
        wp_node,
    ])
