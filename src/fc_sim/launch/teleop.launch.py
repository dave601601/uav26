"""Sim + fc_sim_node only. Run teleop_pub.py in a separate terminal so
its stdin (keyboard) is not multiplexed through ros2 launch.

    Terminal A:  ros2 launch fc_sim teleop.launch.py
    Terminal B:  ros2 run fc_sim teleop_pub.py
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    pkg_world = get_package_share_directory("world")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_world, "launch", "sim.launch.py")
        ),
    )
    return LaunchDescription([sim])
