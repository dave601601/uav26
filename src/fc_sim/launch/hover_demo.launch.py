"""Open-loop hover demo: brings up the full sim + auto-publishes a hover
setpoint so the drone lifts off on its own.

Equivalent to running these two commands in separate terminals:
    ros2 launch world sim.launch.py
    ros2 topic pub --rate 100 /fc/setpoint fc_sim_msgs/msg/Setpoint '{...}'

Single command:
    ros2 launch fc_sim hover_demo.launch.py
    ros2 launch fc_sim hover_demo.launch.py thrust_norm:=0.55
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    thrust = LaunchConfiguration("thrust_norm").perform(context)
    delay = float(LaunchConfiguration("settle_delay").perform(context))

    setpoint_yaml = (
        "{"
        "mode: 1, arm: true, "
        "roll_sp: 0.0, pitch_sp: 0.0, yawrate_sp: 0.0, "
        "vz_sp: 0.0, "
        f"thrust_norm: {thrust}"
        "}"
    )

    return [
        TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "topic", "pub", "--rate", "100",
                        "/fc/setpoint",
                        "fc_sim_msgs/msg/Setpoint",
                        setpoint_yaml,
                    ],
                    output="screen",
                ),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_world = get_package_share_directory("world")

    thrust_arg = DeclareLaunchArgument(
        "thrust_norm",
        default_value="0.51",
        description="Hover thrust_norm (0..1). ~0.50 is hover in sim.",
    )
    delay_arg = DeclareLaunchArgument(
        "settle_delay",
        default_value="3.0",
        description="Seconds to wait for gz to come up before publishing.",
    )

    # Bring up gz + bridge + fc_sim_node + marker_randomize.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_world, "launch", "sim.launch.py")
        ),
    )

    # ros2 topic pub stays alive until the launch is killed, so Ctrl-C
    # tears down both gz and the publisher.
    hover = OpaqueFunction(function=_launch_setup)

    return LaunchDescription([thrust_arg, delay_arg, sim, hover])
