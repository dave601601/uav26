"""Standalone launcher for fc_sim_node.

Useful for unit-debugging the FC against synthetic IMU/setpoint topics
without bringing up the full world/ launch. For the full sim, world's
sim.launch.py spawns fc_sim_node directly.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Drive Control() from /clock instead of wall time."),

        Node(
            package="fc_sim",
            executable="fc_sim_node",
            name="fc_sim_node",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
