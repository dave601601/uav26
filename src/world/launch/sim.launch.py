"""UAV26 Gazebo Harmonic 시뮬 launch.

기능:
  1. GZ_SIM_RESOURCE_PATH 에 본 패키지 share/<pkg>/models 추가 → model:// 해석
  2. ros_gz_sim 의 gz_sim.launch.py 를 include 해 Gazebo 서버+GUI 기동
  3. ros_gz_bridge parameter_bridge 로 토픽 매핑 (config/bridge.yaml)
  4. 가짜 FC 활성화 메시지 (gz topic /uav26_quad/enable=true) 1회 publish

Launch arguments:
  world          : SDF 파일명 (기본 competition.sdf)
  gui            : true / false (기본 true)
  headless       : 'true' 면 -s (서버 only)
  use_sim_time   : ROS 노드들이 /clock 사용 (기본 true)
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_world = get_package_share_directory("world")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="competition.sdf",
        description="SDF world filename (under share/world/worlds)",
    )
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true", description="Show Gazebo GUI"
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Server-only mode (overrides gui when true)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    # gz-sim 이 model:// 을 찾을 수 있도록 본 패키지 모델 경로를 추가.
    set_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_world, "models")
    )
    set_resource_path_parent = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", pkg_world
    )

    world_path = PathJoinSubstitution(
        [pkg_world, "worlds", LaunchConfiguration("world")]
    )

    # gz-sim 인자:
    #   -r : 시작 시 자동 unpause
    #   -s : server only (GUI 안 띄움) — DISPLAY 없는 환경에서 필수
    #   -v 3 : 로그 레벨
    gz_args_with_gui = [world_path, " -r -v 3"]
    gz_args_headless = [world_path, " -s -r -v 3"]

    headless_cond = PythonExpression(
        ["'", LaunchConfiguration("headless"), "' == 'true'"]
    )
    gui_cond = PythonExpression(
        ["'", LaunchConfiguration("headless"), "' != 'true' and '",
         LaunchConfiguration("gui"), "' == 'true'"]
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_args_with_gui,
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(gui_cond),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_args_headless,
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(headless_cond),
    )

    # headless 도 gui 도 아닌 경우 (gui:=false, headless:=false) → server only.
    gz_sim_server_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_args_headless,
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(
            PythonExpression(
                ["'", LaunchConfiguration("headless"), "' == 'true' or '",
                 LaunchConfiguration("gui"), "' == 'true'"]
            )
        ),
    )

    bridge_yaml = os.path.join(pkg_world, "config", "bridge.yaml")
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_yaml,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # 가짜 FC enable: gz topic 으로 Boolean=true 1회 publish.
    # MulticopterVelocityControl 는 enable=true 받기 전엔 hover 안 함.
    enable_fc = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "gz",
                    "topic",
                    "-t",
                    "/uav26_quad/enable",
                    "-m",
                    "gz.msgs.Boolean",
                    "-p",
                    "data: true",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            gui_arg,
            headless_arg,
            use_sim_time_arg,
            set_resource_path,
            set_resource_path_parent,
            gz_sim_gui,
            gz_sim_headless,
            gz_sim_server_only,
            bridge_node,
            enable_fc,
        ]
    )
