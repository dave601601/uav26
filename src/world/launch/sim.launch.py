"""UAV26 Gazebo Harmonic 시뮬 launch.

기능:
  1. GZ_SIM_RESOURCE_PATH 에 본 패키지 share/<pkg>/models 추가 → model:// 해석
  2. marker_randomize.py 를 pre-launch 로 실행 (4 markers, --seed)
  3. ros_gz_sim 의 gz_sim.launch.py 를 include 해 Gazebo 서버+GUI 기동
  4. ros_gz_bridge parameter_bridge 로 토픽 매핑 (config/bridge.yaml)
  5. fc_sim_node 노드 spawn (fc_core 기반 시뮬 FC)

Launch arguments:
  world          : SDF 파일명 (기본 competition.sdf)
  gui            : true / false (기본 true)
  headless       : 'true' 면 -s (서버 only)
  use_sim_time   : ROS 노드들이 /clock 사용 (기본 true)
  marker_seed    : marker_randomize.py 의 --seed (-1 = 시계 기반)
  mission_cruise : 미션 순항 속도 override, m/s (기본 0.5)
  mission_max_vxy: 미션 xy 속도 클램프 override, m/s (기본 0.8)
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
        default_value="competition_runtime.sdf",
        description=(
            "SDF world filename (under share/world/worlds). The default is the "
            "post-randomization SDF written by marker_randomize.py."
        ),
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
    marker_seed_arg = DeclareLaunchArgument(
        "marker_seed",
        default_value="-1",
        description="Seed for marker_randomize.py (-1 = clock-based)",
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

    # Pre-launch: randomize 4 markers across grid intersections. Writes
    # models/world_assets/markers_runtime.sdf (gitignored) + overwrites
    # config/aruco_layout.yaml so line_tracer reads the same coordinates.
    marker_randomize = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(pkg_world, "script", "marker_randomize.py"),
            "--seed",
            LaunchConfiguration("marker_seed"),
            "--share-dir",
            pkg_world,
        ],
        output="screen",
    )

    # Optional fc_sim_node gain overrides exposed as launch args so the
    # caller can dial down responsiveness without rebuilding.
    # Half-gain defaults: with the sphere body collision the drone can
    # actually rotate on the ground at takeoff, and the firmware-native
    # 0.80 atti gain over-corrects on every contact perturbation which
    # makes DartSim's ODE collision detector explode. The halved gains
    # (0.20 rate / 0.40 atti / 0.20 kd) match what flight_demo +
    # waypoint_demo verified stable in earlier runs.
    rate_kp_p_arg     = DeclareLaunchArgument("rate_kp_p",     default_value="0.20")
    rate_kp_q_arg     = DeclareLaunchArgument("rate_kp_q",     default_value="0.20")
    rate_kp_r_arg     = DeclareLaunchArgument("rate_kp_r",     default_value="0.40")
    atti_kp_roll_arg  = DeclareLaunchArgument("atti_kp_roll",  default_value="0.40")
    atti_kp_pitch_arg = DeclareLaunchArgument("atti_kp_pitch", default_value="0.40")
    atti_kd_roll_arg  = DeclareLaunchArgument("atti_kd_roll",  default_value="0.20")
    atti_kd_pitch_arg = DeclareLaunchArgument("atti_kd_pitch", default_value="0.20")

    # Mission outer-loop gains, overridable so cruise-speed experiments run
    # without rebuilding fc_sim. max_vxy clamps total xy velocity, so raising
    # cruise past it is a no-op unless max_vxy rises too. Defaults match
    # fc_mission_gains (cruise 0.5, max_vxy 0.8).
    mission_cruise_arg   = DeclareLaunchArgument("mission_cruise",   default_value="0.5")
    mission_max_vxy_arg  = DeclareLaunchArgument("mission_max_vxy",  default_value="0.8")

    # The simulated FC: fc_core control loop ticked from /clock, publishing
    # actuator_msgs/Actuators back through the bridge into Gazebo.
    fc_sim_node = Node(
        package="fc_sim",
        executable="fc_sim_node",
        name="fc_sim_node",
        output="screen",
        parameters=[{
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
            "rate_kp_p":       LaunchConfiguration("rate_kp_p"),
            "rate_kp_q":       LaunchConfiguration("rate_kp_q"),
            "rate_kp_r":       LaunchConfiguration("rate_kp_r"),
            "atti_kp_roll":    LaunchConfiguration("atti_kp_roll"),
            "atti_kp_pitch":   LaunchConfiguration("atti_kp_pitch"),
            "atti_kd_roll":    LaunchConfiguration("atti_kd_roll"),
            "atti_kd_pitch":   LaunchConfiguration("atti_kd_pitch"),
            "mission_cruise":  LaunchConfiguration("mission_cruise"),
            "mission_max_vxy": LaunchConfiguration("mission_max_vxy"),
        }],
    )

    return LaunchDescription(
        [
            world_arg,
            gui_arg,
            headless_arg,
            use_sim_time_arg,
            marker_seed_arg,
            rate_kp_p_arg, rate_kp_q_arg, rate_kp_r_arg,
            atti_kp_roll_arg, atti_kp_pitch_arg,
            atti_kd_roll_arg, atti_kd_pitch_arg,
            mission_cruise_arg, mission_max_vxy_arg,
            set_resource_path,
            set_resource_path_parent,
            marker_randomize,
            gz_sim_gui,
            gz_sim_headless,
            gz_sim_server_only,
            bridge_node,
            fc_sim_node,
        ]
    )
