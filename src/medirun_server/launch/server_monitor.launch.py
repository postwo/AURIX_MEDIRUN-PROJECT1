# 대시보드(pc_bringup.launch.py)와 같이 띄우는 관제 보조 모드.
# 출동은 대시보드가 계속 맡고, 서버는 로봇 상태 집계(/medirun/server/status)·운용 기록·
# get_status 서비스만 제공한다. 콜존 파이프라인은 pc_bringup 이 이미 띄우므로 여기선 안 띄운다.
#
# publish_system_health 는 기본 false — 기존 대시보드 운용에서는 readiness/battery 를 아무도
# 발행하지 않던 동작을 그대로 유지한다. 출동 조건 검사를 켜려면 true 로 넘긴다.
#
#   ros2 launch medirun_server server_monitor.launch.py
#   ros2 launch medirun_server server_monitor.launch.py publish_system_health:=true
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    server_config = str(Path(get_package_share_directory("medirun_server")) / "config" / "server.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=server_config),
            DeclareLaunchArgument("publish_system_health", default_value="false"),
            Node(
                package="medirun_server",
                executable="central_server",
                name="medirun_central_server",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "dispatch_mode": "monitor",
                        "publish_system_health": ParameterValue(
                            LaunchConfiguration("publish_system_health"), value_type=bool
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
