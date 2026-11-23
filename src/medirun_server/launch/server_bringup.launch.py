# 중앙 서버(헤드리스) 운용 launch — 대시보드 없이 PC 한 대에서 출동을 처리한다.
#
#   ESP32 콜버튼 ─MQTT→ mqtt_bridge ─/medirun/sensor/sos_button→ emergency_dispatcher
#     ─/medirun/mission/dispatch_request→ medirun_central_server(active)
#     ─navigate_to_pose(액션)→ Pi Nav2 ─도착→ /medirun/mission/arrival_notice → Pi arrival_return_manager
#
# medirun_gui(pc_bringup.launch.py)와 동시에 띄우지 않는다: 대시보드의 MissionController 도
# dispatch_request 를 받아 goal 을 보내므로 같은 로봇에 goal 이 두 번 가서 서로 preempt 된다.
# 대시보드와 같이 쓰려면 server_monitor.launch.py 를 쓴다.
#
# 사용 예(워크스페이스 루트에서):
#   ros2 launch medirun_server server_bringup.launch.py
#   ros2 launch medirun_server server_bringup.launch.py with_callzone:=false   # 콜존 파이프라인을 따로 띄울 때
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    server_config = str(Path(get_package_share_directory("medirun_server")) / "config" / "server.yaml")
    bringup_launch_dir = os.path.join(get_package_share_directory("medirun_bringup"), "launch")
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=server_config),
            DeclareLaunchArgument("with_callzone", default_value="true",
                                  description="mqtt_bridge + emergency_dispatcher 를 같이 띄울지"),
            Node(
                package="medirun_server",
                executable="central_server",
                name="medirun_central_server",
                parameters=[LaunchConfiguration("params_file"), {"dispatch_mode": "active"}],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([bringup_launch_dir, "/callzone_bringup.launch.py"]),
                condition=IfCondition(LaunchConfiguration("with_callzone")),
            ),
        ]
    )
