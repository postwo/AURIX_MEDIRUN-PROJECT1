#!/bin/bash
# 중앙 서버(헤드리스, ROS 2) 실행 스크립트 — 대시보드 없이 PC에서 출동을 처리한다.
#
#   ./scripts/run_server.sh                          # active 모드 + 콜존 파이프라인
#   ./scripts/run_server.sh with_callzone:=false     # 콜존(mqtt_bridge/emergency_dispatcher) 별도 실행 시
#   MEDIRUN_SERVER_MODE=monitor ./scripts/run_server.sh   # 대시보드와 함께(관제 보조 모드)
#
# 전제: 로봇(Pi)에서 pi_bringup.launch.py가 떠 있고, PC와 같은 ROS_DOMAIN_ID·RMW(CycloneDDS)를 쓴다.
# 웨이포인트(maps/waypoints)·DB 설정(config/db.yaml)·로그(logs/medirun_server)는
# 워크스페이스 루트 기준 상대경로라서 루트로 이동한 뒤 실행한다.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "${SCRIPT_DIR}")"
cd "${WORKSPACE_ROOT}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"

: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"
export RMW_IMPLEMENTATION

if [ "${MEDIRUN_SERVER_MODE:-active}" = "monitor" ]; then
  exec ros2 launch medirun_server server_monitor.launch.py "$@"
fi
exec ros2 launch medirun_server server_bringup.launch.py "$@"
