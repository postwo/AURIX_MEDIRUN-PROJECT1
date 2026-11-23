"""MediRun 중앙 서버 노드(PC, 헤드리스).

로봇(Pi)에서 각자 돌던 판단 중 "한곳에서 봐야 하는 것"을 이 노드가 맡는다.
로봇과는 ROS 2(DDS) 토픽·액션·서비스로만 통신한다 — 별도 HTTP/WebSocket 경로는 없다.

┌──────────── 로봇(Pi) ────────────┐            ┌──────────── 중앙 서버(PC) ─────────────┐
│ arduino_bridge / Nav2 / AMCL     │ ─ topic ─▶ │ RobotHealth  (배터리·위치·적재·안전·Nav2) │
│ arrival_return_manager           │ ◀─ topic ─ │ MissionQueue (호출 대기열·상태 기계)     │
│ docking_manager                  │ ◀─ action ─│ Nav2 navigate_to_pose (출동)            │
└──────────────────────────────────┘            │ MissionRecorder (JSONL + PostgreSQL)    │
        ▲ /medirun/sensor/sos_button             └─────────────────────────────────────────┘
        │ (mqtt_bridge ← ESP32 콜버튼)                       │ /medirun/server/status
                                                             ▼ (대시보드·RViz·ros2 topic echo)

dispatch_mode
- ``active``  : 서버가 출동의 주인. dispatch_request 를 받아 대기열 → Nav2 goal → 도착 시
                arrival_notice 를 Pi 로 보낸다. **대시보드(medirun_gui)의 MissionController 와 동시에
                쓰지 않는다**(같은 로봇에 goal 이 두 번 가서 서로 preempt 된다 — callzone_bringup 주석 참고).
                server_bringup.launch.py 가 이 모드로 띄운다(GUI 없이 운용).
- ``monitor`` : 대시보드가 출동을 계속 맡고, 서버는 상태 집계·준비 판단·기록·서비스만 한다.
                pc_bringup 과 같이 띄워도 충돌하지 않는다.

토픽/서비스 (★ = 이 노드가 새로 만든 인터페이스)
  구독  /medirun/mission/dispatch_request  (String JSON)  emergency_dispatcher → 서버
        /medirun/mission/dispatch_rejection(String JSON)  거부 기록
        /medirun/mission/pi_return_status  (String JSON)  Pi 반출·복귀 단계
        /medirun/vision/dock_result        (String JSON)  도킹 결과 → 미션 종료
        /medirun/ui/mission_command        (String JSON)  {"command": "cancel"|"return_home"}
        /battery_state, /amcl_pose, /medirun/sensor/payload_loaded, /arduino/payload_loaded,
        /medirun/safety/state
  발행  ★/medirun/server/status        (String JSON, 1Hz) 로봇 상태 + 대기열 스냅샷
        ★/medirun/server/mission_event (String JSON)      미션 전이 이벤트
        /medirun/mission/arrival_notice  (String JSON)      active 모드 — Pi 에 도착·복귀 위임
        /medirun/system/battery, /medirun/system/readiness  emergency_dispatcher 출동 조건
        /medirun/vision/dock_request (Bool)                 운영자 홈 복귀 후 도킹 요청
  액션  navigate_to_pose (nav2_msgs/NavigateToPose)         active 모드 출동/홈 복귀
  서비스 ★/medirun/server/get_status, ★/medirun/server/cancel_mission, ★/medirun/server/return_home
        (std_srvs/Trigger)
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .json_topic import dump_json, load_json_object
from .mission_queue import ALLOWED, Mission, MissionQueue, Phase, TransitionError
from .mission_recorder import MissionRecorder, load_db_config
from .robot_health import HealthThresholds, RobotHealth
from .waypoint_store import Pose, WaypointLookupError, WaypointStore

# 시험에서 바꿔 끼울 수 있도록 모듈 수준에 둔다.
monotonic = time.monotonic

# Pi(arrival_return_manager)가 보내는 단계 → 서버 미션 단계
PI_PHASE = {
    "payload_removed": Phase.PAYLOAD_REMOVED,
    "returning": Phase.RETURNING,
    "return_failed": Phase.FAILED,
}


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CentralServer(Node):
    def __init__(self) -> None:
        super().__init__("medirun_central_server")
        self.declare_parameter("robot_id", "medirun_bot")
        self.declare_parameter("dispatch_mode", "active")
        self.declare_parameter("waypoints_path", "maps/waypoints")
        self.declare_parameter("home_waypoint_id", "HOME")
        self.declare_parameter("max_nav_attempts", 3)  # 최초 1회 + 재계획 2회(설계서)
        self.declare_parameter("heartbeat_timeout_sec", 5.0)
        self.declare_parameter("battery_reject_percent", 30.0)
        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("publish_system_health", True)
        self.declare_parameter("log_dir", "logs/medirun_server")
        self.declare_parameter("db_config_path", "config/db.yaml")

        mode = str(self.get_parameter("dispatch_mode").value).lower()
        if mode not in {"active", "monitor"}:
            self.get_logger().warning("알 수 없는 dispatch_mode=%s → monitor 로 동작" % mode)
            mode = "monitor"
        self._active_mode = mode == "active"
        self._home_waypoint_id = str(self.get_parameter("home_waypoint_id").value)
        self._max_nav_attempts = int(self.get_parameter("max_nav_attempts").value)
        self._publish_health = bool(self.get_parameter("publish_system_health").value)

        self._queue = MissionQueue()
        self._health = RobotHealth(
            str(self.get_parameter("robot_id").value),
            HealthThresholds(
                heartbeat_timeout_sec=float(self.get_parameter("heartbeat_timeout_sec").value),
                battery_reject_percent=float(self.get_parameter("battery_reject_percent").value),
            ),
        )
        self._waypoints = self._load_waypoints()
        db_config = load_db_config(Path(str(self.get_parameter("db_config_path").value)))
        self._recorder = MissionRecorder(
            Path(str(self.get_parameter("log_dir").value)),
            self.get_logger(),
            db_config=db_config,
            own_mission_rows=self._active_mode,
        )
        self._goal_handle = None
        # goal 을 보낼 때마다 1씩 올린다. 취소된 옛 goal 의 늦은 콜백이 새 goal 의 handle 을
        # 지우지 않도록, 콜백은 자기 순번이 최신일 때만 _goal_handle 을 건드린다.
        self._goal_seq = 0
        self._last_block_reason = ""

        # ---- 발행 ----
        self._status_pub = self.create_publisher(String, "/medirun/server/status", 10)
        self._event_pub = self.create_publisher(String, "/medirun/server/mission_event", 10)
        self._arrival_pub = self.create_publisher(String, "/medirun/mission/arrival_notice", 10)
        self._battery_pub = self.create_publisher(String, "/medirun/system/battery", 10)
        self._readiness_pub = self.create_publisher(String, "/medirun/system/readiness", 10)
        self._dock_pub = self.create_publisher(Bool, "/medirun/vision/dock_request", 10)

        # ---- 구독 ----
        self.create_subscription(String, "/medirun/mission/dispatch_request", self._on_dispatch_request, 10)
        self.create_subscription(String, "/medirun/mission/dispatch_rejection", self._on_dispatch_rejection, 10)
        self.create_subscription(String, "/medirun/mission/pi_return_status", self._on_pi_return_status, 10)
        self.create_subscription(String, "/medirun/vision/dock_result", self._on_dock_result, 10)
        self.create_subscription(String, "/medirun/ui/mission_command", self._on_operator_command, 10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10)
        self.create_subscription(Bool, "/medirun/sensor/payload_loaded", self._on_payload, 10)
        self.create_subscription(Bool, "/arduino/payload_loaded", self._on_payload, 10)
        self.create_subscription(String, "/medirun/safety/state", self._on_safety, 10)

        # ---- 액션 / 서비스 ----
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_service(Trigger, "/medirun/server/get_status", self._srv_get_status)
        self.create_service(Trigger, "/medirun/server/cancel_mission", self._srv_cancel)
        self.create_service(Trigger, "/medirun/server/return_home", self._srv_return_home)

        self.create_timer(float(self.get_parameter("status_period_sec").value), self._tick)
        self._recorder.record("server_started", {"dispatch_mode": mode, "waypoints": self._waypoints.waypoint_count})
        self.get_logger().info(
            "중앙 서버 시작: mode=%s, 웨이포인트 %d개, DB=%s"
            % (mode, self._waypoints.waypoint_count, "on" if db_config else "off")
        )

    # =================================================================== 입력: 로봇 상태
    def _load_waypoints(self) -> WaypointStore:
        path = Path(str(self.get_parameter("waypoints_path").value))
        try:
            return WaypointStore.from_directory(path)
        except (OSError, ValueError) as exc:
            self.get_logger().error("웨이포인트 로딩 실패(%s): %s" % (path, exc))
            return WaypointStore({}, {})

    def _on_battery(self, msg: BatteryState) -> None:
        self._health.update_battery(float(msg.percentage), monotonic())

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self._health.update_pose(pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation), monotonic())

    def _on_payload(self, msg: Bool) -> None:
        self._health.update_payload(bool(msg.data), monotonic())

    def _on_safety(self, msg: String) -> None:
        payload = load_json_object(msg.data)
        state = str(payload.get("state", "")) if payload else msg.data
        self._health.update_safety(state, monotonic())

    # =================================================================== 입력: 미션
    def _on_dispatch_request(self, msg: String) -> None:
        payload = load_json_object(msg.data)
        if payload is None:
            self.get_logger().warning("dispatch_request 무시: JSON 객체가 아님 %r" % msg.data)
            return
        mission_id = str(payload.get("mission_id") or "")
        button_id = str(payload.get("button_id") or "")
        if not self._active_mode:
            # monitor 모드: 출동은 대시보드가 한다. 기록만 남긴다.
            self._recorder.record("dispatch_request_observed", {"mission_id": mission_id, "button_id": button_id})
            return
        if not mission_id:
            self.get_logger().warning("dispatch_request 무시: mission_id 없음")
            return
        mission, reason = self._queue.enqueue(
            mission_id, button_id, monotonic(), str(payload.get("request_source", "sos_button"))
        )
        if mission is None:
            self._recorder.record("dispatch_ignored", {"mission_id": mission_id, "button_id": button_id,
                                                       "reason": reason}, "warn")
            self.get_logger().warning("호출 무시(%s): button_id=%s" % (reason, button_id))
            return
        self._emit_transition(mission, reason)
        self._try_dispatch()

    def _on_dispatch_rejection(self, msg: String) -> None:
        payload = load_json_object(msg.data) or {"raw": msg.data}
        self._recorder.record("dispatch_rejected", payload, "warn")

    def _on_pi_return_status(self, msg: String) -> None:
        payload = load_json_object(msg.data)
        if payload is None:
            return
        phase = PI_PHASE.get(str(payload.get("phase", "")))
        mission_id = str(payload.get("mission_id", ""))
        if phase is None:
            return  # armed 등은 상태 변화 없음
        self._advance(mission_id, phase, "pi:" + str(payload.get("phase")))

    def _on_dock_result(self, msg: String) -> None:
        payload = load_json_object(msg.data)
        active = self._queue.active
        if payload is None or active is None or active.phase is not Phase.RETURNING:
            return
        status = str(payload.get("status", ""))
        if status == "SUCCEEDED":
            self._advance(active.mission_id, Phase.COMPLETED, "docked")
        elif status == "FAILED":
            self._advance(active.mission_id, Phase.FAILED, "dock_failed:" + str(payload.get("reason", "")))

    def _on_operator_command(self, msg: String) -> None:
        if not self._active_mode:
            return  # monitor 모드에서는 대시보드가 같은 명령을 처리한다
        payload = load_json_object(msg.data) or {}
        command = payload.get("command")
        if command == "cancel":
            self._cancel_active("operator_topic")
        elif command == "return_home":
            self._return_home_active("operator_topic")

    # =================================================================== 출동
    def _try_dispatch(self) -> None:
        if not self._active_mode or self._queue.active is not None or not self._queue.pending:
            return
        blockers = self._health.blockers(monotonic())
        if blockers:
            reason = ",".join(blockers)
            if reason != self._last_block_reason:
                self.get_logger().warning("대기 호출 %d건, 출동 보류: %s" % (len(self._queue.pending), reason))
                self._recorder.record("dispatch_blocked", {"reason": reason}, "warn")
            self._last_block_reason = reason
            return
        self._last_block_reason = ""
        mission = self._queue.pop_next(monotonic())
        if mission is None:
            return
        self._emit_transition(mission, "dispatched")
        self._send_service_goal(mission)

    def _send_service_goal(self, mission: Mission) -> None:
        try:
            target = self._waypoints.call_zone_for_button(mission.button_id)
        except WaypointLookupError as exc:
            self._advance(mission.mission_id, Phase.FAILED, "UNKNOWN_BUTTON: %s" % exc)
            return
        self._send_goal(mission, target.pose, "service_target")

    def _send_goal(self, mission: Mission, pose: Pose, kind: str) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self._advance(mission.mission_id, Phase.FAILED, "NAV2_UNAVAILABLE")
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(pose)
        self._goal_seq += 1
        seq = self._goal_seq
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, mid=mission.mission_id, k=kind, s=seq: self._on_goal_response(f, mid, k, s)
        )

    def _pose_stamped(self, pose: Pose) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = pose.frame_id
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.position.x = pose.x
        stamped.pose.position.y = pose.y
        stamped.pose.orientation.z = math.sin(pose.yaw / 2.0)
        stamped.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return stamped

    def _on_goal_response(self, future, mission_id: str, kind: str, seq: int) -> None:
        handle = future.result()
        if seq != self._goal_seq:
            # 응답이 오기 전에 취소/복귀 명령으로 새 goal 이 나갔다 → 옛 goal 은 바로 취소한다.
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self._goal_handle = None
            self._advance(mission_id, Phase.FAILED, "GOAL_REJECTED")
            return
        self._goal_handle = handle
        if kind == "service_target":
            self._advance(mission_id, Phase.NAVIGATING, "goal_accepted")
        handle.get_result_async().add_done_callback(
            lambda f, mid=mission_id, k=kind, s=seq: self._on_goal_result(f, mid, k, s)
        )

    def _on_goal_result(self, future, mission_id: str, kind: str, seq: int) -> None:
        if seq != self._goal_seq:
            return  # 이미 다른 goal 로 대체됨(취소된 출동 goal 의 결과 등)
        self._goal_handle = None
        mission = self._queue.get(mission_id)
        if mission is None or mission.is_terminal:
            return
        status = future.result().status
        if kind == "return_home":
            if status == GoalStatus.STATUS_SUCCEEDED:
                # Pi 의 arrival_return_manager 가 하던 마무리(도킹 요청)를 서버가 대신한다.
                self._dock_pub.publish(Bool(data=True))
            else:
                self._advance(mission_id, Phase.FAILED, "RETURN_GOAL_STATUS_%d" % status)
            return
        if mission.phase is not Phase.NAVIGATING:
            return  # 그 사이 취소/복귀 명령이 들어왔다
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._advance(mission_id, Phase.ARRIVED, "nav_succeeded")
            self._publish_arrival_notice(mission, armed=True)
        elif status == GoalStatus.STATUS_CANCELED:
            return  # 취소 경로에서 이미 처리
        elif mission.nav_attempts < self._max_nav_attempts:
            self.get_logger().warning("목표 도달 실패(status=%d), 재계획 %d/%d"
                                      % (status, mission.nav_attempts, self._max_nav_attempts - 1))
            self._advance(mission_id, Phase.DISPATCHED, "retry")
            self._send_service_goal(mission)
        else:
            self._advance(mission_id, Phase.FAILED, "NAV_FAILED_STATUS_%d" % status)

    def _publish_arrival_notice(self, mission: Mission, armed: bool) -> None:
        payload: Dict[str, Any] = {"armed": armed, "mission_id": mission.mission_id, "queue_id": mission.queue_id}
        if armed:
            try:
                home = self._waypoints.home(self._home_waypoint_id)
            except WaypointLookupError as exc:
                # home_pose 가 없으면 Pi 가 복귀 goal 을 못 만든다 → 즉시 실패로 드러낸다.
                self._advance(mission.mission_id, Phase.FAILED, "HOME_UNKNOWN: %s" % exc)
                return
            payload["home_pose"] = home.pose.as_dict()
            payload["trajectory"] = [p.as_dict() for p in self._waypoints.return_trajectory(mission.button_id)]
        self._arrival_pub.publish(String(data=dump_json(payload)))

    # =================================================================== 운영자 명령
    def _cancel_active(self, source: str) -> str:
        mission = self._queue.active
        if mission is None:
            return "진행 중 미션 없음"
        self._cancel_goal()
        self._publish_arrival_notice(mission, armed=False)  # Pi 쪽 타이머·복귀 대기 해제
        self._advance(mission.mission_id, Phase.CANCELLED, "cancelled_by_" + source)
        return "취소됨: %s" % mission.mission_id

    def _return_home_active(self, source: str) -> str:
        mission = self._queue.active
        if mission is None:
            return "진행 중 미션 없음"
        if mission.phase is Phase.RETURNING:
            return "이미 복귀 중"
        if Phase.RETURNING not in ALLOWED[mission.phase]:
            return "%s 상태에서는 홈 복귀를 시작할 수 없습니다(goal 수락 대기 중)" % mission.phase.value
        try:
            home = self._waypoints.home(self._home_waypoint_id)
        except WaypointLookupError as exc:
            return "홈 웨이포인트 조회 실패: %s" % exc
        self._cancel_goal()
        self._publish_arrival_notice(mission, armed=False)
        self._advance(mission.mission_id, Phase.RETURNING, "return_home_by_" + source)
        self._send_goal(mission, home.pose, "return_home")
        return "홈 복귀 시작: %s" % mission.mission_id

    def _cancel_goal(self) -> None:
        handle, self._goal_handle = self._goal_handle, None
        self._goal_seq += 1  # 아직 응답 전인 goal 도 무효화(응답 오면 _on_goal_response 가 취소)
        if handle is not None:
            handle.cancel_goal_async()

    # =================================================================== 서비스
    def _srv_get_status(self, _request, response):
        response.success = True
        response.message = dump_json(self._status_payload())
        return response

    def _srv_cancel(self, _request, response):
        if not self._active_mode:
            response.success, response.message = False, "monitor 모드에서는 대시보드에서 취소하세요"
            return response
        response.message = self._cancel_active("service")
        response.success = response.message.startswith("취소됨")
        return response

    def _srv_return_home(self, _request, response):
        if not self._active_mode:
            response.success, response.message = False, "monitor 모드에서는 대시보드에서 복귀시키세요"
            return response
        response.message = self._return_home_active("service")
        response.success = response.message.startswith("홈 복귀 시작")
        return response

    # =================================================================== 공용
    def _advance(self, mission_id: str, phase: Phase, note: str) -> Optional[Mission]:
        try:
            mission = self._queue.transition(mission_id, phase, monotonic(), note)
        except TransitionError as exc:
            self.get_logger().debug("전이 무시: %s" % exc)
            return None
        self._emit_transition(mission, note)
        if mission.is_terminal:
            self._try_dispatch()
        return mission

    def _emit_transition(self, mission: Mission, note: str) -> None:
        self._event_pub.publish(String(data=dump_json({**mission.as_dict(), "note": note})))
        self._recorder.record_transition(mission, note)
        self.get_logger().info("mission %s [%s] → %s (%s)"
                               % (mission.queue_id, mission.button_id, mission.phase.value, note))

    def _status_payload(self) -> Dict[str, Any]:
        now = monotonic()
        return {
            "dispatch_mode": "active" if self._active_mode else "monitor",
            "robot": self._health.as_dict(now),
            "missions": self._queue.snapshot(),
            "db_connected": self._recorder.db_connected,
        }

    def _tick(self) -> None:
        now = monotonic()
        self._health.nav2_available = bool(self._nav_client.server_is_ready())
        if self._publish_health:
            if self._health.battery_percent is not None:
                self._battery_pub.publish(String(data="%.1f" % self._health.battery_percent))
            self._readiness_pub.publish(String(data=self._health.readiness_text(now)))
        self._status_pub.publish(String(data=dump_json(self._status_payload())))
        self._try_dispatch()  # 로봇이 준비되면 보류된 호출을 꺼낸다

    def destroy_node(self) -> None:
        self._recorder.record("server_stopped", {})
        self._recorder.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CentralServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
