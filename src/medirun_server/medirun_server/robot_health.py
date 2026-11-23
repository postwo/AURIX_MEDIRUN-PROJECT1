"""로봇 상태 집계(ROS 의존 없음).

로봇이 내보내는 토픽(/battery_state, /amcl_pose, 적재 센서, 안전 상태, Nav2 액션 서버 유무)을
모아 "지금 출동 가능한가"를 한곳에서 판단한다. 판단 결과는 /medirun/system/readiness,
/medirun/system/battery 로 다시 방송돼 emergency_dispatcher 의 출동 조건 검사에 쓰인다
(이전에는 이 두 토픽을 아무도 발행하지 않아 배터리·준비 검사가 항상 통과했다).

배터리 기준(설계서): 30% 미만 신규 미션 거부, 20% 미만 즉시 복귀, 10% 미만 안전정지.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class HealthThresholds:
    heartbeat_timeout_sec: float = 5.0
    battery_reject_percent: float = 30.0
    battery_return_percent: float = 20.0
    battery_critical_percent: float = 10.0


class RobotHealth:
    def __init__(self, robot_id: str, thresholds: Optional[HealthThresholds] = None) -> None:
        self.robot_id = robot_id
        self.thresholds = thresholds or HealthThresholds()
        self.battery_percent: Optional[float] = None
        self.payload_loaded: Optional[bool] = None
        self.safety_state: str = "UNKNOWN"
        self.pose: Optional[Dict[str, float]] = None
        self.nav2_available: bool = False
        self._last_seen: Optional[float] = None
        self._last_pose_at: Optional[float] = None

    # ---- 입력 ----------------------------------------------------------------
    def touch(self, now: float) -> None:
        self._last_seen = now

    def update_battery(self, percentage: float, now: float) -> None:
        # 드라이버에 따라 0.0~1.0(REP 표준) 또는 0~100 을 그대로 내보낸다(turtlebot3_node 는 후자).
        value = percentage * 100.0 if 0.0 <= percentage <= 1.0 else percentage
        self.battery_percent = max(0.0, min(100.0, value))
        self.touch(now)

    def update_pose(self, x: float, y: float, yaw: float, now: float) -> None:
        self.pose = {"x": x, "y": y, "yaw": yaw}
        self._last_pose_at = now
        self.touch(now)

    def update_payload(self, loaded: bool, now: float) -> None:
        self.payload_loaded = loaded
        self.touch(now)

    def update_safety(self, state: str, now: float) -> None:
        self.safety_state = state or "UNKNOWN"
        self.touch(now)

    # ---- 판단 ----------------------------------------------------------------
    def online(self, now: float) -> bool:
        return self._last_seen is not None and now - self._last_seen <= self.thresholds.heartbeat_timeout_sec

    def blockers(self, now: float) -> List[str]:
        """신규 출동을 막는 사유 목록. 비어 있으면 출동 가능."""
        reasons: List[str] = []
        if not self.online(now):
            reasons.append("ROBOT_OFFLINE")
        if not self.nav2_available:
            reasons.append("NAV2_UNAVAILABLE")
        if self._last_pose_at is None:
            reasons.append("LOCALIZATION_UNKNOWN")
        if self.battery_percent is not None and self.battery_percent < self.thresholds.battery_reject_percent:
            reasons.append("LOW_BATTERY")
        if self.payload_loaded is False:
            reasons.append("PAYLOAD_MISSING")
        if self.safety_state.upper() in {"ESTOP", "EMERGENCY_STOP"}:
            reasons.append("EMERGENCY_STOP")
        return reasons

    def battery_level(self) -> str:
        if self.battery_percent is None:
            return "UNKNOWN"
        if self.battery_percent < self.thresholds.battery_critical_percent:
            return "CRITICAL"
        if self.battery_percent < self.thresholds.battery_return_percent:
            return "RETURN"
        if self.battery_percent < self.thresholds.battery_reject_percent:
            return "LOW"
        return "OK"

    def readiness_text(self, now: float) -> str:
        """emergency_dispatcher 는 문자열에 READY 가 들어 있는지만 본다 → NOT_READY 는 쓰지 않는다."""
        reasons = self.blockers(now)
        return "READY" if not reasons else "BLOCKED:" + ",".join(reasons)

    def as_dict(self, now: float) -> Dict[str, object]:
        return {
            "robot_id": self.robot_id,
            "online": self.online(now),
            "battery_percent": self.battery_percent,
            "battery_level": self.battery_level(),
            "payload_loaded": self.payload_loaded,
            "safety_state": self.safety_state,
            "nav2_available": self.nav2_available,
            "pose": self.pose,
            "blockers": self.blockers(now),
        }
