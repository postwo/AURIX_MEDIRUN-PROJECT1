"""중앙 서버의 호출 대기열과 미션 상태 기계(ROS 의존 없음 — 단위 시험 대상).

설계서 기준 판단 원칙을 그대로 따른다.

- 주행 중 중복 호출: 기존 미션 유지, 신규 호출은 대기열에 등록한다.
- 같은 버튼의 호출이 이미 대기·진행 중이면 다시 받지 않는다(중복 출동 방지).
- 진행 중 미션은 한 번에 하나(단일 로봇). 끝나면(COMPLETED/FAILED/CANCELLED) 다음 대기 호출을 꺼낸다.

상태 전이::

    QUEUED → DISPATCHED → NAVIGATING → ARRIVED → PAYLOAD_REMOVED → RETURNING → COMPLETED
                 │             │           │              │              │
                 └─────────────┴───────────┴──────────────┴──────────────┴─→ FAILED / CANCELLED

ARRIVED → RETURNING 은 3분 미반출 강제 복귀(Pi 판단) 또는 운영자 홈 복귀 명령일 때 생긴다.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque, Dict, List, Optional, Tuple


class Phase(str, Enum):
    QUEUED = "QUEUED"
    DISPATCHED = "DISPATCHED"
    NAVIGATING = "NAVIGATING"
    ARRIVED = "ARRIVED"
    PAYLOAD_REMOVED = "PAYLOAD_REMOVED"
    RETURNING = "RETURNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


TERMINAL = frozenset({Phase.COMPLETED, Phase.FAILED, Phase.CANCELLED})
_ABORTABLE = frozenset({Phase.FAILED, Phase.CANCELLED})

ALLOWED: Dict[Phase, frozenset] = {
    Phase.QUEUED: frozenset({Phase.DISPATCHED}) | _ABORTABLE,
    # 재시도(NAVIGATING → DISPATCHED)는 Nav2 goal 을 다시 보낼 때 생긴다.
    Phase.DISPATCHED: frozenset({Phase.NAVIGATING}) | _ABORTABLE,
    Phase.NAVIGATING: frozenset({Phase.DISPATCHED, Phase.ARRIVED, Phase.RETURNING}) | _ABORTABLE,
    Phase.ARRIVED: frozenset({Phase.PAYLOAD_REMOVED, Phase.RETURNING}) | _ABORTABLE,
    Phase.PAYLOAD_REMOVED: frozenset({Phase.RETURNING}) | _ABORTABLE,
    Phase.RETURNING: frozenset({Phase.COMPLETED}) | _ABORTABLE,
    Phase.COMPLETED: frozenset(),
    Phase.FAILED: frozenset(),
    Phase.CANCELLED: frozenset(),
}


@dataclass
class Mission:
    mission_id: str
    queue_id: str
    button_id: str
    requested_at: float
    request_source: str = "sos_button"
    phase: Phase = Phase.QUEUED
    nav_attempts: int = 0
    failure_reason: str = ""
    history: List[Tuple[str, float, str]] = field(default_factory=list)

    @property
    def is_terminal(self) -> bool:
        return self.phase in TERMINAL

    def as_dict(self) -> Dict[str, object]:
        return {
            "mission_id": self.mission_id,
            "queue_id": self.queue_id,
            "button_id": self.button_id,
            "phase": self.phase.value,
            "requested_at": self.requested_at,
            "request_source": self.request_source,
            "nav_attempts": self.nav_attempts,
            "failure_reason": self.failure_reason,
        }


class TransitionError(ValueError):
    pass


class MissionQueue:
    def __init__(self, history_size: int = 50) -> None:
        self._pending: Deque[Mission] = deque()
        self._active: Optional[Mission] = None
        self._by_id: Dict[str, Mission] = {}
        self._finished: Deque[Mission] = deque(maxlen=history_size)
        self._sequence = 0

    # ---- 등록 ----------------------------------------------------------------
    def enqueue(self, mission_id: str, button_id: str, now: float,
                request_source: str = "sos_button") -> Tuple[Optional[Mission], str]:
        """(미션, 사유). 거부되면 미션은 None 이고 사유에 거부 코드가 들어간다."""
        if not button_id:
            return None, "EMPTY_BUTTON_ID"
        if mission_id in self._by_id:
            return None, "DUPLICATE_MISSION_ID"
        if any(m.button_id == button_id for m in self._open_missions()):
            return None, "DUPLICATE_CALL"
        self._sequence += 1
        mission = Mission(
            mission_id=mission_id,
            queue_id=f"Q-{self._sequence:04d}",
            button_id=button_id,
            requested_at=now,
            request_source=request_source,
        )
        mission.history.append((Phase.QUEUED.value, now, "enqueued"))
        self._pending.append(mission)
        self._by_id[mission_id] = mission
        return mission, "QUEUED" if self._active is None else "QUEUED_BEHIND_ACTIVE"

    # ---- 조회 ----------------------------------------------------------------
    @property
    def active(self) -> Optional[Mission]:
        return self._active

    @property
    def pending(self) -> List[Mission]:
        return list(self._pending)

    def get(self, mission_id: str) -> Optional[Mission]:
        return self._by_id.get(mission_id)

    def _open_missions(self) -> List[Mission]:
        missions = list(self._pending)
        if self._active is not None:
            missions.append(self._active)
        return missions

    # ---- 진행 ----------------------------------------------------------------
    def pop_next(self, now: float) -> Optional[Mission]:
        """진행 중 미션이 없을 때만 대기열 맨 앞을 DISPATCHED 로 꺼낸다."""
        if self._active is not None or not self._pending:
            return None
        mission = self._pending.popleft()
        self._active = mission
        self._apply(mission, Phase.DISPATCHED, now, "dispatched")
        return mission

    def transition(self, mission_id: str, phase: Phase, now: float, note: str = "") -> Mission:
        mission = self._by_id.get(mission_id)
        if mission is None:
            raise TransitionError(f"알 수 없는 mission_id={mission_id}")
        if phase not in ALLOWED[mission.phase]:
            raise TransitionError(f"{mission.phase.value} → {phase.value} 전이는 허용되지 않습니다")
        if mission.phase is Phase.QUEUED and phase in _ABORTABLE:
            # 대기열에서 바로 취소/실패되는 경우
            self._pending.remove(mission)
        self._apply(mission, phase, now, note)
        return mission

    def _apply(self, mission: Mission, phase: Phase, now: float, note: str) -> None:
        mission.phase = phase
        if phase is Phase.DISPATCHED:
            mission.nav_attempts += 1
        if phase in _ABORTABLE and note:
            mission.failure_reason = note
        mission.history.append((phase.value, now, note))
        if phase in TERMINAL:
            if self._active is mission:
                self._active = None
            self._finished.appendleft(mission)

    # ---- 보고 ----------------------------------------------------------------
    def snapshot(self) -> Dict[str, object]:
        return {
            "active": self._active.as_dict() if self._active else None,
            "pending": [m.as_dict() for m in self._pending],
            "recent": [m.as_dict() for m in list(self._finished)[:10]],
        }
