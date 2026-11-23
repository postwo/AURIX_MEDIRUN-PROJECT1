"""웨이포인트 YAML(maps/waypoints/*.yaml) 조회.

GUI(WaypointRepository)·waypoint_manager 와 같은 스키마를 읽는다::

    frame_id: map
    waypoints:
      BTN_01: {x: 3.1, y: 1.09, yaw: -3.14, kind: call_zone, label: call_01}
      HOME:   {x: 2.87, y: -2.02, yaw: 1.45, kind: home, label: home}
    trajectories:
      btn1_home: [BTN_01, W-01, W-02]

여러 파일에 나뉘어 저장되므로(버튼/홈/경로 파일이 따로 있음) 디렉터리 전체를 합쳐서 본다.
같은 ID가 여러 파일에 있으면 파일명 정렬 순서상 나중 파일이 이긴다(GUI 저장 순서와 동일).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import yaml


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"

    def as_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "frame_id": self.frame_id}


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    pose: Pose
    kind: str = "waypoint"
    label: str = ""


class WaypointLookupError(LookupError):
    pass


class WaypointStore:
    def __init__(self, waypoints: Dict[str, Waypoint], trajectories: Dict[str, List[str]]) -> None:
        self._waypoints = waypoints
        self._trajectories = trajectories

    # ---- 로딩 ----------------------------------------------------------------
    @classmethod
    def from_paths(cls, paths: Iterable[Path]) -> "WaypointStore":
        waypoints: Dict[str, Waypoint] = {}
        trajectories: Dict[str, List[str]] = {}
        for path in paths:
            with Path(path).open("r", encoding="utf-8") as stream:
                payload = yaml.safe_load(stream) or {}
            frame_id = str(payload.get("frame_id", "map"))
            for waypoint_id, raw in (payload.get("waypoints") or {}).items():
                if not isinstance(raw, dict) or "x" not in raw or "y" not in raw:
                    continue
                waypoints[str(waypoint_id)] = Waypoint(
                    waypoint_id=str(waypoint_id),
                    pose=Pose(float(raw["x"]), float(raw["y"]), float(raw.get("yaw", 0.0)), frame_id),
                    kind=str(raw.get("kind", "waypoint")),
                    label=str(raw.get("label", "")),
                )
            for name, steps in (payload.get("trajectories") or {}).items():
                if isinstance(steps, list):
                    trajectories[str(name)] = [str(step) for step in steps]
        return cls(waypoints, trajectories)

    @classmethod
    def from_directory(cls, directory: Path) -> "WaypointStore":
        directory = Path(directory)
        if directory.is_file():
            return cls.from_paths([directory])
        return cls.from_paths(sorted(directory.glob("*.yaml")))

    # ---- 조회 ----------------------------------------------------------------
    @property
    def waypoint_count(self) -> int:
        return len(self._waypoints)

    def get(self, waypoint_id: str) -> Optional[Waypoint]:
        return self._waypoints.get(waypoint_id)

    def call_zone_for_button(self, button_id: str) -> Waypoint:
        """버튼 ID 와 1:1 매핑된 호출 구역. 버튼 ID 가 곧 웨이포인트 ID 다(waypoint_manager 와 동일)."""
        waypoint = self._waypoints.get(button_id)
        if waypoint is None:
            raise WaypointLookupError(f"버튼 {button_id!r} 에 매핑된 웨이포인트가 없습니다")
        return waypoint

    def home(self, home_waypoint_id: str = "HOME") -> Waypoint:
        """홈 웨이포인트. 지정 ID 가 없으면 kind=home 이 정확히 하나일 때만 그것을 쓴다."""
        waypoint = self._waypoints.get(home_waypoint_id)
        if waypoint is not None:
            return waypoint
        homes = [w for w in self._waypoints.values() if w.kind == "home"]
        if len(homes) == 1:
            return homes[0]
        if not homes:
            raise WaypointLookupError("home 웨이포인트가 없습니다")
        raise WaypointLookupError("home 웨이포인트가 여러 개라 하나로 정할 수 없습니다")

    def return_trajectory(self, button_id: str) -> List[Pose]:
        """버튼 구역에서 홈까지 복귀할 때 경유할 포즈 목록(없으면 빈 목록 = HOME 직행).

        trajectory 의 첫 원소가 버튼 ID 인 경로를 고른다. 첫 원소(이미 도착한 호출 구역)는
        경유점에서 뺀다. 조회되지 않는 경유점 ID 가 섞여 있으면 경로 전체를 버리고 직행한다 —
        중간 점 하나가 빠진 채 달리면 설계와 다른 길로 가게 된다.
        """
        for name in sorted(self._trajectories):
            steps = self._trajectories[name]
            if not steps or steps[0] != button_id:
                continue
            poses: List[Pose] = []
            for step in steps[1:]:
                waypoint = self._waypoints.get(step)
                if waypoint is None:
                    return []
                poses.append(waypoint.pose)
            return poses
        return []
