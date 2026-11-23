"""미션·이벤트 기록.

- 항상: 날짜별 JSONL(logs/medirun_server/server_events_YYYYMMDD.jsonl) — event_logger 관례.
- 선택: PostgreSQL(config/db.yaml). psycopg2 가 없거나 DB 연결이 실패해도 서버는 멈추지 않고
  JSONL 기록만 계속한다(기록 실패가 출동을 막으면 안 된다).

DB 쓰기 범위
- system_event: 모든 미션 전이·거부·운영자 명령(event_type=mission_<phase> 등).
- mission: dispatch_mode=active 일 때만(=중앙 서버가 미션의 주인일 때). monitor 모드에서는 GUI
  (MissionStorageService)가 mission 행을 쓰므로 중복으로 만들지 않는다.
"""
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from .mission_queue import Mission, Phase

# mission.status CHECK 제약과 맞춘다(returning 상태 컬럼값은 없어서 dispensing 으로 둔다).
MISSION_STATUS = {
    Phase.QUEUED: "queued",
    Phase.DISPATCHED: "dispatched",
    Phase.NAVIGATING: "navigating",
    Phase.ARRIVED: "arrived",
    Phase.PAYLOAD_REMOVED: "dispensing",
    Phase.RETURNING: "dispensing",
    Phase.COMPLETED: "completed",
    Phase.FAILED: "failed",
    Phase.CANCELLED: "cancelled",
}

SEVERITY = {Phase.FAILED: "error", Phase.CANCELLED: "warn"}


def load_db_config(path: Path) -> Optional[Dict[str, Any]]:
    path = Path(path)
    if not path.is_file():
        return None
    with path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    if not payload.get("database") or not payload.get("user"):
        return None
    return {
        "host": payload.get("host", "localhost"),
        "port": int(payload.get("port", 5432)),
        "dbname": payload["database"],
        "user": payload["user"],
        "password": payload.get("password") or "",
        "connect_timeout": 3,
    }


class MissionRecorder:
    def __init__(self, log_dir: Path, logger, db_config: Optional[Dict[str, Any]] = None,
                 own_mission_rows: bool = False) -> None:
        self._logger = logger
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._db_config = db_config
        self._own_mission_rows = own_mission_rows
        self._conn = None
        self._mission_row_ids: Dict[str, int] = {}

    # ---- 공개 API ------------------------------------------------------------
    def record(self, event_type: str, payload: Dict[str, Any], severity: str = "info") -> None:
        entry = {"at": datetime.now().astimezone().isoformat(), "event_type": event_type,
                 "severity": severity, **payload}
        self._append_jsonl(entry)
        self._db_execute(
            "INSERT INTO system_event (event_type, severity, message) VALUES (%s, %s, %s)",
            (event_type[:64], severity, json.dumps(payload, ensure_ascii=False)),
        )

    def record_transition(self, mission: Mission, note: str = "") -> None:
        self.record(
            "mission_" + mission.phase.value.lower(),
            {**mission.as_dict(), "note": note},
            SEVERITY.get(mission.phase, "info"),
        )
        if self._own_mission_rows:
            self._upsert_mission_row(mission)

    @property
    def db_connected(self) -> bool:
        return self._conn is not None

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001
                pass
            self._conn = None

    # ---- JSONL ---------------------------------------------------------------
    def _append_jsonl(self, entry: Dict[str, Any]) -> None:
        path = self._log_dir / f"server_events_{datetime.now().strftime('%Y%m%d')}.jsonl"
        try:
            with path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except OSError as exc:
            self._logger.warning("JSONL 기록 실패: %s" % exc)

    # ---- PostgreSQL ----------------------------------------------------------
    def _connection(self):
        if self._db_config is None:
            return None
        if self._conn is not None and not getattr(self._conn, "closed", 0):
            return self._conn
        try:
            import psycopg2  # 선택 의존성
        except ImportError:
            self._logger.warning("psycopg2 가 없어 DB 기록을 끕니다(JSONL 기록은 계속)")
            self._db_config = None
            return None
        try:
            self._conn = psycopg2.connect(**self._db_config)
            self._conn.autocommit = True
            self._logger.info("PostgreSQL 연결: %s/%s" % (self._db_config["host"], self._db_config["dbname"]))
        except Exception as exc:  # noqa: BLE001 - DB 장애가 서버를 멈추게 하지 않는다
            self._logger.warning("PostgreSQL 연결 실패(다음 기록 때 재시도): %s" % exc)
            self._conn = None
        return self._conn

    def _db_execute(self, sql: str, params: tuple, fetch_one: bool = False):
        conn = self._connection()
        if conn is None:
            return None
        try:
            with conn.cursor() as cursor:
                cursor.execute(sql, params)
                return cursor.fetchone() if fetch_one else None
        except Exception as exc:  # noqa: BLE001
            self._logger.warning("DB 기록 실패: %s" % exc)
            self.close()
            return None

    def _upsert_mission_row(self, mission: Mission) -> None:
        status = MISSION_STATUS[mission.phase]
        row_id = self._mission_row_ids.get(mission.mission_id)
        if row_id is None:
            row = self._db_execute(
                "INSERT INTO mission (call_button_id, is_emergency, status) "
                "SELECT id, TRUE, %s FROM call_button WHERE code = %s AND deregistered_at IS NULL "
                "RETURNING id",
                (status, mission.button_id),
                fetch_one=True,
            )
            if row:
                self._mission_row_ids[mission.mission_id] = int(row[0])
            return
        stamp_column = {
            Phase.DISPATCHED: "dispatched_at",
            Phase.ARRIVED: "arrived_at",
            Phase.PAYLOAD_REMOVED: "dispensed_at",
        }.get(mission.phase)
        if mission.is_terminal:
            stamp_column = "ended_at"
        if stamp_column:
            # 재시도로 DISPATCHED 가 여러 번 와도 최초 시각을 유지한다.
            self._db_execute(
                f"UPDATE mission SET status = %s, {stamp_column} = COALESCE({stamp_column}, NOW()) WHERE id = %s",
                (status, row_id),
            )
        else:
            self._db_execute("UPDATE mission SET status = %s WHERE id = %s", (status, row_id))
