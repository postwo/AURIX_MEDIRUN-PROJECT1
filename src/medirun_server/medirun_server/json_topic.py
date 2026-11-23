"""std_msgs/String 에 JSON 을 싣는 프로젝트 관례(medirun_mission.json_topic 과 동일 형식)."""
from __future__ import annotations

import json
from typing import Any, Dict, Optional


def dump_json(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False, sort_keys=True)


def load_json_object(raw: str) -> Optional[Dict[str, Any]]:
    """JSON 객체가 아니면 None. 잘못된 메시지 하나가 콜백 예외로 노드를 멈추게 하지 않는다."""
    if not raw:
        return None
    try:
        payload = json.loads(raw)
    except ValueError:
        return None
    return payload if isinstance(payload, dict) else None
