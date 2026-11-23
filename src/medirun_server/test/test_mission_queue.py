import pytest

from medirun_server.mission_queue import MissionQueue, Phase, TransitionError


def test_enqueue_and_pop_in_order():
    queue = MissionQueue()
    first, reason = queue.enqueue("M-1", "BTN_01", 1.0)
    assert reason == "QUEUED" and first.queue_id == "Q-0001"
    second, reason = queue.enqueue("M-2", "BTN_02", 2.0)
    assert reason == "QUEUED"
    assert queue.pop_next(3.0) is first
    assert first.phase is Phase.DISPATCHED and first.nav_attempts == 1
    # 진행 중 미션이 있으면 다음 호출은 꺼내지 않는다(단일 로봇)
    assert queue.pop_next(4.0) is None
    assert queue.pending == [second]


def test_call_while_active_is_queued_behind():
    queue = MissionQueue()
    queue.enqueue("M-1", "BTN_01", 1.0)
    queue.pop_next(1.0)
    _, reason = queue.enqueue("M-2", "BTN_02", 2.0)
    assert reason == "QUEUED_BEHIND_ACTIVE"


def test_duplicate_button_call_rejected_while_open():
    queue = MissionQueue()
    queue.enqueue("M-1", "BTN_01", 1.0)
    mission, reason = queue.enqueue("M-2", "BTN_01", 2.0)
    assert mission is None and reason == "DUPLICATE_CALL"


def test_duplicate_mission_id_and_empty_button_rejected():
    queue = MissionQueue()
    queue.enqueue("M-1", "BTN_01", 1.0)
    assert queue.enqueue("M-1", "BTN_02", 2.0) == (None, "DUPLICATE_MISSION_ID")
    assert queue.enqueue("M-3", "", 2.0) == (None, "EMPTY_BUTTON_ID")


def test_full_happy_path_frees_slot_for_next():
    queue = MissionQueue()
    queue.enqueue("M-1", "BTN_01", 1.0)
    queue.enqueue("M-2", "BTN_02", 1.5)
    queue.pop_next(2.0)
    for phase in (Phase.NAVIGATING, Phase.ARRIVED, Phase.PAYLOAD_REMOVED, Phase.RETURNING, Phase.COMPLETED):
        queue.transition("M-1", phase, 3.0)
    assert queue.active is None
    assert queue.pop_next(4.0).mission_id == "M-2"
    assert queue.snapshot()["recent"][0]["phase"] == "COMPLETED"


def test_same_button_can_call_again_after_completion():
    queue = MissionQueue()
    queue.enqueue("M-1", "BTN_01", 1.0)
    queue.pop_next(1.0)
    queue.transition("M-1", Phase.FAILED, 2.0, "NAV_FAILED")
    mission, reason = queue.enqueue("M-2", "BTN_01", 3.0)
    assert mission is not None and reason == "QUEUED"


def test_invalid_transition_raises():
    queue = MissionQueue()
    queue.enqueue("M-1", "BTN_01", 1.0)
    queue.pop_next(1.0)
    with pytest.raises(TransitionError):
        queue.transition("M-1", Phase.COMPLETED, 2.0)
    with pytest.raises(TransitionError):
        queue.transition("nope", Phase.FAILED, 2.0)


def test_cancel_from_queue_removes_pending():
    queue = MissionQueue()
    queue.enqueue("M-1", "BTN_01", 1.0)
    queue.transition("M-1", Phase.CANCELLED, 2.0, "operator")
    assert queue.pending == [] and queue.get("M-1").failure_reason == "operator"


def test_retry_counts_attempts():
    queue = MissionQueue()
    queue.enqueue("M-1", "BTN_01", 1.0)
    queue.pop_next(1.0)
    queue.transition("M-1", Phase.NAVIGATING, 2.0)
    queue.transition("M-1", Phase.DISPATCHED, 3.0, "retry")
    assert queue.get("M-1").nav_attempts == 2
