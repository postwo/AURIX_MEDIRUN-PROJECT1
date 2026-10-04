import json
import math

import fake_ros
import pytest
from fake_ros import AudioCommand, Bool, FakeActionClient, FakeGoalHandle, FakeResult, GoalStatus, String

from medirun_mission import arrival_return_manager as arm_module
from medirun_mission.arrival_return_manager import (
    FORCED_RETURN_TIMEOUT_SEC,
    PAYLOAD_REMOVED_CONFIRM_SEC,
    REMINDER_INTERVAL_SEC,
    RETURN_DELAY_AFTER_PAYLOAD_REMOVED_SEC,
    ArrivalReturnManager,
)

NOTICE = "/medirun/mission/arrival_notice"
PAYLOAD = "/arduino/payload_loaded"
OBSTACLE = "/medirun/safety/obstacle_stop"
AUDIO = "/audio/command"
STATUS = "/medirun/mission/pi_return_status"
DOCK = "/medirun/vision/dock_request"

HOME = {"x": 1.0, "y": 2.0, "yaw": math.pi, "frame_id": "map"}


@pytest.fixture
def node(clock, tmp_path, monkeypatch):
    monkeypatch.setattr(arm_module, "PENDING_EVENTS_PATH", tmp_path / "pi" / "events.jsonl")
    return ArrivalReturnManager()


def tick(node):
    node.timers[0][1]()


def notice(node, armed=True, home=HOME, trajectory=None, mission="M-1", queue="Q-1"):
    payload = {"armed": armed, "mission_id": mission, "queue_id": queue}
    if home is not None:
        payload["home_pose"] = home
    if trajectory is not None:
        payload["trajectory"] = trajectory
    node.deliver(NOTICE, String(data=json.dumps(payload)))


def statuses(node):
    return [json.loads(msg.data) for msg in node.messages(STATUS)]


def phases(node):
    return [item["phase"] for item in statuses(node)]


def spoken(node):
    return [msg.text for msg in node.messages(AUDIO)]


def remove_payload(node, clock):
    """홀센서가 '적재됨 → 비어 있음'으로 바뀌고 확인 시간이 지나도록 만든다."""
    node.deliver(PAYLOAD, Bool(data=True))
    node.deliver(PAYLOAD, Bool(data=False))
    clock.advance(PAYLOAD_REMOVED_CONFIRM_SEC + 0.1)
    node.deliver(PAYLOAD, Bool(data=False))


def nav_client():
    return next(c for c in FakeActionClient.instances if c.name == "navigate_to_pose")


def through_client():
    return next(c for c in FakeActionClient.instances if c.name == "navigate_through_poses")


# ---- 도착 통지(arm / disarm) ----------------------------------------------------


def test_arrival_notice_arms_and_reports(node):
    notice(node)

    assert phases(node) == ["armed"]
    assert statuses(node)[0]["mission_id"] == "M-1"
    assert statuses(node)[0]["queue_id"] == "Q-1"
    assert node._armed


@pytest.mark.parametrize("home", [None, {}, {"x": 1.0}, {"y": 2.0}])
def test_notice_without_usable_home_is_ignored(node, home):
    notice(node, home=home)

    assert not node._armed
    assert phases(node) == []
    assert node.log_text("error")


def test_disarm_notice_stops_all_timers(node, clock):
    notice(node)
    notice(node, armed=False)

    clock.advance(FORCED_RETURN_TIMEOUT_SEC + 100)
    tick(node)

    assert not node._armed
    assert spoken(node) == []
    assert phases(node) == ["armed"]


def test_disarm_when_not_armed_is_harmless(node):
    notice(node, armed=False)

    assert not node._armed
    assert phases(node) == []


def test_rearming_resets_previous_removal_state(node, clock):
    notice(node)
    remove_payload(node, clock)
    assert node._payload_removed

    notice(node, mission="M-2")

    assert not node._payload_removed
    assert node._removed_deadline is None
    assert statuses(node)[-1]["mission_id"] == "M-2"


# ---- 반출 감지 -----------------------------------------------------------------


def test_payload_removal_needs_confirmation_time(node, clock):
    notice(node)
    node.deliver(PAYLOAD, Bool(data=True))
    node.deliver(PAYLOAD, Bool(data=False))
    clock.advance(PAYLOAD_REMOVED_CONFIRM_SEC - 0.1)
    node.deliver(PAYLOAD, Bool(data=False))

    assert "payload_removed" not in phases(node)

    clock.advance(0.2)
    node.deliver(PAYLOAD, Bool(data=False))

    assert phases(node)[-1] == "payload_removed"


def test_sensor_flicker_does_not_count_as_removal(node, clock):
    notice(node)
    node.deliver(PAYLOAD, Bool(data=True))
    node.deliver(PAYLOAD, Bool(data=False))
    clock.advance(0.3)
    node.deliver(PAYLOAD, Bool(data=True))  # 다시 감지됨 → 타이머 취소
    clock.advance(0.3)
    node.deliver(PAYLOAD, Bool(data=False))
    clock.advance(0.3)
    node.deliver(PAYLOAD, Bool(data=False))

    assert "payload_removed" not in phases(node)


def test_payload_events_before_arming_are_only_remembered(node, clock):
    node.deliver(PAYLOAD, Bool(data=True))
    node.deliver(PAYLOAD, Bool(data=False))
    clock.advance(5)
    node.deliver(PAYLOAD, Bool(data=False))

    assert phases(node) == []
    assert not node._payload_removed


def test_removal_is_reported_only_once(node, clock):
    notice(node)
    remove_payload(node, clock)
    node.deliver(PAYLOAD, Bool(data=False))
    node.deliver(PAYLOAD, Bool(data=False))

    assert phases(node).count("payload_removed") == 1


def test_removal_switches_from_forced_timeout_to_short_delay(node, clock):
    notice(node)
    remove_payload(node, clock)

    assert node._forced_deadline is None
    assert node._removed_deadline == pytest.approx(clock.now + RETURN_DELAY_AFTER_PAYLOAD_REMOVED_SEC)
    assert node._next_reminder_at is None


# ---- 타이머: 안내·복귀 ----------------------------------------------------------


def test_reminder_repeats_every_interval_without_returning(node, clock):
    notice(node)

    clock.advance(REMINDER_INTERVAL_SEC - 1)
    tick(node)
    assert spoken(node) == []

    clock.advance(1)
    tick(node)
    assert spoken(node) == ["AED를 반출해 주세요"]

    clock.advance(REMINDER_INTERVAL_SEC)
    tick(node)
    assert spoken(node) == ["AED를 반출해 주세요"] * 2
    assert "returning" not in phases(node)


def test_forced_return_after_timeout(node, clock):
    notice(node)

    clock.advance(FORCED_RETURN_TIMEOUT_SEC)
    tick(node)

    assert phases(node)[-1] == "returning"
    assert spoken(node)[-1] == "홈으로 복귀합니다"
    assert not node._armed
    assert nav_client().goals  # 복귀 goal 전송


def test_forced_return_wins_over_reminder_at_same_tick(node, clock):
    notice(node)

    clock.advance(FORCED_RETURN_TIMEOUT_SEC)  # 180초는 60초 안내 시점과 겹친다
    tick(node)

    assert "AED를 반출해 주세요" not in spoken(node)


def test_return_after_removal_waits_for_the_delay(node, clock):
    notice(node)
    remove_payload(node, clock)

    clock.advance(RETURN_DELAY_AFTER_PAYLOAD_REMOVED_SEC - 1)
    tick(node)
    assert "returning" not in phases(node)

    clock.advance(1.5)
    tick(node)
    assert phases(node)[-1] == "returning"


def test_no_reminders_after_removal(node, clock):
    notice(node)
    remove_payload(node, clock)

    clock.advance(REMINDER_INTERVAL_SEC)
    tick(node)

    assert "AED를 반출해 주세요" not in spoken(node)


def test_tick_does_nothing_when_not_armed(node, clock):
    clock.advance(10_000)
    tick(node)

    assert phases(node) == []
    assert spoken(node) == []


# ---- 복귀 goal -----------------------------------------------------------------


def start_return(node, clock, trajectory=None):
    notice(node, trajectory=trajectory)
    clock.advance(FORCED_RETURN_TIMEOUT_SEC)
    tick(node)


def test_single_goal_targets_home_pose(node, clock):
    start_return(node, clock)

    [goal] = nav_client().goals
    pose = goal.pose
    assert pose.header.frame_id == "map"
    assert (pose.pose.position.x, pose.pose.position.y) == (1.0, 2.0)
    # yaw = π → 쿼터니언 (z, w) = (1, ~0)
    assert pose.pose.orientation.z == pytest.approx(1.0)
    assert pose.pose.orientation.w == pytest.approx(0.0, abs=1e-9)
    assert not through_client().goals


def test_trajectory_is_driven_before_home_in_one_goal(node, clock):
    start_return(node, clock, trajectory=[{"x": 5.0, "y": 6.0}, {"x": 7.0, "y": 8.0, "yaw": 1.0}])

    [goal] = through_client().goals
    assert [(p.pose.position.x, p.pose.position.y) for p in goal.poses] == [(5.0, 6.0), (7.0, 8.0), (1.0, 2.0)]
    assert not nav_client().goals


def test_trajectory_steps_without_coordinates_are_dropped(node, clock):
    start_return(node, clock, trajectory=[{"x": 5.0}, {"foo": 1}, {"x": 7.0, "y": 8.0}])

    [goal] = through_client().goals
    assert len(goal.poses) == 2  # 유효한 한 점 + HOME


def test_trajectory_feedback_reports_progress(node, clock):
    start_return(node, clock, trajectory=[{"x": 5.0, "y": 6.0}, {"x": 7.0, "y": 8.0}])
    callback = through_client().feedback_callbacks[0]
    feedback = fake_ros._Msg(feedback=fake_ros._Msg(number_of_poses_remaining=2))

    callback(feedback)

    last = statuses(node)[-1]
    assert last["phase"] == "returning"
    assert last["waypoint_index"] == 1  # 전체 3점 중 2점 남음 → 1점 통과
    assert last["poses_total"] == 3


@pytest.mark.parametrize("trajectory", [None, [{"x": 5.0, "y": 6.0}]])
def test_missing_action_server_reports_failure(node, clock, trajectory):
    FakeActionClient.server_ready = False

    start_return(node, clock, trajectory=trajectory)

    assert phases(node)[-1] == "return_failed"
    assert all(not c.goals for c in FakeActionClient.instances)


def test_rejected_goal_reports_failure(node, clock):
    start_return(node, clock)

    nav_client().futures[0].resolve(FakeGoalHandle(accepted=False))

    assert phases(node)[-1] == "return_failed"
    assert node._goal_handle is None


def test_missing_goal_handle_reports_failure(node, clock):
    start_return(node, clock)

    nav_client().futures[0].resolve(None)

    assert phases(node)[-1] == "return_failed"


def accept_goal(node, clock):
    start_return(node, clock)
    handle = FakeGoalHandle(accepted=True)
    nav_client().futures[0].resolve(handle)
    return handle


def test_successful_return_announces_and_requests_docking(node, clock):
    handle = accept_goal(node, clock)

    handle.result_future.resolve(FakeResult(GoalStatus.STATUS_SUCCEEDED))

    assert spoken(node)[-1] == "복귀했습니다"
    [dock] = node.messages(DOCK)
    assert dock.data is True
    assert node._goal_handle is None
    assert "return_failed" not in phases(node)


@pytest.mark.parametrize("status", [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED])
def test_unsuccessful_return_reports_failure_without_docking(node, clock, status):
    handle = accept_goal(node, clock)

    handle.result_future.resolve(FakeResult(status))

    assert spoken(node)[-1] == "오류 발생"
    assert phases(node)[-1] == "return_failed"
    assert node.messages(DOCK) == []


def test_audio_commands_are_spoken_tts(node, clock):
    start_return(node, clock)

    message = node.messages(AUDIO)[-1]
    assert message.type == AudioCommand.TYPE_TTS
    assert message.volume == 1.0
    assert message.repeat == 1


# ---- 장애물 안내 ---------------------------------------------------------------


def test_obstacle_stop_is_announced_once_while_returning(node, clock):
    accept_goal(node, clock)
    spoken_before = len(spoken(node))

    node.deliver(OBSTACLE, Bool(data=True))
    node.deliver(OBSTACLE, Bool(data=True))  # 계속 정지 중 → 반복 안내 없음

    assert spoken(node)[spoken_before:] == ["비켜주세요"]

    node.deliver(OBSTACLE, Bool(data=False))
    node.deliver(OBSTACLE, Bool(data=True))  # 다시 막힘 → 다시 안내
    assert spoken(node)[spoken_before:] == ["비켜주세요", "비켜주세요"]


def test_obstacle_stop_is_silent_when_not_returning(node):
    node.deliver(OBSTACLE, Bool(data=True))

    assert spoken(node) == []


# ---- 로컬 버퍼와 재전송 ---------------------------------------------------------


def buffered(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_every_status_is_buffered_locally(node):
    notice(node)

    assert [item["phase"] for item in buffered(arm_module.PENDING_EVENTS_PATH)] == ["armed"]


def test_events_are_replayed_when_pc_reconnects(node):
    notice(node)
    node.published.clear()
    node.subscriber_count = 0
    tick(node)  # 구독자 없음 → 재전송 안 함
    assert node.messages(STATUS) == []

    node.subscriber_count = 1
    tick(node)  # 0 → 1 전환

    [replayed] = statuses(node)
    assert replayed["phase"] == "armed"
    assert replayed["replayed"] is True
    assert not arm_module.PENDING_EVENTS_PATH.exists()  # 재전송 후 비운다


def test_no_replay_while_subscriber_stays_connected(node):
    node.subscriber_count = 1
    tick(node)  # 첫 tick 에서 0 → 1 전환(버퍼 없음)
    notice(node)
    node.published.clear()

    tick(node)
    tick(node)

    assert node.messages(STATUS) == []


def test_replay_without_buffer_file_is_harmless(node):
    node.subscriber_count = 1

    tick(node)

    assert node.messages(STATUS) == []


def test_replay_skips_corrupt_lines_and_keeps_order(node):
    path = arm_module.PENDING_EVENTS_PATH
    path.parent.mkdir(parents=True)
    path.write_text('{"phase": "armed"}\nnot json\n\n{"phase": "returning"}\n', encoding="utf-8")
    node.subscriber_count = 1

    tick(node)

    assert phases(node) == ["armed", "returning"]
    assert all(item["replayed"] for item in statuses(node))


def test_buffer_write_failure_is_logged_not_raised(node, tmp_path, monkeypatch):
    blocker = tmp_path / "blocker"
    blocker.write_text("file, not a directory")
    monkeypatch.setattr(arm_module, "PENDING_EVENTS_PATH", blocker / "events.jsonl")

    notice(node)  # 버퍼링에 실패해도 PC로의 발행은 계속된다

    assert phases(node) == ["armed"]
    assert any("버퍼" in text for text in node.log_text("error"))
