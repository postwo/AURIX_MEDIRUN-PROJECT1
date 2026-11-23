"""CentralServer 노드 시험(가짜 rclpy). 실제 토픽 이름·JSON 형식이 Pi/대시보드와 맞는지 본다."""
import json

import pytest
from fake_ros import (
    BatteryState, Bool, FakeActionClient, FakeGoalHandle, FakeNode, FakeResult, GoalStatus,
    PoseWithCovarianceStamped, String,
)

from medirun_server.central_server import CentralServer

DISPATCH_REQUEST = "/medirun/mission/dispatch_request"
PI_STATUS = "/medirun/mission/pi_return_status"
DOCK_RESULT = "/medirun/vision/dock_result"
ARRIVAL = "/medirun/mission/arrival_notice"
EVENTS = "/medirun/server/mission_event"
STATUS = "/medirun/server/status"
READINESS = "/medirun/system/readiness"
BATTERY = "/medirun/system/battery"
DOCK_REQUEST = "/medirun/vision/dock_request"


def send(node, topic, **payload):
    node.deliver(topic, String(data=json.dumps(payload)))


def phases(node):
    return [json.loads(m.data)["phase"] for m in node.messages(EVENTS)]


def nav(node) -> FakeActionClient:
    return FakeActionClient.instances[-1]


def tick(node):
    node.timers[0][1]()


def make_ready(node):
    node.deliver("/battery_state", BatteryState(percentage=90.0))
    node.deliver("/amcl_pose", PoseWithCovarianceStamped(x=1.0, y=1.0))
    node.deliver("/medirun/sensor/payload_loaded", Bool(data=True))
    tick(node)  # nav2_available 갱신


def accept_goal(node, index=-1):
    handle = FakeGoalHandle(accepted=True)
    nav(node).futures[index].resolve(handle)
    return handle


@pytest.fixture
def server(clock, tmp_path, waypoints_dir):
    FakeNode.param_overrides = {
        "waypoints_path": str(waypoints_dir),
        "log_dir": str(tmp_path / "logs"),
        "db_config_path": str(tmp_path / "missing.yaml"),
    }
    node = CentralServer()
    make_ready(node)
    return node


def dispatch(node, mission_id="M-1", button="BTN_01"):
    send(node, DISPATCH_REQUEST, mission_id=mission_id, button_id=button, request_source="sos_button")


def test_dispatch_sends_nav2_goal_to_call_zone(server):
    dispatch(server)
    assert phases(server) == ["QUEUED", "DISPATCHED"]
    goal = nav(server).goals[-1]
    assert (goal.pose.pose.position.x, goal.pose.pose.position.y) == (3.1, 1.09)
    assert goal.pose.header.frame_id == "map"
    accept_goal(server)
    assert phases(server)[-1] == "NAVIGATING"


def test_arrival_publishes_notice_in_pi_format(server):
    dispatch(server)
    handle = accept_goal(server)
    handle.result_future.resolve(FakeResult(GoalStatus.STATUS_SUCCEEDED))
    assert phases(server)[-1] == "ARRIVED"
    notice = json.loads(server.messages(ARRIVAL)[-1].data)
    # arrival_return_manager._handle_arrival_notice 가 기대하는 필드
    assert notice["armed"] is True and notice["mission_id"] == "M-1" and notice["queue_id"] == "Q-0001"
    assert notice["home_pose"]["x"] == 2.87 and notice["home_pose"]["frame_id"] == "map"
    assert [(p["x"], p["y"]) for p in notice["trajectory"]] == [(2.0, 0.5)]


def test_full_cycle_with_pi_status_and_docking(server):
    dispatch(server)
    accept_goal(server).result_future.resolve(FakeResult(GoalStatus.STATUS_SUCCEEDED))
    send(server, PI_STATUS, phase="armed", mission_id="M-1", queue_id="Q-0001")
    send(server, PI_STATUS, phase="payload_removed", mission_id="M-1", queue_id="Q-0001")
    send(server, PI_STATUS, phase="returning", mission_id="M-1", queue_id="Q-0001")
    send(server, DOCK_RESULT, status="SUCCEEDED")
    assert phases(server)[-4:] == ["ARRIVED", "PAYLOAD_REMOVED", "RETURNING", "COMPLETED"]


def test_second_call_waits_then_dispatches_after_completion(server):
    dispatch(server, "M-1", "BTN_01")
    dispatch(server, "M-2", "BTN_02")
    assert len(nav(server).goals) == 1
    accept_goal(server).result_future.resolve(FakeResult(GoalStatus.STATUS_SUCCEEDED))
    send(server, PI_STATUS, phase="returning", mission_id="M-1")
    send(server, DOCK_RESULT, status="SUCCEEDED")
    assert len(nav(server).goals) == 2
    second_goal = nav(server).goals[-1]
    assert second_goal.pose.pose.position.x == 2.22


def test_nav_failure_retries_then_fails(server):
    dispatch(server)
    for _ in range(3):
        accept_goal(server).result_future.resolve(FakeResult(GoalStatus.STATUS_ABORTED))
    assert len(nav(server).goals) == 3  # 최초 + 재계획 2회
    assert phases(server)[-1] == "FAILED"


def test_unknown_button_fails_without_goal(server):
    dispatch(server, button="BTN_99")
    assert phases(server)[-1] == "FAILED"
    assert nav(server).goals == []


def test_offline_robot_holds_call_until_ready(clock, server):
    clock.now += 30.0  # 하트비트 끊김
    tick(server)
    dispatch(server)
    assert phases(server) == ["QUEUED"]
    assert "ROBOT_OFFLINE" in server.messages(READINESS)[-1].data
    make_ready(server)
    assert phases(server)[-1] == "DISPATCHED"


def test_cancel_service_cancels_goal_and_disarms_pi(server):
    dispatch(server)
    handle = accept_goal(server)
    response = server.call_service("/medirun/server/cancel_mission")
    assert response.success and handle.cancel_requested
    assert json.loads(server.messages(ARRIVAL)[-1].data)["armed"] is False
    assert phases(server)[-1] == "CANCELLED"
    # 늦게 도착한 옛 goal 결과는 무시된다
    handle.result_future.resolve(FakeResult(GoalStatus.STATUS_CANCELED))
    assert phases(server)[-1] == "CANCELLED"


def test_return_home_service_sends_home_goal_and_docks(server):
    dispatch(server)
    accept_goal(server).result_future.resolve(FakeResult(GoalStatus.STATUS_SUCCEEDED))
    response = server.call_service("/medirun/server/return_home")
    assert response.success
    home_goal = nav(server).goals[-1]
    assert home_goal.pose.pose.position.x == 2.87
    accept_goal(server).result_future.resolve(FakeResult(GoalStatus.STATUS_SUCCEEDED))
    assert server.messages(DOCK_REQUEST)[-1].data is True
    send(server, DOCK_RESULT, status="SUCCEEDED")
    assert phases(server)[-1] == "COMPLETED"


def test_return_home_rejected_before_goal_accepted(server):
    dispatch(server)
    response = server.call_service("/medirun/server/return_home")
    assert not response.success


def test_status_and_health_topics(server):
    tick(server)
    status = json.loads(server.messages(STATUS)[-1].data)
    assert status["dispatch_mode"] == "active"
    assert status["robot"]["battery_percent"] == 90.0
    assert server.messages(READINESS)[-1].data == "READY"
    assert server.messages(BATTERY)[-1].data == "90.0"
    payload = json.loads(server.call_service("/medirun/server/get_status").message)
    assert payload["robot"]["online"] is True


def test_monitor_mode_never_sends_goals(clock, tmp_path, waypoints_dir):
    FakeNode.param_overrides = {
        "dispatch_mode": "monitor",
        "waypoints_path": str(waypoints_dir),
        "log_dir": str(tmp_path / "logs"),
        "db_config_path": str(tmp_path / "missing.yaml"),
    }
    node = CentralServer()
    make_ready(node)
    dispatch(node)
    assert nav(node).goals == [] and phases(node) == []
    assert not node.call_service("/medirun/server/cancel_mission").success


def test_events_are_written_to_jsonl(server, tmp_path):
    dispatch(server)
    files = list((tmp_path / "logs").glob("server_events_*.jsonl"))
    lines = [json.loads(line) for line in files[0].read_text(encoding="utf-8").splitlines()]
    assert [line["event_type"] for line in lines][-2:] == ["mission_queued", "mission_dispatched"]


def test_malformed_json_is_ignored(server):
    server.deliver(DISPATCH_REQUEST, String(data="not json"))
    server.deliver(PI_STATUS, String(data="[]"))
    assert phases(server) == []
