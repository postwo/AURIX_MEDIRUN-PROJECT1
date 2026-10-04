import json

import pytest
from fake_ros import Bool, String

from medirun_mission.mission_manager import MissionManager

DISPATCH = "/medirun/mission/dispatch_command"
NAV_STATUS = "/medirun/navigation/status"
PAYLOAD_EVENT = "/medirun/mission/payload_event"
DOCK_RESULT = "/medirun/vision/dock_result"
UI_COMMAND = "/medirun/ui/mission_command"
STATE = "/medirun/mission/state"
NAV_GOAL = "/medirun/navigation/goal"
NAV_CANCEL = "/medirun/navigation/cancel"
DOCK_REQUEST = "/medirun/vision/dock_request"

ROUTE = {"target_waypoint_id": "BTN_01", "frame_id": "map", "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0}}


def send(node, topic, **payload):
    node.deliver(topic, String(data=json.dumps(payload)))


def dispatch(node, mission_id="M-1"):
    send(node, DISPATCH, mission_id=mission_id, button_id="BTN_01", target_waypoint_id="BTN_01", route=ROUTE)


def states(node):
    return [json.loads(msg.data)["state"] for msg in node.messages(STATE)]


def goals(node):
    return [json.loads(msg.data) for msg in node.messages(NAV_GOAL)]


def tick(node):
    node.timers[0][1]()


@pytest.fixture
def node(clock):
    return MissionManager()


@pytest.fixture
def navigating(node):
    dispatch(node)
    return node


@pytest.fixture
def arrived(navigating):
    send(navigating, NAV_STATUS, status="SUCCEEDED", goal_type="service_target")
    return navigating


def test_starts_idle_and_publishes_state(node):
    assert states(node) == ["IDLE"]


def test_dispatch_sends_goal_to_the_route_target(node):
    dispatch(node)

    assert states(node)[-3:] == ["IDLE", "DISPATCHED", "NAVIGATING"]
    [goal] = goals(node)
    assert goal["goal_type"] == "service_target"
    assert goal["waypoint_id"] == "BTN_01"
    assert goal["mission_id"] == "M-1"
    assert goal["route"] == ROUTE


def test_new_dispatch_is_ignored_while_busy(navigating):
    dispatch(navigating, mission_id="M-2")

    assert len(goals(navigating)) == 1
    assert any("ignored" in text for text in navigating.log_text("warning"))


def test_arrival_moves_to_arrived(navigating):
    send(navigating, NAV_STATUS, status="SUCCEEDED", goal_type="service_target")

    assert states(navigating)[-1] == "ARRIVED"


def test_arrival_result_is_ignored_unless_navigating(node):
    send(node, NAV_STATUS, status="SUCCEEDED", goal_type="service_target")

    assert states(node) == ["IDLE"]


def test_success_with_unexpected_goal_type_is_ignored(navigating):
    send(navigating, NAV_STATUS, status="SUCCEEDED", goal_type="something_else")

    assert states(navigating)[-1] == "NAVIGATING"


def test_navigation_failure_fails_mission(navigating):
    send(navigating, NAV_STATUS, status="FAILED")

    assert states(navigating)[-1] == "FAILED"


def test_other_navigation_statuses_do_not_change_state(navigating):
    send(navigating, NAV_STATUS, status="ACCEPTED")
    send(navigating, NAV_STATUS, status="CANCELED")

    assert states(navigating)[-1] == "NAVIGATING"


def test_arrival_timeout_triggers_return_home(arrived, clock):
    clock.advance(179)
    tick(arrived)
    assert states(arrived)[-1] == "ARRIVED"

    clock.advance(2)  # 합계 181초 > 180초
    tick(arrived)

    assert states(arrived)[-1] == "RETURNING_HOME"
    assert goals(arrived)[-1]["goal_type"] == "return_home"
    assert goals(arrived)[-1]["waypoint_id"] == "HOME"


def test_payload_removal_shortens_the_wait(arrived, clock):
    clock.advance(100)
    send(arrived, PAYLOAD_EVENT, event="PAYLOAD_REMOVED")
    assert states(arrived)[-1] == "PAYLOAD_REMOVED"

    clock.advance(14)
    tick(arrived)
    assert states(arrived)[-1] == "PAYLOAD_REMOVED"

    clock.advance(2)  # 반출 후 15초 경과
    tick(arrived)
    assert states(arrived)[-1] == "RETURNING_HOME"


def test_payload_event_outside_arrived_is_ignored(navigating):
    send(navigating, PAYLOAD_EVENT, event="PAYLOAD_REMOVED")

    assert states(navigating)[-1] == "NAVIGATING"


def test_other_payload_events_are_ignored(arrived):
    send(arrived, PAYLOAD_EVENT, event="PAYLOAD_LOADED")

    assert states(arrived)[-1] == "ARRIVED"


def test_timeout_does_nothing_before_arrival(navigating, clock):
    clock.advance(10_000)
    tick(navigating)

    assert states(navigating)[-1] == "NAVIGATING"


def test_home_arrival_starts_docking(arrived):
    send(arrived, UI_COMMAND, command="return_home")

    send(arrived, NAV_STATUS, status="SUCCEEDED", goal_type="return_home")

    assert states(arrived)[-1] == "DOCKING"
    [dock] = arrived.messages(DOCK_REQUEST)
    assert dock.data is True


def test_dock_success_completes_mission(arrived):
    send(arrived, UI_COMMAND, command="return_home")
    send(arrived, NAV_STATUS, status="SUCCEEDED", goal_type="return_home")

    send(arrived, DOCK_RESULT, status="SUCCEEDED")

    assert states(arrived)[-2:] == ["DOCKED", "COMPLETED"]


def test_dock_failure_fails_mission(arrived):
    send(arrived, UI_COMMAND, command="return_home")
    send(arrived, NAV_STATUS, status="SUCCEEDED", goal_type="return_home")

    send(arrived, DOCK_RESULT, status="FAILED")

    assert states(arrived)[-1] == "FAILED"


def test_dock_result_is_ignored_unless_docking(arrived):
    send(arrived, DOCK_RESULT, status="SUCCEEDED")

    assert states(arrived)[-1] == "ARRIVED"


def test_operator_cancel_cancels_navigation_and_fails(navigating):
    send(navigating, UI_COMMAND, command="cancel")

    [cancel] = navigating.messages(NAV_CANCEL)
    assert cancel.data is True
    assert states(navigating)[-1] == "FAILED"


def test_operator_return_home_works_from_navigating(navigating):
    send(navigating, UI_COMMAND, command="return_home")

    assert states(navigating)[-1] == "RETURNING_HOME"
    assert goals(navigating)[-1]["waypoint_id"] == "HOME"


@pytest.mark.parametrize("busy_state", ["RETURNING_HOME", "DOCKING", "COMPLETED"])
def test_return_home_is_not_repeated(arrived, busy_state):
    send(arrived, UI_COMMAND, command="return_home")
    if busy_state in {"DOCKING", "COMPLETED"}:
        send(arrived, NAV_STATUS, status="SUCCEEDED", goal_type="return_home")
    if busy_state == "COMPLETED":
        send(arrived, DOCK_RESULT, status="SUCCEEDED")
    goal_count = len(goals(arrived))

    send(arrived, UI_COMMAND, command="return_home")

    assert len(goals(arrived)) == goal_count
    assert states(arrived)[-1] == busy_state


@pytest.mark.parametrize("final", ["COMPLETED", "FAILED"])
def test_new_dispatch_is_accepted_after_a_finished_mission(node, final):
    dispatch(node, mission_id="M-1")
    if final == "FAILED":
        send(node, NAV_STATUS, status="FAILED")
    else:
        send(node, NAV_STATUS, status="SUCCEEDED", goal_type="service_target")
        send(node, UI_COMMAND, command="return_home")
        send(node, NAV_STATUS, status="SUCCEEDED", goal_type="return_home")
        send(node, DOCK_RESULT, status="SUCCEEDED")
    assert states(node)[-1] == final

    dispatch(node, mission_id="M-2")

    assert states(node)[-1] == "NAVIGATING"
    assert goals(node)[-1]["mission_id"] == "M-2"


def test_state_message_describes_the_mission(navigating):
    last = json.loads(navigating.messages(STATE)[-1].data)

    assert last["mission_id"] == "M-1"
    assert last["state"] == "NAVIGATING"
    assert last["button_id"] == "BTN_01"
    assert last["route"] == ROUTE


def test_malformed_dispatch_does_not_wedge_the_manager(node):
    send(node, DISPATCH, mission_id="BAD")  # route 없음

    assert states(node) == ["IDLE"]  # 상태가 바뀌지 않는다
    dispatch(node, mission_id="M-2")  # 다음 정상 호출은 처리된다

    assert states(node)[-1] == "NAVIGATING"
    assert goals(node)[-1]["mission_id"] == "M-2"


@pytest.mark.parametrize(
    "raw",
    [
        "{not json",
        "[1, 2, 3]",
        '"just a string"',
        "42",
        json.dumps({"mission_id": "M", "route": "not a dict"}),
        json.dumps({"mission_id": "M", "route": {}}),
        json.dumps({"mission_id": "M", "route": {"target_waypoint_id": ""}}),
        json.dumps({"mission_id": "M", "route": {"target_waypoint_id": 7}}),
        json.dumps({"mission_id": "M", "route": None}),
    ],
)
def test_unusable_dispatch_is_ignored_with_warning(node, raw):
    node.deliver(DISPATCH, String(data=raw))

    assert states(node) == ["IDLE"]
    assert goals(node) == []
    assert node.log_text("warning")
    dispatch(node)  # 이후 정상 호출은 문제없이 처리된다
    assert states(node)[-1] == "NAVIGATING"


def test_empty_dispatch_message_is_ignored(node):
    node.deliver(DISPATCH, String(data=""))  # 빈 문자열은 {} 로 읽히지만 route 가 없다

    assert states(node) == ["IDLE"]


def test_bad_dispatch_does_not_overwrite_a_finished_mission(node):
    dispatch(node, mission_id="M-1")
    send(node, NAV_STATUS, status="FAILED")
    before = json.loads(node.messages(STATE)[-1].data)

    send(node, DISPATCH, mission_id="BAD")  # route 없음 — 이전 미션 정보를 덮어쓰면 안 된다

    assert node._mission["mission_id"] == "M-1"
    assert json.loads(node.messages(STATE)[-1].data) == before


@pytest.mark.parametrize("topic", [NAV_STATUS, PAYLOAD_EVENT, DOCK_RESULT, UI_COMMAND])
@pytest.mark.parametrize("raw", ["{broken", "[1]", "null"])
def test_garbage_on_other_topics_is_ignored(arrived, topic, raw):
    arrived.deliver(topic, String(data=raw))

    assert states(arrived)[-1] == "ARRIVED"
    assert arrived.log_text("warning")
