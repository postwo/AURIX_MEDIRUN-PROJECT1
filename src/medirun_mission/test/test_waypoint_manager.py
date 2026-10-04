import json

import fake_ros
import pytest
from fake_ros import String

from medirun_mission.waypoint_manager import WaypointManager

REQUEST = "/medirun/mission/dispatch_request"
COMMAND = "/medirun/mission/dispatch_command"

GUI_FILE = """\
frame_id: odom
waypoints:
  BTN_01:
    x: 1.5
    y: -2.0
    yaw: 0.5
    kind: call_zone
    label: 병실1
  HOME:
    x: 0.0
    y: 0.0
    kind: home
trajectories: {}
"""


def make_node(tmp_path, content=GUI_FILE):
    path = tmp_path / "waypoints.yaml"
    path.write_text(content, encoding="utf-8")
    fake_ros.FakeNode.param_overrides = {"waypoints_file": str(path)}
    return WaypointManager()


def request(node, **payload):
    node.deliver(REQUEST, String(data=json.dumps(payload)))


def commands(node):
    return [json.loads(msg.data) for msg in node.messages(COMMAND)]


def test_loads_every_waypoint_as_a_route(tmp_path):
    node = make_node(tmp_path)

    assert set(node._waypoints) == {"BTN_01", "HOME"}
    assert any("2 routes" in text for text in node.log_text("info"))


def test_route_carries_frame_pose_and_target(tmp_path):
    node = make_node(tmp_path)

    assert node._waypoints["BTN_01"] == {
        "target_waypoint_id": "BTN_01",
        "frame_id": "odom",
        "pose": {"x": 1.5, "y": -2.0, "yaw": 0.5},
    }


def test_missing_yaw_defaults_to_zero(tmp_path):
    node = make_node(tmp_path)

    assert node._waypoints["HOME"]["pose"]["yaw"] == 0.0


def test_missing_frame_id_defaults_to_map(tmp_path):
    node = make_node(tmp_path, "waypoints:\n  A:\n    x: 1\n    y: 2\n")

    assert node._waypoints["A"]["frame_id"] == "map"


@pytest.mark.parametrize("content", ["", "frame_id: map\nwaypoints: {}\ntrajectories: {}\n"])
def test_empty_files_give_no_routes(tmp_path, content):
    node = make_node(tmp_path, content)

    assert node._waypoints == {}


def test_missing_file_raises_file_not_found(tmp_path):
    fake_ros.FakeNode.param_overrides = {"waypoints_file": str(tmp_path / "nope.yaml")}

    with pytest.raises(FileNotFoundError):
        WaypointManager()


def test_known_button_is_forwarded_with_its_route(tmp_path):
    node = make_node(tmp_path)

    request(node, button_id="BTN_01", mission_id="M-1", request_source="sos_button")

    [command] = commands(node)
    assert command["mission_id"] == "M-1"
    assert command["request_source"] == "sos_button"  # 원래 필드는 그대로 전달
    assert command["target_waypoint_id"] == "BTN_01"
    assert command["route"]["pose"] == {"x": 1.5, "y": -2.0, "yaw": 0.5}


def test_unknown_button_is_dropped_with_warning(tmp_path):
    node = make_node(tmp_path)

    request(node, button_id="BTN_99", mission_id="M-1")

    assert commands(node) == []
    assert any("BTN_99" in text for text in node.log_text("warning"))


def test_request_without_button_id_is_dropped(tmp_path):
    node = make_node(tmp_path)

    request(node, mission_id="M-1")

    assert commands(node) == []
