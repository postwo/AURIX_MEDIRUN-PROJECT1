import json

import fake_ros
import pytest
from fake_ros import Bool, String

from medirun_mission.emergency_dispatcher import EmergencyDispatcher

BUTTON = "/medirun/sensor/sos_button"
PAYLOAD = "/medirun/sensor/payload_loaded"
READINESS = "/medirun/system/readiness"
BATTERY = "/medirun/system/battery"
DISPATCH = "/medirun/mission/dispatch_request"
REJECT = "/medirun/mission/dispatch_rejection"


@pytest.fixture
def node(clock):
    return EmergencyDispatcher()


def press(node, button_id="BTN_01"):
    node.deliver(BUTTON, String(data=button_id))


def rejections(node):
    return [json.loads(msg.data) for msg in node.messages(REJECT)]


def dispatches(node):
    return [json.loads(msg.data) for msg in node.messages(DISPATCH)]


def test_accepts_call_when_everything_is_ready(node):
    press(node, "BTN_07")

    assert rejections(node) == []
    [request] = dispatches(node)
    assert request["button_id"] == "BTN_07"
    assert request["request_source"] == "sos_button"
    assert request["mission_id"]  # uuid


def test_each_dispatch_gets_a_unique_mission_id(node, clock):
    press(node)
    clock.advance(5)
    press(node)

    first, second = dispatches(node)
    assert first["mission_id"] != second["mission_id"]


def test_second_press_inside_cooldown_is_rejected(node, clock):
    press(node)
    clock.advance(1.0)
    press(node)

    assert len(dispatches(node)) == 1
    assert rejections(node) == [{"reason": "COOLDOWN", "button_id": "BTN_01"}]


def test_press_after_cooldown_is_accepted(node, clock):
    press(node)
    clock.advance(2.0)
    press(node)

    assert len(dispatches(node)) == 2


def test_cooldown_parameter_can_be_changed(clock):
    fake_ros.FakeNode.param_overrides = {"call_cooldown_sec": 0.0}
    node = EmergencyDispatcher()

    press(node)
    press(node)

    assert len(dispatches(node)) == 2


def test_missing_payload_is_rejected(node):
    node.deliver(PAYLOAD, Bool(data=False))
    press(node)

    assert rejections(node) == [{"reason": "PAYLOAD_MISSING", "button_id": "BTN_01"}]
    assert dispatches(node) == []


def test_low_battery_is_rejected_but_threshold_is_allowed(node, clock):
    node.deliver(BATTERY, String(data="29.9"))
    press(node)
    assert rejections(node)[-1]["reason"] == "LOW_BATTERY"

    node.deliver(BATTERY, String(data="30.0"))
    clock.advance(5)
    press(node)
    assert len(dispatches(node)) == 1


def test_system_not_ready_is_rejected_until_ready(node, clock):
    node.deliver(READINESS, String(data="NOT_OK"))
    press(node)
    assert rejections(node)[-1]["reason"] == "SYSTEM_NOT_READY"

    node.deliver(READINESS, String(data="system READY"))
    clock.advance(5)
    press(node)
    assert len(dispatches(node)) == 1


def test_rejection_order_is_cooldown_payload_battery_readiness(node, clock):
    node.deliver(PAYLOAD, Bool(data=False))
    node.deliver(BATTERY, String(data="1"))
    node.deliver(READINESS, String(data="NO"))

    press(node)
    assert rejections(node)[-1]["reason"] == "PAYLOAD_MISSING"

    node.deliver(PAYLOAD, Bool(data=True))
    press(node)
    assert rejections(node)[-1]["reason"] == "LOW_BATTERY"

    node.deliver(BATTERY, String(data="100"))
    press(node)
    assert rejections(node)[-1]["reason"] == "SYSTEM_NOT_READY"


def test_rejected_press_does_not_start_the_cooldown(node):
    node.deliver(PAYLOAD, Bool(data=False))
    press(node)
    node.deliver(PAYLOAD, Bool(data=True))

    press(node)  # 직전 거부는 쿨다운을 소모하지 않는다

    assert len(dispatches(node)) == 1


def test_invalid_battery_value_keeps_last_known_value(node, clock):
    node.deliver(BATTERY, String(data="10"))
    node.deliver(BATTERY, String(data="garbage"))
    press(node)

    # 배터리 10%를 기억하고 있으므로 여전히 거부해야 한다(100%로 간주하면 안 된다).
    assert rejections(node)[-1]["reason"] == "LOW_BATTERY"
    assert any("garbage" in text for text in node.log_text("warning"))


def test_valid_battery_value_replaces_previous(node, clock):
    node.deliver(BATTERY, String(data="10"))
    node.deliver(BATTERY, String(data="80"))
    press(node)

    assert len(dispatches(node)) == 1
