import json

import pytest

from medirun_mission.json_topic import dump_json_message, load_json_message


def test_empty_message_is_empty_dict():
    assert load_json_message("") == {}


def test_loads_valid_json():
    assert load_json_message('{"a": 1, "b": "x"}') == {"a": 1, "b": "x"}


def test_invalid_json_raises_value_error():
    with pytest.raises(ValueError):
        load_json_message("{not json")


def test_dump_is_sorted_and_ascii_only():
    text = dump_json_message({"b": 1, "a": "병실"})
    assert text == '{"a": "\\ubcd1\\uc2e4", "b": 1}'
    assert text.isascii()


def test_round_trip_keeps_korean_and_nested_values():
    payload = {"route": {"pose": {"x": 1.5, "yaw": 0.0}}, "name": "병실 1", "ok": True, "none": None}
    assert load_json_message(dump_json_message(payload)) == payload
    assert json.loads(dump_json_message(payload)) == payload
