import pytest

from medirun_server.waypoint_store import WaypointLookupError, WaypointStore


def test_merges_directory_and_resolves(waypoints_dir):
    (waypoints_dir / "zz_override.yaml").write_text(
        "frame_id: map\nwaypoints:\n  BTN_02: {x: 9.0, y: 9.0, kind: call_zone}\n", encoding="utf-8")
    store = WaypointStore.from_directory(waypoints_dir)
    assert store.call_zone_for_button("BTN_01").pose.x == pytest.approx(3.1)
    assert store.call_zone_for_button("BTN_02").pose.x == pytest.approx(9.0)  # 뒤 파일이 이긴다
    assert store.home().pose.y == pytest.approx(-2.02)


def test_unknown_button_raises(waypoints_dir):
    store = WaypointStore.from_directory(waypoints_dir)
    with pytest.raises(WaypointLookupError):
        store.call_zone_for_button("BTN_99")


def test_return_trajectory_skips_call_zone_itself(waypoints_dir):
    store = WaypointStore.from_directory(waypoints_dir)
    poses = store.return_trajectory("BTN_01")
    assert [(p.x, p.y) for p in poses] == [(2.0, 0.5)]
    assert store.return_trajectory("BTN_02") == []


def test_trajectory_with_missing_step_falls_back_to_direct(tmp_path):
    path = tmp_path / "w.yaml"
    path.write_text("waypoints:\n  BTN_01: {x: 1, y: 1}\ntrajectories:\n  t: [BTN_01, GHOST]\n", encoding="utf-8")
    assert WaypointStore.from_directory(path).return_trajectory("BTN_01") == []


def test_home_fallback_by_kind_and_ambiguity(tmp_path):
    path = tmp_path / "w.yaml"
    path.write_text("waypoints:\n  H1: {x: 1, y: 1, kind: home}\n", encoding="utf-8")
    assert WaypointStore.from_directory(path).home("HOME").waypoint_id == "H1"
    path.write_text("waypoints:\n  H1: {x: 1, y: 1, kind: home}\n  H2: {x: 2, y: 2, kind: home}\n", encoding="utf-8")
    with pytest.raises(WaypointLookupError):
        WaypointStore.from_directory(path).home("HOME")
