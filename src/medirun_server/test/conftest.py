import sys
from pathlib import Path

import pytest

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))  # medirun_server 패키지

import fake_ros  # noqa: E402

fake_ros.install()


@pytest.fixture(autouse=True)
def _reset_fakes():
    fake_ros.FakeNode.param_overrides = {}
    fake_ros.FakeActionClient.server_ready = True
    fake_ros.FakeActionClient.instances = []
    yield


class Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def clock(monkeypatch):
    fake = Clock()
    import medirun_server.central_server as central_server

    monkeypatch.setattr(central_server, "monotonic", fake)
    return fake


WAYPOINTS_YAML = """
frame_id: map
waypoints:
  BTN_01: {x: 3.1, y: 1.09, yaw: -3.14, kind: call_zone, label: call_01}
  BTN_02: {x: 2.22, y: -0.64, yaw: 1.17, kind: call_zone, label: call_02}
  W-01: {x: 2.0, y: 0.5, yaw: 0.0, kind: waypoint}
  HOME: {x: 2.87, y: -2.02, yaw: 1.45, kind: home, label: home}
trajectories:
  btn1_home: [BTN_01, W-01]
"""


@pytest.fixture
def waypoints_dir(tmp_path):
    directory = tmp_path / "waypoints"
    directory.mkdir()
    (directory / "all.yaml").write_text(WAYPOINTS_YAML, encoding="utf-8")
    return directory
