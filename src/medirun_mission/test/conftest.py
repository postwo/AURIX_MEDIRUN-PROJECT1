import sys
from pathlib import Path

import pytest

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))  # medirun_mission 패키지

import fake_ros  # noqa: E402

fake_ros.install()


@pytest.fixture(autouse=True)
def _reset_fakes():
    fake_ros.FakeNode.param_overrides = {}
    fake_ros.FakeActionClient.server_ready = True
    fake_ros.FakeActionClient.instances = []
    yield


class Clock:
    """time.monotonic 대체용 가짜 시계."""

    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


@pytest.fixture
def clock(monkeypatch):
    fake_clock = Clock()
    import time

    monkeypatch.setattr(time, "monotonic", fake_clock)
    return fake_clock
