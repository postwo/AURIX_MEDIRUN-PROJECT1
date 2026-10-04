"""ROS 2 없이 medirun_mission 노드를 시험하기 위한 최소 대역(rclpy 등).

노드가 쓰는 부분(파라미터, publisher/subscription/timer, 액션 클라이언트, 메시지 타입)만
흉내 낸다. install() 이 sys.modules 에 가짜 모듈을 등록한다.
"""
from __future__ import annotations

import sys
import types
from typing import Any, Callable, Dict, List, Tuple


# ---- 메시지 ---------------------------------------------------------------------
class _Msg:
    def __init__(self, **kwargs: Any) -> None:
        for key, value in kwargs.items():
            setattr(self, key, value)


class String(_Msg):
    data = ""


class Bool(_Msg):
    data = False


class Int32(_Msg):
    data = 0


class _Vec(_Msg):
    x = 0.0
    y = 0.0
    z = 0.0


class Twist(_Msg):
    def __init__(self, **kwargs: Any) -> None:
        self.linear = _Vec()
        self.angular = _Vec()
        super().__init__(**kwargs)


class _Header(_Msg):
    def __init__(self) -> None:
        self.frame_id = ""
        self.stamp = None


class _Pose(_Msg):
    def __init__(self) -> None:
        self.position = _Vec()
        self.orientation = _Vec()
        self.orientation.w = 1.0


class PoseStamped(_Msg):
    def __init__(self) -> None:
        self.header = _Header()
        self.pose = _Pose()


class AudioCommand(_Msg):
    TYPE_TTS = 1

    def __init__(self, **kwargs: Any) -> None:
        self.type = 0
        self.text = ""
        self.volume = 0.0
        self.repeat = 0
        super().__init__(**kwargs)


class GoalStatus:
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class NavigateToPose:
    class Goal(_Msg):
        def __init__(self) -> None:
            self.pose = PoseStamped()


class NavigateThroughPoses:
    class Goal(_Msg):
        def __init__(self) -> None:
            self.poses: List[PoseStamped] = []


# ---- rclpy ----------------------------------------------------------------------
class FakeFuture:
    def __init__(self) -> None:
        self._result: Any = None
        self._callbacks: List[Callable[["FakeFuture"], None]] = []

    def result(self) -> Any:
        return self._result

    def add_done_callback(self, callback: Callable[["FakeFuture"], None]) -> None:
        self._callbacks.append(callback)

    def resolve(self, result: Any) -> None:
        self._result = result
        for callback in list(self._callbacks):
            callback(self)


class FakePublisher:
    def __init__(self, node: "FakeNode", topic: str) -> None:
        self._node = node
        self.topic = topic

    def publish(self, msg: Any) -> None:
        self._node.published.append((self.topic, msg))

    def get_subscription_count(self) -> int:
        return self._node.subscriber_count


class _Logger:
    def __init__(self, node: "FakeNode") -> None:
        self._node = node

    def _log(self, level: str, text: str) -> None:
        self._node.logs.append((level, text))

    def info(self, text: str) -> None:
        self._log("info", text)

    def warning(self, text: str) -> None:
        self._log("warning", text)

    def warn(self, text: str) -> None:
        self._log("warning", text)

    def error(self, text: str) -> None:
        self._log("error", text)

    def debug(self, text: str) -> None:
        self._log("debug", text)


class _Param:
    def __init__(self, value: Any) -> None:
        self.value = value


class _Time:
    def to_msg(self) -> str:
        return "stamp"


class _Clock:
    def now(self) -> _Time:
        return _Time()


class FakeNode:
    """rclpy.node.Node 대역. 시험에서는 published/subscriptions/timers/logs 를 직접 검사한다."""

    param_overrides: Dict[str, Any] = {}

    def __init__(self, name: str = "node") -> None:
        self.name = name
        self.published: List[Tuple[str, Any]] = []
        self.logs: List[Tuple[str, str]] = []
        self.subscriptions: Dict[str, Callable[[Any], None]] = {}
        self.timers: List[Tuple[float, Callable[[], None]]] = []
        self.subscriber_count = 0
        self._params: Dict[str, Any] = {}

    def declare_parameter(self, name: str, default: Any = None) -> _Param:
        self._params[name] = type(self).param_overrides.get(name, default)
        return _Param(self._params[name])

    def get_parameter(self, name: str) -> _Param:
        return _Param(self._params[name])

    def set_parameter_value(self, name: str, value: Any) -> None:
        self._params[name] = value

    def create_publisher(self, _type: Any, topic: str, _qos: Any = None) -> FakePublisher:
        return FakePublisher(self, topic)

    def create_subscription(self, _type: Any, topic: str, callback: Callable[[Any], None], _qos: Any = None) -> None:
        self.subscriptions[topic] = callback

    def create_timer(self, period: float, callback: Callable[[], None]) -> None:
        self.timers.append((period, callback))

    def get_logger(self) -> _Logger:
        return _Logger(self)

    def get_clock(self) -> _Clock:
        return _Clock()

    def destroy_node(self) -> None:
        pass

    # ---- 시험 보조 ----
    def deliver(self, topic: str, msg: Any) -> None:
        self.subscriptions[topic](msg)

    def messages(self, topic: str) -> List[Any]:
        return [msg for published_topic, msg in self.published if published_topic == topic]

    def log_text(self, level: str) -> List[str]:
        return [text for log_level, text in self.logs if log_level == level]


class FakeActionClient:
    server_ready = True
    instances: List["FakeActionClient"] = []

    def __init__(self, _node: Any, action_type: Any, name: str) -> None:
        self.action_type = action_type
        self.name = name
        self.goals: List[Any] = []
        self.feedback_callbacks: List[Any] = []
        self.futures: List[FakeFuture] = []
        type(self).instances.append(self)

    def wait_for_server(self, timeout_sec: float = 0.0) -> bool:
        return type(self).server_ready

    def send_goal_async(self, goal: Any, feedback_callback: Any = None) -> FakeFuture:
        self.goals.append(goal)
        self.feedback_callbacks.append(feedback_callback)
        future = FakeFuture()
        self.futures.append(future)
        return future


class FakeGoalHandle:
    def __init__(self, accepted: bool = True) -> None:
        self.accepted = accepted
        self.result_future = FakeFuture()

    def get_result_async(self) -> FakeFuture:
        return self.result_future


class FakeResult:
    def __init__(self, status: int) -> None:
        self.status = status


def _module(name: str, **attrs: Any) -> types.ModuleType:
    module = types.ModuleType(name)
    module.__dict__.update(attrs)
    sys.modules[name] = module
    return module


def install() -> None:
    """가짜 ROS 모듈을 sys.modules 에 등록한다(실제 ROS 설치 여부와 무관하게 항상 대역을 쓴다)."""
    _module("rclpy", init=lambda *a, **k: None, spin=lambda *a, **k: None, shutdown=lambda *a, **k: None)
    _module("rclpy.node", Node=FakeNode)
    _module("rclpy.action", ActionClient=FakeActionClient)
    _module("std_msgs")
    _module("std_msgs.msg", String=String, Bool=Bool, Int32=Int32)
    _module("geometry_msgs")
    _module("geometry_msgs.msg", Twist=Twist, PoseStamped=PoseStamped)
    _module("action_msgs")
    _module("action_msgs.msg", GoalStatus=GoalStatus)
    _module("nav2_msgs")
    _module("nav2_msgs.action", NavigateToPose=NavigateToPose, NavigateThroughPoses=NavigateThroughPoses)
    _module("robot_audio_interfaces")
    _module("robot_audio_interfaces.msg", AudioCommand=AudioCommand)
