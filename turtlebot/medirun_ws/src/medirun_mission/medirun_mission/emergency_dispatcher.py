from __future__ import annotations

import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .json_topic import dump_json_message


class EmergencyDispatcher(Node):
    def __init__(self) -> None:
        super().__init__("emergency_dispatcher")
        self.declare_parameter("battery_reject_percent", 30.0)
        self.declare_parameter("call_cooldown_sec", 2.0)

        self._ready = True
        self._payload_loaded = True
        self._battery_percent = 100.0
        self._last_dispatch_at = 0.0

        self._dispatch_pub = self.create_publisher(String, "/medirun/mission/dispatch_request", 10)
        self._reject_pub = self.create_publisher(String, "/medirun/mission/dispatch_rejection", 10)

        self.create_subscription(String, "/medirun/sensor/sos_button", self._handle_button, 10)
        self.create_subscription(Bool, "/medirun/sensor/payload_loaded", self._handle_payload, 10)
        self.create_subscription(String, "/medirun/system/readiness", self._handle_readiness, 10)
        self.create_subscription(String, "/medirun/system/battery", self._handle_battery, 10)

    def _handle_payload(self, msg: Bool) -> None:
        self._payload_loaded = msg.data

    def _handle_readiness(self, msg: String) -> None:
        self._ready = "READY" in msg.data

    def _handle_battery(self, msg: String) -> None:
        # 파싱에 실패해도 100%로 간주하지 않고 마지막 정상 값을 유지한다.
        # (이전에는 100.0으로 덮어써, 배터리를 모르는 채로 호출을 받아들였다.)
        try:
            self._battery_percent = float(msg.data)
        except ValueError:
            self.get_logger().warning(
                "잘못된 배터리 값 %r 무시, 마지막 값 %.1f%% 유지" % (msg.data, self._battery_percent)
            )

    def _handle_button(self, msg: String) -> None:
        now = time.monotonic()
        if now - self._last_dispatch_at < float(self.get_parameter("call_cooldown_sec").value):
            self._reject("COOLDOWN", msg.data)
            return
        if not self._payload_loaded:
            self._reject("PAYLOAD_MISSING", msg.data)
            return
        if self._battery_percent < float(self.get_parameter("battery_reject_percent").value):
            self._reject("LOW_BATTERY", msg.data)
            return
        if not self._ready:
            self._reject("SYSTEM_NOT_READY", msg.data)
            return

        self._last_dispatch_at = now
        mission_id = str(uuid.uuid4())
        out = String()
        out.data = dump_json_message(
            {
                "mission_id": mission_id,
                "button_id": msg.data,
                "requested_at_sec": now,
                "request_source": "sos_button",
            }
        )
        self._dispatch_pub.publish(out)
        self.get_logger().info("dispatch requested: mission_id=%s button_id=%s" % (mission_id, msg.data))

    def _reject(self, reason: str, button_id: str) -> None:
        msg = String()
        msg.data = dump_json_message({"reason": reason, "button_id": button_id})
        self._reject_pub.publish(msg)
        self.get_logger().warning("dispatch rejected: %s button_id=%s" % (reason, button_id))


def main() -> None:
    rclpy.init()
    node = EmergencyDispatcher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
