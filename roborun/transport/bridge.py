"""rosbridge backend — full capability surface when a bridge runs on the robot.

Wraps roborun.rosbridge.RosbridgeClient behind the Transport interface:
services, actions, params, and arbitrary message types all work because the
bridge does the (de)serialization server-side. Needs `ros2 launch
rosbridge_server rosbridge_websocket_launch.xml` (or ROS 1 equivalent) on
the robot — that requirement is exactly why DDS direct also exists.
"""
from __future__ import annotations

from typing import Any, Callable

from roborun.rosbridge import RosbridgeClient
from roborun.transport import Transport


class RosbridgeTransport(Transport):
    name = "rosbridge"

    def __init__(self, host: str = "127.0.0.1", port: int = 9090,
                 client: RosbridgeClient | None = None, **_ignored) -> None:
        if client is not None:
            self._client = client
        else:
            self._client = RosbridgeClient(host, port)
            self._client.connect()
        self.host = host

    def topics(self) -> dict[str, str]:
        return {t["topic"]: t["type"] for t in self._client.list_topics()}

    def subscribe(self, topic: str, callback: Callable[[dict], None],
                  msg_type: str | None = None) -> str:
        return self._client.subscribe(topic, msg_type or "", callback)

    def unsubscribe(self, topic: str) -> None:
        self._client.unsubscribe(topic)

    def publish(self, topic: str, msg_type: str, msg: dict) -> dict:
        try:
            self._client.publish(topic, msg_type, msg)
            return {"ok": True, "topic": topic, "type": msg_type,
                    "transport": "rosbridge"}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def call_service(self, service: str, srv_type: str = "",
                     args: dict | None = None, timeout: float = 10.0) -> dict:
        result = self._client.call_service(service, srv_type, args, timeout)
        return {"ok": result is not None, "result": result}

    def send_goal(self, action: str, action_type: str, goal: dict,
                  timeout: float = 30.0) -> dict:
        result = self._client.send_action_goal(action, action_type, goal, timeout)
        return {"ok": result is not None, "result": result}

    def get_param(self, node: str, parameter: str) -> dict:
        value = self._client.get_param(node, parameter)
        return {"ok": True, "value": value}

    def capabilities(self) -> dict[str, Any]:
        caps = super().capabilities()
        caps["connected"] = self._client.is_connected
        caps["health"] = self._client.health
        return caps

    def close(self) -> None:
        self._client.disconnect()
