"""Native rclpy backend — full capability when running on-robot in a ROS env.

Thin by design (spec phases it as "later"): present so the interface is
complete and on-robot deployments get everything without a bridge. Raises
ImportError at construction when rclpy is absent, which get_transport("auto")
treats as "try the next backend".
"""
from __future__ import annotations

import threading
from typing import Callable

from roborun.transport import Transport


def _import_msg(msg_type: str):
    """'geometry_msgs/Twist' or 'geometry_msgs/msg/Twist' → class via rosidl."""
    import importlib
    parts = msg_type.split("/")
    pkg, name = parts[0], parts[-1]
    return getattr(importlib.import_module(f"{pkg}.msg"), name)


class NativeTransport(Transport):
    name = "native"

    def __init__(self, node_name: str = "roborun", **_ignored) -> None:
        import rclpy
        from rclpy.node import Node
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node: "Node" = rclpy.create_node(node_name)
        self._subs: dict[str, object] = {}
        self._pubs: dict[str, object] = {}
        self._spin = threading.Thread(target=self._spin_loop, daemon=True,
                                      name="NativeSpin")
        self._spin.start()

    def _spin_loop(self) -> None:
        while self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=0.1)

    def topics(self) -> dict[str, str]:
        out: dict[str, str] = {}
        for name, types in self._node.get_topic_names_and_types():
            if types:
                out[name] = types[0].replace("/msg/", "/")
        return out

    def subscribe(self, topic: str, callback: Callable[[dict], None],
                  msg_type: str | None = None) -> str:
        from rosidl_runtime_py.convert import message_to_ordereddict
        msg_type = msg_type or self.type_of(topic)
        if not msg_type:
            raise ValueError(f"unknown type for {topic}")
        cls = _import_msg(msg_type)
        sub = self._node.create_subscription(
            cls, topic, lambda m: callback(dict(message_to_ordereddict(m))), 10)
        self._subs[topic] = sub
        return topic

    def unsubscribe(self, topic: str) -> None:
        sub = self._subs.pop(topic, None)
        if sub is not None:
            self._node.destroy_subscription(sub)

    def publish(self, topic: str, msg_type: str, msg: dict) -> dict:
        try:
            from rosidl_runtime_py.set_message import set_message_fields
            cls = _import_msg(msg_type)
            pub = self._pubs.get(topic)
            if pub is None:
                pub = self._node.create_publisher(cls, topic, 10)
                self._pubs[topic] = pub
            m = cls()
            set_message_fields(m, msg)
            pub.publish(m)
            return {"ok": True, "topic": topic, "type": msg_type, "transport": "native"}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def call_service(self, service: str, srv_type: str = "",
                     args: dict | None = None, timeout: float = 10.0) -> dict:
        try:
            import importlib
            from rosidl_runtime_py.set_message import set_message_fields
            from rosidl_runtime_py.convert import message_to_ordereddict
            pkg, name = srv_type.split("/")[0], srv_type.split("/")[-1]
            cls = getattr(importlib.import_module(f"{pkg}.srv"), name)
            client = self._node.create_client(cls, service)
            if not client.wait_for_service(timeout_sec=timeout):
                return {"ok": False, "error": f"service {service} unavailable"}
            req = cls.Request()
            set_message_fields(req, args or {})
            future = client.call_async(req)
            self._rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
            if future.result() is None:
                return {"ok": False, "error": "service call timed out"}
            return {"ok": True, "result": dict(message_to_ordereddict(future.result()))}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def close(self) -> None:
        try:
            self._node.destroy_node()
        except Exception:
            pass
