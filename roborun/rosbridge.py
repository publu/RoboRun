"""RosBridge WebSocket client — native ROS 2 transport for any robot.

Connects to a rosbridge_server (ws://robot_ip:9090) and exposes the same
tool surface as agenticROS: topic pub/sub, services, actions, and param
get/set. This lets RoboRun control any ROS 2 robot over the network.

Usage:
    client = RosbridgeClient("192.168.1.100")
    client.connect()
    topics = client.list_topics()
    client.publish("/cmd_vel", "geometry_msgs/Twist", {"linear": {"x": 0.5}})
    msg = client.subscribe_once("/scan", timeout=5.0)
    result = client.call_service("/set_bool", "std_srvs/SetBool", {"data": True})
    client.disconnect()
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from typing import Any, Callable


import logging

_log = logging.getLogger(__name__)


class RosbridgeClient:
    DEFAULT_PORT = 9090

    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT,
                 auto_reconnect: bool = True) -> None:
        self._host = host
        self._port = port
        self._ws = None
        self._lock = threading.Lock()
        self._pending: dict[str, dict] = {}
        self._subscribers: dict[str, list[Callable]] = {}
        self._connected = False
        self._recv_thread: threading.Thread | None = None
        self._health_thread: threading.Thread | None = None
        self._auto_reconnect = auto_reconnect
        self._reconnect_delay = 1.0
        self._max_reconnect_delay = 30.0
        self._disconnect_count = 0

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self, timeout: float = 5.0) -> None:
        import websocket
        url = f"ws://{self._host}:{self._port}"
        self._ws = websocket.WebSocket()
        self._ws.connect(url, timeout=timeout)
        self._connected = True
        self._reconnect_delay = 1.0
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        if self._auto_reconnect and self._health_thread is None:
            self._health_thread = threading.Thread(target=self._health_loop, daemon=True)
            self._health_thread.start()
        _log.info("Connected to rosbridge at %s:%d", self._host, self._port)

    def disconnect(self) -> None:
        self._auto_reconnect = False
        self._connected = False
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None

    def _reconnect(self) -> bool:
        try:
            import websocket
            url = f"ws://{self._host}:{self._port}"
            ws = websocket.WebSocket()
            ws.connect(url, timeout=3.0)
            self._ws = ws
            self._connected = True
            self._reconnect_delay = 1.0
            self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._recv_thread.start()
            _log.info("Reconnected to rosbridge at %s:%d", self._host, self._port)
            return True
        except Exception:
            self._reconnect_delay = min(self._reconnect_delay * 2, self._max_reconnect_delay)
            return False

    def _health_loop(self) -> None:
        while self._auto_reconnect:
            time.sleep(5.0)
            if not self._connected and self._auto_reconnect:
                self._disconnect_count += 1
                _log.warning("Rosbridge disconnected (count=%d), retrying in %.1fs",
                             self._disconnect_count, self._reconnect_delay)
                time.sleep(self._reconnect_delay)
                self._reconnect()

    @property
    def is_connected(self) -> bool:
        return self._connected and self._ws is not None

    @property
    def health(self) -> dict:
        return {
            "connected": self.is_connected,
            "host": self._host,
            "port": self._port,
            "disconnect_count": self._disconnect_count,
            "auto_reconnect": self._auto_reconnect,
        }

    def _send(self, msg: dict) -> None:
        if not self._ws:
            raise RuntimeError("Not connected to rosbridge")
        with self._lock:
            self._ws.send(json.dumps(msg))

    def _recv_loop(self) -> None:
        while self._connected and self._ws:
            try:
                raw = self._ws.recv()
                if not raw:
                    break
                msg = json.loads(raw)
            except Exception:
                break
            self._dispatch(msg)
        self._connected = False

    def _dispatch(self, msg: dict) -> None:
        op = msg.get("op")
        if op == "publish":
            topic = msg.get("topic", "")
            for cb in self._subscribers.get(topic, []):
                try:
                    cb(msg.get("msg", {}))
                except Exception:
                    pass
        elif op in ("service_response", "call_result", "action_result"):
            uid = msg.get("id", "")
            if uid in self._pending:
                self._pending[uid]["result"] = msg
                self._pending[uid]["event"].set()

    # ── Topic operations ──────────────────────────────────────────────────────

    def list_topics(self, timeout: float = 5.0) -> list[dict[str, str]]:
        uid = str(uuid.uuid4())
        event = threading.Event()
        self._pending[uid] = {"event": event, "result": None}
        self._send({"op": "call_service", "id": uid,
                    "service": "/rosapi/topics", "args": {}})
        event.wait(timeout)
        raw = self._pending.pop(uid, {}).get("result")
        if not raw:
            return []
        values = raw.get("values", {})
        topics = values.get("topics", [])
        types = values.get("types", [])
        return [{"topic": t, "type": tp} for t, tp in zip(topics, types)]

    def publish(self, topic: str, msg_type: str, message: dict) -> None:
        self._send({"op": "publish", "topic": topic,
                    "type": msg_type, "msg": message})

    def subscribe(self, topic: str, msg_type: str, callback: Callable,
                  throttle_rate: int = 0) -> str:
        sub_id = f"sub_{uuid.uuid4().hex[:8]}"
        self._subscribers.setdefault(topic, []).append(callback)
        self._send({"op": "subscribe", "id": sub_id, "topic": topic,
                    "type": msg_type, "throttle_rate": throttle_rate})
        return sub_id

    def unsubscribe(self, topic: str) -> None:
        self._send({"op": "unsubscribe", "topic": topic})
        self._subscribers.pop(topic, None)

    def subscribe_once(self, topic: str, msg_type: str = "",
                       timeout: float = 5.0) -> dict | None:
        result: list[dict] = []
        ev = threading.Event()

        def _cb(msg: dict) -> None:
            result.append(msg)
            ev.set()

        sub_id = self.subscribe(topic, msg_type, _cb)
        ev.wait(timeout)
        self.unsubscribe(topic)
        return result[0] if result else None

    # ── Services ──────────────────────────────────────────────────────────────

    def call_service(self, service: str, srv_type: str = "",
                     args: dict | None = None, timeout: float = 10.0) -> dict | None:
        uid = str(uuid.uuid4())
        event = threading.Event()
        self._pending[uid] = {"event": event, "result": None}
        payload: dict[str, Any] = {"op": "call_service", "id": uid,
                                    "service": service, "args": args or {}}
        if srv_type:
            payload["type"] = srv_type
        self._send(payload)
        event.wait(timeout)
        raw = self._pending.pop(uid, {}).get("result")
        return raw.get("values") if raw else None

    # ── Actions ───────────────────────────────────────────────────────────────

    def send_action_goal(self, action: str, action_type: str,
                         goal: dict, timeout: float = 30.0) -> dict | None:
        uid = str(uuid.uuid4())
        event = threading.Event()
        self._pending[uid] = {"event": event, "result": None}
        self._send({"op": "send_action_goal", "id": uid,
                    "action": action, "action_type": action_type, "goal": goal})
        event.wait(timeout)
        raw = self._pending.pop(uid, {}).get("result")
        return raw.get("values") if raw else None

    # ── Params ────────────────────────────────────────────────────────────────

    def get_param(self, node: str, parameter: str, timeout: float = 5.0) -> Any:
        result = self.call_service(
            "/rosapi/get_param",
            args={"name": f"{node}/{parameter}"},
            timeout=timeout,
        )
        return result.get("value") if result else None

    def set_param(self, node: str, parameter: str, value: Any,
                  timeout: float = 5.0) -> bool:
        result = self.call_service(
            "/rosapi/set_param",
            args={"name": f"{node}/{parameter}", "value": json.dumps(value)},
            timeout=timeout,
        )
        return bool(result)

    # ── Camera helpers ────────────────────────────────────────────────────────

    def camera_snapshot(self, topic: str = "/camera/image_raw/compressed",
                        timeout: float = 10.0) -> bytes | None:
        """Read a single frame from a CompressedImage topic. Returns raw JPEG bytes."""
        msg = self.subscribe_once(topic, timeout=timeout)
        if not msg:
            raw_topic = topic.replace("/compressed", "").replace("/raw", "")
            msg = self.subscribe_once(raw_topic, timeout=timeout)
        if not msg:
            return None
        import base64
        data = msg.get("data", "")
        if isinstance(data, str):
            try:
                return base64.b64decode(data)
            except Exception:
                return None
        return bytes(data) if data else None

    def depth_distance(self, topic: str = "/camera/depth/image_raw",
                       timeout: float = 5.0) -> float | None:
        """Sample center pixel of a depth image. Returns distance in meters."""
        msg = self.subscribe_once(topic, timeout=timeout)
        if not msg:
            return None
        data = msg.get("data", [])
        width = msg.get("width", 0)
        height = msg.get("height", 0)
        if not data or not width or not height:
            return None
        import struct
        center_idx = (height // 2) * width + (width // 2)
        encoding = msg.get("encoding", "32FC1")
        try:
            if encoding == "32FC1":
                offset = center_idx * 4
                if offset + 4 > len(data):
                    return None
                (val,) = struct.unpack_from("<f", bytes(data), offset)
                return float(val) if val == val else None  # nan check
            elif encoding == "16UC1":
                offset = center_idx * 2
                if offset + 2 > len(data):
                    return None
                (val,) = struct.unpack_from("<H", bytes(data), offset)
                return val / 1000.0  # mm -> m
        except Exception:
            return None

    # ── Velocity control ──────────────────────────────────────────────────────

    def move(self, linear_x: float = 0.0, linear_y: float = 0.0,
             angular_z: float = 0.0,
             topic: str = "/cmd_vel", *, linear_z: float = 0.0) -> None:
        self.publish(topic, "geometry_msgs/Twist", {
            "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        })

    def stop(self, topic: str = "/cmd_vel") -> None:
        self.move(0.0, 0.0, 0.0, topic)


# ── Singleton management ──────────────────────────────────────────────────────

_client: RosbridgeClient | None = None
_client_lock = threading.Lock()


def get_client(host: str | None = None, port: int = RosbridgeClient.DEFAULT_PORT,
               auto_connect: bool = True) -> RosbridgeClient | None:
    global _client
    with _client_lock:
        if _client is not None and not _client.is_connected:
            _client = None
        if _client is None and host and auto_connect:
            try:
                _client = RosbridgeClient(host, port)
                _client.connect()
            except Exception:
                _client = None
        return _client


def reset_client() -> None:
    global _client
    with _client_lock:
        if _client:
            _client.disconnect()
        _client = None
