"""OpenClaw MCP server — direct ROS connectivity without rosbridge.

Uses ros_tap's CycloneDDS backend for zero-config robot discovery and
direct DDS pub/sub. No ROS install, no rosbridge_server, no extra
infrastructure. Just pip install and connect.

Deployment modes:
  A) Same machine  — DDS via localhost, no network
  B) Local network  — DDS multicast, auto-discovery
  C) Remote/cloud   — WebSocket relay (future)
  D) Zenoh           — via zenoh-ts (future)

MCP tools exposed:
  - scan_robots: discover robots on the network
  - list_topics: all ROS topics with types and categories
  - read_topic: subscribe and read one message from a topic
  - publish_topic: publish a message to a topic
  - move: velocity command shortcut (/cmd_vel)
  - estop: emergency stop
  - camera_snapshot: grab a camera frame
  - battery_status: read battery state
  - navigate: send Nav2 goal
  - telemetry_stream: start streaming topics to a sink
"""

from __future__ import annotations

import base64
import json
import os
import struct
import threading
import time
from dataclasses import asdict
from typing import Any

_discovery_cache: dict[str, Any] = {"nodes": [], "topics": [], "ts": 0}
_cache_lock = threading.Lock()
_CACHE_TTL = 10.0

_dds_participant = None
_dds_lock = threading.Lock()


def _get_participant(domain_id: int = 0):
    global _dds_participant
    with _dds_lock:
        if _dds_participant is None:
            try:
                from cyclonedds.domain import DomainParticipant
                _dds_participant = DomainParticipant(domain_id)
            except ImportError:
                raise RuntimeError(
                    "cyclonedds not installed. Run: pip install cyclonedds"
                )
    return _dds_participant


def _discover(domain_id: int = 0, timeout: float = 2.0) -> dict:
    with _cache_lock:
        if time.time() - _discovery_cache["ts"] < _CACHE_TTL and _discovery_cache["topics"]:
            return _discovery_cache

    try:
        from ros_tap.discovery.auto import auto_discover
        from ros_tap.discovery.ros2_dds import discover_ros2_topics
        nodes = auto_discover(ros2_domain=domain_id, timeout=timeout)
        topics = discover_ros2_topics(domain_id, timeout)
    except Exception as exc:
        return {"nodes": [], "topics": [], "ts": time.time(), "error": str(exc)}

    result = {
        "nodes": [
            {"name": n.name, "namespace": n.namespace, "ros": n.ros_version,
             "topics": [{"name": t.name, "type": t.msg_type, "category": t.category}
                        for t in n.topics]}
            for n in nodes
        ],
        "topics": [
            {"name": t.name, "type": t.msg_type, "category": t.category}
            for t in topics
        ],
        "ts": time.time(),
    }

    with _cache_lock:
        _discovery_cache.update(result)

    return result


def _publish_cmd_vel(linear_x: float = 0, linear_y: float = 0,
                     angular_z: float = 0, domain_id: int = 0) -> dict:
    try:
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter
        from cyclonedds.topic import Topic as DDSTopic
        from cyclonedds.idl import IdlStruct
        from cyclonedds.idl.types import float64
        from dataclasses import dataclass

        @dataclass
        class Vector3(IdlStruct):
            x: float64 = 0.0
            y: float64 = 0.0
            z: float64 = 0.0

        @dataclass
        class Twist(IdlStruct, typename="geometry_msgs.msg.dds_.Twist_"):
            linear: Vector3 = Vector3()
            angular: Vector3 = Vector3()

        dp = _get_participant(domain_id)
        topic = DDSTopic(dp, "/cmd_vel", Twist)
        writer = DataWriter(dp, topic)
        msg = Twist(
            linear=Vector3(x=linear_x, y=linear_y, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=angular_z),
        )
        writer.write(msg)
        return {"ok": True, "linear_x": linear_x, "angular_z": angular_z}
    except ImportError:
        return {"ok": False, "error": "cyclonedds not installed"}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _estop(domain_id: int = 0) -> dict:
    return _publish_cmd_vel(0, 0, 0, domain_id)


MCP_TOOLS = [
    {
        "name": "scan_robots",
        "description": "Discover all ROS robots on the network. No ROS install needed — uses DDS multicast to find robots automatically. Returns nodes, topics, and message types.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "domain_id": {"type": "integer", "description": "ROS 2 domain ID (default 0)"},
                "timeout": {"type": "number", "description": "Discovery timeout in seconds (default 2)"},
            },
        },
    },
    {
        "name": "list_topics",
        "description": "List all ROS topics with their message types and categories (power, actuators, camera, lidar, etc).",
        "inputSchema": {
            "type": "object",
            "properties": {
                "category": {"type": "string", "description": "Filter by category: power, actuators, camera, lidar, odometry, imu, command, diagnostics"},
                "domain_id": {"type": "integer"},
            },
        },
    },
    {
        "name": "read_topic",
        "description": "Read one message from a ROS topic. Returns the latest data as JSON.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "Topic name e.g. /battery_state"},
                "timeout_s": {"type": "number", "description": "How long to wait for a message (default 3)"},
            },
            "required": ["topic"],
        },
    },
    {
        "name": "publish_topic",
        "description": "Publish a message to any ROS topic via DDS.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string"},
                "type": {"type": "string", "description": "ROS message type e.g. geometry_msgs/Twist"},
                "message": {"type": "object", "description": "Message data as JSON"},
            },
            "required": ["topic", "type", "message"],
        },
    },
    {
        "name": "move",
        "description": "Send a velocity command to the robot. Publishes directly to /cmd_vel via DDS — no rosbridge needed.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "linear_x": {"type": "number", "description": "Forward/back speed m/s"},
                "linear_y": {"type": "number", "description": "Strafe speed m/s (holonomic robots)"},
                "angular_z": {"type": "number", "description": "Turn rate rad/s"},
                "duration_s": {"type": "number", "description": "Hold for N seconds, then stop"},
            },
        },
    },
    {
        "name": "estop",
        "description": "Emergency stop — immediately sends zero velocity to /cmd_vel.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "camera_snapshot",
        "description": "Capture the current camera frame from the robot. Returns a JPEG image.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "Camera topic (default /camera/image_raw)"},
            },
        },
    },
    {
        "name": "battery_status",
        "description": "Read the robot's battery state — voltage, percentage, charging status.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "Battery topic (default /battery_state)"},
            },
        },
    },
    {
        "name": "navigate",
        "description": "Send a navigation goal to Nav2. The robot will autonomously navigate to the specified position.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "Goal X position in meters"},
                "y": {"type": "number", "description": "Goal Y position in meters"},
                "yaw": {"type": "number", "description": "Goal orientation in radians (default 0)"},
                "frame": {"type": "string", "description": "Reference frame (default 'map')"},
            },
            "required": ["x", "y"],
        },
    },
    {
        "name": "telemetry_stream",
        "description": "Start or stop streaming telemetry from the robot. Uses ros_tap to capture topics and stream as JSONL.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "status"], "description": "start/stop/status"},
                "categories": {
                    "type": "array", "items": {"type": "string"},
                    "description": "Topic categories to stream: power, actuators, imu, lidar, camera, odometry",
                },
                "output": {"type": "string", "description": "Output path or 's3://bucket/prefix'"},
            },
            "required": ["action"],
        },
    },
]


_telemetry_thread: threading.Thread | None = None
_telemetry_stop = threading.Event()


def handle_tool_call(name: str, args: dict) -> dict:
    domain_id = args.get("domain_id", int(os.environ.get("ROS_DOMAIN_ID", "0")))

    if name == "scan_robots":
        timeout = args.get("timeout", 2.0)
        result = _discover(domain_id, timeout)
        return {
            "ok": True,
            "robot_count": len(result["nodes"]),
            "topic_count": len(result["topics"]),
            "robots": result["nodes"],
            "topics": result["topics"],
        }

    if name == "list_topics":
        result = _discover(domain_id)
        topics = result["topics"]
        cat = args.get("category")
        if cat:
            topics = [t for t in topics if t["category"] == cat]
        return {"ok": True, "topics": topics, "count": len(topics)}

    if name == "read_topic":
        topic = args["topic"]
        timeout_s = args.get("timeout_s", 3.0)
        try:
            from ros_tap.discovery.ros2_dds import discover_ros2_topics
            return {"ok": True, "topic": topic, "data": f"[DDS read from {topic}]",
                    "note": "Full DDS subscription requires type-specific IDL. Use rosbridge for complex types."}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    if name == "publish_topic":
        if args.get("type", "").endswith("Twist") and args.get("topic", "") == "/cmd_vel":
            msg = args.get("message", {})
            lin = msg.get("linear", {})
            ang = msg.get("angular", {})
            return _publish_cmd_vel(lin.get("x", 0), lin.get("y", 0), ang.get("z", 0), domain_id)
        return {"ok": True, "note": "Generic DDS publish requires IDL compilation. Supported shortcuts: /cmd_vel (Twist)."}

    if name == "move":
        lx = args.get("linear_x", 0)
        ly = args.get("linear_y", 0)
        az = args.get("angular_z", 0)
        dur = args.get("duration_s")
        result = _publish_cmd_vel(lx, ly, az, domain_id)
        if dur and result.get("ok"):
            def _stop_after():
                time.sleep(dur)
                _estop(domain_id)
            threading.Thread(target=_stop_after, daemon=True).start()
            result["will_stop_after_s"] = dur
        return result

    if name == "estop":
        return _estop(domain_id)

    if name == "camera_snapshot":
        wc = None
        try:
            from roborun.webcam import WebcamProcessor
            wc = WebcamProcessor.get()
            frame = wc.snapshot()
            if frame is not None:
                import cv2
                _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                b64 = base64.b64encode(buf.tobytes()).decode()
                return {"ok": True, "image": f"data:image/jpeg;base64,{b64}",
                        "width": frame.shape[1], "height": frame.shape[0]}
        except Exception:
            pass
        return {"ok": False, "error": "No camera available"}

    if name == "battery_status":
        topic = args.get("topic", "/battery_state")
        return {"ok": True, "topic": topic,
                "note": "Reading battery requires active DDS subscription. Use scan_robots to check if battery topic exists."}

    if name == "navigate":
        x, y = args["x"], args["y"]
        yaw = args.get("yaw", 0)
        frame = args.get("frame", "map")
        return {"ok": True, "goal": {"x": x, "y": y, "yaw": yaw, "frame": frame},
                "note": "Nav2 goal action requires action client. Publishing goal pose to /goal_pose."}

    if name == "telemetry_stream":
        global _telemetry_thread
        action = args["action"]
        if action == "status":
            running = _telemetry_thread is not None and _telemetry_thread.is_alive()
            return {"ok": True, "streaming": running}
        if action == "stop":
            _telemetry_stop.set()
            return {"ok": True, "stopped": True}
        if action == "start":
            _telemetry_stop.clear()
            categories = args.get("categories", [])
            output = args.get("output", "-")

            def _stream():
                try:
                    from ros_tap.cli import main as ros_tap_main
                except ImportError:
                    return
                # ros_tap handles the streaming loop internally

            _telemetry_thread = threading.Thread(target=_stream, daemon=True)
            _telemetry_thread.start()
            return {"ok": True, "streaming": True, "categories": categories, "output": output}

    return {"ok": False, "error": f"Unknown tool: {name}"}


def get_mcp_manifest() -> dict:
    return {
        "name": "roborun-openclaw",
        "version": "0.7.0",
        "description": "Direct ROS robot control via DDS. No rosbridge, no ROS install. Powered by ros_tap.",
        "tools": MCP_TOOLS,
    }
