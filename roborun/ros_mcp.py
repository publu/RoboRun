"""Direct ROS MCP server — zero-config robot connectivity.

Two transport layers, automatic fallback (both vendored in roborun.transport):
  1. DDS direct (CycloneDDS) — no rosbridge, no ROS install; pub/sub on the
     common message families, not just Twist
  2. Rosbridge WebSocket — full message introspection, service calls, actions

The MCP auto-detects which transports are available and uses the best one;
`get_capabilities` tells the agent exactly what this connection supports.
"""

from __future__ import annotations

import base64
import json
import os
import threading
import time
from typing import Any

# ── Transport layer ──────────────────────────────────────────────────────────

_rosbridge_client = None
_rosbridge_lock = threading.Lock()

_dds_available: bool | None = None
_rosbridge_available: bool | None = None

_discovery_cache: dict[str, Any] = {"nodes": [], "topics": [], "ts": 0}
_cache_lock = threading.Lock()
_CACHE_TTL = 10.0


def _check_dds() -> bool:
    global _dds_available
    if _dds_available is not None:
        return _dds_available
    try:
        from cyclonedds.domain import DomainParticipant  # noqa: F401
        _dds_available = True
    except ImportError:
        _dds_available = False
    return _dds_available


def _get_rosbridge(host: str | None = None, port: int = 9090):
    global _rosbridge_client
    with _rosbridge_lock:
        if _rosbridge_client and _rosbridge_client.is_connected:
            return _rosbridge_client
        if host:
            try:
                from roborun.rosbridge import RosbridgeClient
                _rosbridge_client = RosbridgeClient(host, port)
                _rosbridge_client.connect(timeout=3.0)
                return _rosbridge_client
            except Exception:
                _rosbridge_client = None
    return _rosbridge_client


def _get_robot_host() -> str | None:
    try:
        from roborun.server import load_profile
        profile = load_profile()
        return profile.get("robotIp") or None
    except Exception:
        return os.environ.get("ROBOT_IP")


_dds_transport = None
_dds_transport_lock = threading.Lock()


def _get_dds_transport():
    """Singleton DDS transport (roborun.transport) for general pub/sub."""
    global _dds_transport
    with _dds_transport_lock:
        if _dds_transport is None:
            from roborun.transport.dds import DDSTransport
            _dds_transport = DDSTransport(
                domain=int(os.environ.get("ROS_DOMAIN_ID", "0")))
        return _dds_transport


# ── Discovery (DDS + rosbridge merge) ────────────────────────────────────────

def _discover(domain_id: int = 0, timeout: float = 2.0) -> dict:
    with _cache_lock:
        if time.time() - _discovery_cache["ts"] < _CACHE_TTL and _discovery_cache["topics"]:
            return dict(_discovery_cache)

    topics_dds: list[dict] = []
    nodes_dds: list[dict] = []
    topics_rb: list[dict] = []

    if _check_dds():
        try:
            from ros_tap.discovery.auto import auto_discover
            from ros_tap.discovery.ros2_dds import discover_ros2_topics
            nodes = auto_discover(ros2_domain=domain_id, timeout=timeout)
            raw_topics = discover_ros2_topics(domain_id, timeout)
            nodes_dds = [
                {"name": n.name, "namespace": n.namespace, "ros": n.ros_version,
                 "topics": [{"name": t.name, "type": t.msg_type, "category": t.category}
                            for t in n.topics]}
                for n in nodes
            ]
            topics_dds = [
                {"name": t.name, "type": t.msg_type, "category": t.category}
                for t in raw_topics
            ]
        except Exception:
            pass

    rb = _get_rosbridge(_get_robot_host())
    if rb:
        try:
            raw = rb.list_topics(timeout=3.0)
            topics_rb = [{"name": t["topic"], "type": t["type"], "category": _classify(t["topic"])}
                         for t in raw]
        except Exception:
            pass

    seen = {}
    for t in topics_dds + topics_rb:
        seen.setdefault(t["name"], t)

    result = {
        "nodes": nodes_dds,
        "topics": list(seen.values()),
        "ts": time.time(),
        "transports": {
            "dds": _check_dds(),
            "rosbridge": rb is not None and rb.is_connected if rb else False,
        },
    }

    with _cache_lock:
        _discovery_cache.update(result)
    return result


def _classify(topic_name: str) -> str:
    categories = {
        "battery": "power", "joint": "actuators", "imu": "imu",
        "camera": "camera", "image": "camera", "scan": "lidar",
        "lidar": "lidar", "odom": "odometry", "cmd_vel": "command",
        "diagnostics": "diagnostics", "tf": "transforms",
        "mavros": "flight", "depth": "depth",
    }
    lower = topic_name.lower()
    for key, cat in categories.items():
        if key in lower:
            return cat
    return "other"


# ── DDS direct publish ───────────────────────────────────────────────────────

def _publish_twist_dds(topic: str, lx: float, ly: float, az: float,
                       domain_id: int = 0) -> dict:
    """Twist over DDS via the vendored transport (correct rt/ topic mangling)."""
    try:
        return _get_dds_transport().publish(topic, "geometry_msgs/Twist", {
            "linear": {"x": lx, "y": ly, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": az},
        })
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


# ── Tool implementations ─────────────────────────────────────────────────────

def _tool_scan_robots(args: dict) -> dict:
    domain_id = args.get("domain_id", int(os.environ.get("ROS_DOMAIN_ID", "0")))
    timeout = args.get("timeout", 2.0)
    result = _discover(domain_id, timeout)
    return {
        "ok": True,
        "robot_count": len(result["nodes"]),
        "topic_count": len(result["topics"]),
        "robots": result["nodes"],
        "topics": result["topics"],
        "transports": result["transports"],
    }


def _tool_connect(args: dict) -> dict:
    ip = args.get("ip", "").strip()
    port = int(args.get("port", 9090))
    if not ip:
        return {"ok": False, "error": "IP address required"}
    rb = _get_rosbridge(ip, port)
    connected = rb is not None and rb.is_connected
    dds = _check_dds()
    return {
        "ok": connected or dds,
        "rosbridge": connected,
        "dds": dds,
        "ip": ip,
        "port": port,
    }


def _tool_list_topics(args: dict) -> dict:
    domain_id = args.get("domain_id", int(os.environ.get("ROS_DOMAIN_ID", "0")))
    result = _discover(domain_id)
    topics = result["topics"]
    cat = args.get("category")
    if cat:
        topics = [t for t in topics if t["category"] == cat]
    return {"ok": True, "topics": topics, "count": len(topics)}


def _tool_get_topic_type(args: dict) -> dict:
    topic = args.get("topic", "")
    rb = _get_rosbridge(_get_robot_host())
    if rb:
        try:
            result = rb.call_service("/rosapi/topic_type", args={"topic": topic}, timeout=3.0)
            if result:
                return {"ok": True, "topic": topic, "type": result.get("type", "unknown")}
        except Exception:
            pass
    result = _discover()
    for t in result["topics"]:
        if t["name"] == topic:
            return {"ok": True, "topic": topic, "type": t["type"]}
    return {"ok": False, "error": f"Topic {topic} not found"}


def _tool_get_message_details(args: dict) -> dict:
    msg_type = args.get("type", "")
    rb = _get_rosbridge(_get_robot_host())
    if rb:
        try:
            result = rb.call_service("/rosapi/message_details",
                                     args={"type": msg_type}, timeout=3.0)
            if result:
                return {"ok": True, "type": msg_type, "details": result}
        except Exception:
            pass
    from roborun.transport.schemas import message_fields, SUPPORTED_TYPES
    fields = message_fields(msg_type)
    if fields:
        return {"ok": True, "type": msg_type, "details": fields,
                "source": "bundled schema"}
    return {"ok": True, "type": msg_type,
            "note": "Not a bundled type; full introspection requires rosbridge. "
                    "Use connect_to_robot first.",
            "bundled_types": list(SUPPORTED_TYPES)}


def _tool_subscribe_once(args: dict) -> dict:
    topic = args.get("topic", "")
    msg_type = args.get("type", "")
    timeout_s = args.get("timeout_s", 5.0)

    rb = _get_rosbridge(_get_robot_host())
    if rb:
        try:
            msg = rb.subscribe_once(topic, msg_type, timeout=timeout_s)
            if msg is not None:
                if _is_image_msg(msg):
                    return _format_image_result(topic, msg)
                return {"ok": True, "topic": topic, "message": msg}
            return {"ok": False, "error": f"No message on {topic} within {timeout_s}s"}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
    return {"ok": False, "error": "No rosbridge connection. Use connect_to_robot or install cyclonedds for DDS."}


def _tool_subscribe_duration(args: dict) -> dict:
    topic = args.get("topic", "")
    msg_type = args.get("type", "")
    duration = args.get("duration_s", 3.0)
    max_msgs = args.get("max_messages", 50)

    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge connection"}

    messages: list[dict] = []
    ev = threading.Event()

    def _cb(msg: dict) -> None:
        messages.append(msg)
        if len(messages) >= max_msgs:
            ev.set()

    try:
        rb.subscribe(topic, msg_type, _cb)
        ev.wait(duration)
        rb.unsubscribe(topic)
        return {"ok": True, "topic": topic, "count": len(messages), "messages": messages}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _tool_publish(args: dict) -> dict:
    topic = args.get("topic", "")
    msg_type = args.get("type", "")
    message = args.get("message", {})

    rb = _get_rosbridge(_get_robot_host())
    if rb:
        try:
            rb.publish(topic, msg_type, message)
            return {"ok": True, "topic": topic, "type": msg_type}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    if _check_dds():
        # General DDS publish: any bundled message family, not just Twist.
        try:
            return _get_dds_transport().publish(topic, msg_type, message)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
    return {"ok": False, "error": "No transport available. Connect to robot first."}


_MAX_LIN = float(os.environ.get("ROBORUN_MAX_LINEAR_VEL", "1.0"))
_MAX_ANG = float(os.environ.get("ROBORUN_MAX_ANGULAR_VEL", "1.5"))


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def _tool_move(args: dict) -> dict:
    lx = _clamp(float(args.get("linear_x", 0)), _MAX_LIN)
    ly = _clamp(float(args.get("linear_y", 0)), _MAX_LIN)
    az = _clamp(float(args.get("angular_z", 0)), _MAX_ANG)
    dur = args.get("duration_s")
    topic = args.get("topic", "/cmd_vel")
    domain_id = int(args.get("domain_id", os.environ.get("ROS_DOMAIN_ID", "0")))

    try:
        from roborun.routes._singletons import get_simulator
        sim = get_simulator()
        if sim.is_running:
            sim.set_cmd_vel(forward=lx, left=ly, turn=az)
            if dur:
                def _stop():
                    time.sleep(dur)
                    sim.set_cmd_vel(0, 0, 0)
                threading.Thread(target=_stop, daemon=True).start()
            return {"ok": True, "linear_x": lx, "linear_y": ly, "angular_z": az,
                    "transport": "mujoco", "will_stop_after_s": dur}
    except Exception:
        pass

    rb = _get_rosbridge(_get_robot_host())
    if rb:
        try:
            rb.move(lx, ly, az, topic)
            if dur:
                def _stop():
                    time.sleep(dur)
                    rb.stop(topic)
                threading.Thread(target=_stop, daemon=True).start()
            return {"ok": True, "linear_x": lx, "linear_y": ly, "angular_z": az,
                    "transport": "rosbridge", "will_stop_after_s": dur}
        except Exception:
            pass

    if _check_dds():
        result = _publish_twist_dds(topic, lx, ly, az, domain_id)
        if result["ok"] and dur:
            def _stop():
                time.sleep(dur)
                _publish_twist_dds(topic, 0, 0, 0, domain_id)
            threading.Thread(target=_stop, daemon=True).start()
            result["will_stop_after_s"] = dur
        result["transport"] = "dds"
        return result

    return {"ok": False, "error": "No transport available"}


def _tool_estop(args: dict) -> dict:
    topic = args.get("topic", "/cmd_vel")
    try:
        from roborun.routes._singletons import get_simulator
        sim = get_simulator()
        if sim.is_running:
            sim.set_cmd_vel(0, 0, 0)
            return {"ok": True, "transport": "mujoco"}
    except Exception:
        pass
    rb = _get_rosbridge(_get_robot_host())
    if rb:
        try:
            rb.stop(topic)
            return {"ok": True, "transport": "rosbridge"}
        except Exception:
            pass
    if _check_dds():
        result = _publish_twist_dds(topic, 0, 0, 0)
        result["transport"] = "dds"
        return result
    return {"ok": False, "error": "No transport available"}


def _tool_get_nodes(args: dict) -> dict:
    rb = _get_rosbridge(_get_robot_host())
    if rb:
        try:
            result = rb.call_service("/rosapi/nodes", args={}, timeout=3.0)
            if result:
                nodes = result.get("nodes", [])
                return {"ok": True, "nodes": nodes, "count": len(nodes)}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    disc = _discover()
    nodes = [n["name"] for n in disc["nodes"]]
    return {"ok": True, "nodes": nodes, "count": len(nodes), "source": "dds"}


def _tool_get_services(args: dict) -> dict:
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge. Services not available via DDS alone."}
    try:
        result = rb.call_service("/rosapi/services", args={}, timeout=3.0)
        if result:
            services = result.get("services", [])
            return {"ok": True, "services": services, "count": len(services)}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": False, "error": "Failed to list services"}


def _tool_get_service_type(args: dict) -> dict:
    service = args.get("service", "")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge"}
    try:
        result = rb.call_service("/rosapi/service_type",
                                 args={"service": service}, timeout=3.0)
        if result:
            return {"ok": True, "service": service, "type": result.get("type", "unknown")}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": False, "error": f"Service {service} not found"}


def _tool_call_service(args: dict) -> dict:
    service = args.get("service", "")
    srv_type = args.get("type", "")
    srv_args = args.get("args", {})
    timeout = args.get("timeout_s", 10.0)

    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge for service calls"}
    try:
        result = rb.call_service(service, srv_type, srv_args, timeout=timeout)
        return {"ok": True, "service": service, "result": result}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _tool_get_actions(args: dict) -> dict:
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge for action listing"}
    try:
        result = rb.call_service("/rosapi/action_servers", args={}, timeout=3.0)
        if result:
            actions = result.get("action_servers", result.get("servers", []))
            return {"ok": True, "actions": actions, "count": len(actions)}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "actions": [], "count": 0}


def _tool_send_action_goal(args: dict) -> dict:
    action = args.get("action", "")
    action_type = args.get("type", "")
    goal = args.get("goal", {})
    timeout = args.get("timeout_s", 30.0)

    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge for actions"}
    try:
        result = rb.send_action_goal(action, action_type, goal, timeout=timeout)
        return {"ok": True, "action": action, "result": result}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _tool_get_params(args: dict) -> dict:
    node = args.get("node", "")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge for parameters"}
    try:
        result = rb.call_service("/rosapi/get_param_names", args={}, timeout=3.0)
        if result:
            names = result.get("names", [])
            if node:
                names = [n for n in names if n.startswith(node)]
            return {"ok": True, "parameters": names, "count": len(names)}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "parameters": [], "count": 0}


def _tool_get_param(args: dict) -> dict:
    name = args.get("name", "")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge"}
    try:
        result = rb.call_service("/rosapi/get_param",
                                 args={"name": name}, timeout=3.0)
        if result:
            return {"ok": True, "name": name, "value": result.get("value")}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": False, "error": f"Parameter {name} not found"}


def _tool_set_param(args: dict) -> dict:
    name = args.get("name", "")
    value = args.get("value")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge"}
    try:
        rb.call_service("/rosapi/set_param",
                        args={"name": name, "value": json.dumps(value)}, timeout=3.0)
        return {"ok": True, "name": name, "value": value}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _tool_camera_snapshot(args: dict) -> dict:
    topic = args.get("topic", "/camera/image_raw/compressed")

    # Try sim frame first
    try:
        from pathlib import Path
        sim_frame = Path("/tmp/roborun_frame.jpg")
        from roborun.routes._singletons import get_simulator
        sim = get_simulator()
        if sim.is_running and sim_frame.exists():
            raw = sim_frame.read_bytes()
            b64 = base64.b64encode(raw).decode()
            return {"ok": True, "source": "mujoco",
                    "image": f"data:image/jpeg;base64,{b64}"}
    except Exception:
        pass

    # Try local webcam
    try:
        from roborun.webcam import WebcamProcessor
        wc = WebcamProcessor.get()
        frame = wc.snapshot()
        if frame is not None:
            import cv2
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            b64 = base64.b64encode(buf.tobytes()).decode()
            return {"ok": True, "source": "webcam",
                    "image": f"data:image/jpeg;base64,{b64}",
                    "width": frame.shape[1], "height": frame.shape[0]}
    except Exception:
        pass

    # Try rosbridge camera
    rb = _get_rosbridge(_get_robot_host())
    if rb:
        try:
            raw = rb.camera_snapshot(topic, timeout=5.0)
            if raw:
                b64 = base64.b64encode(raw).decode()
                return {"ok": True, "source": "rosbridge",
                        "image": f"data:image/jpeg;base64,{b64}"}
        except Exception:
            pass

    return {"ok": False, "error": "No camera available. Start webcam or connect to robot."}


def _tool_navigate(args: dict) -> dict:
    x, y = float(args["x"]), float(args["y"])
    yaw = float(args.get("yaw", 0))
    frame = args.get("frame", "map")

    rb = _get_rosbridge(_get_robot_host())
    if rb:
        import math
        goal_msg = {
            "header": {"frame_id": frame},
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {
                    "x": 0.0, "y": 0.0,
                    "z": math.sin(yaw / 2),
                    "w": math.cos(yaw / 2),
                },
            },
        }
        try:
            rb.publish("/goal_pose", "geometry_msgs/PoseStamped", goal_msg)
            return {"ok": True, "goal": {"x": x, "y": y, "yaw": yaw, "frame": frame},
                    "method": "goal_pose"}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    return {"ok": False, "error": "Navigation requires rosbridge connection"}


def _tool_get_robot_info(args: dict) -> dict:
    try:
        from roborun.routes._singletons import get_simulator
        sim = get_simulator()
        if sim.is_running:
            state = sim.get_state()
            return {
                "ok": True,
                "type": state.get("robot_type", "simulated"),
                "label": f"MuJoCo: {state.get('robot', 'unknown')}",
                "transport": "mujoco",
                "sim_time": state.get("sim_time", 0),
                "position": state.get("position"),
                "has_policy": state.get("has_policy", False),
                "fps": state.get("fps", 0),
            }
    except Exception:
        pass
    from roborun.robot_types import detect_type, get_profile
    disc = _discover()
    topic_names = [t["name"] for t in disc["topics"]]
    robot_type = detect_type(ros_topics=topic_names)
    profile = get_profile(robot_type)
    rb = _get_rosbridge(_get_robot_host())
    return {
        "ok": True,
        "type": profile["type"],
        "label": profile["label"],
        "available_skills": profile["skills"],
        "ros_topics": profile["ros_topics"],
        "control_scheme": profile["control_scheme"],
        "telemetry_channels": profile["telemetry_channels"],
        "transports": {
            "dds": _check_dds(),
            "rosbridge": rb is not None and rb.is_connected if rb else False,
        },
        "discovered_topics": len(disc["topics"]),
    }


_active_tap = None
_tap_lock = threading.Lock()


def _tool_telemetry_stream(args: dict) -> dict:
    """Tap mode: passively record topics into the sealed MCAP run (spec §1.2.4).

    The tap runs at full topic rate with no LLM in the loop; agent events land
    in the same run via the event bus. Stop seals + anchors the recording.
    """
    global _active_tap
    from roborun import recorder as rec_mod

    action = args["action"]
    if action == "status":
        with _tap_lock:
            rec = rec_mod.active_recorder()
            return {"ok": True,
                    "recording": rec.status() if rec else None,
                    "tap": _active_tap.status() if _active_tap else None}

    if action == "start":
        topics = args.get("topics") or None
        robot_id = args.get("robot_id", "local")
        transport = None
        rb = _get_rosbridge(_get_robot_host())
        if rb:
            from roborun.transport.bridge import RosbridgeTransport
            transport = RosbridgeTransport(client=rb)
        elif _check_dds():
            transport = _get_dds_transport()
        if transport is None:
            return {"ok": False, "error": "no transport: connect to a robot "
                                          "or install cyclonedds for DDS direct"}
        from roborun.transport.tap import Tap
        with _tap_lock:
            rec = rec_mod.start_recording(robot_id=robot_id)
            _active_tap = Tap(transport, rec, topics=topics)
            status = _active_tap.start()
        return {"ok": True, "run": rec.run_id, "mcap": str(rec.mcap_path),
                **status}

    if action == "stop":
        with _tap_lock:
            if _active_tap is not None:
                _active_tap.stop()
                _active_tap = None
            seal = rec_mod.stop_recording(do_anchor=not args.get("no_anchor", False))
        if seal is None:
            return {"ok": True, "stopped": True, "note": "nothing was recording"}
        extracted = None
        try:
            from roborun.observations import extract_run
            from roborun.routes._singletons import get_memory
            mcap_path = rec_mod.runs_root() / seal["robot_id"] / f"{seal['run']}.mcap"
            extracted = extract_run(mcap_path, get_memory(), robot_id=seal["robot_id"])
        except Exception:
            pass
        return {"ok": True, "stopped": True, "seal": seal, "indexed": extracted}

    return {"ok": False, "error": f"Unknown action: {action}"}


def _tool_get_capabilities(args: dict) -> dict:
    """The capability matrix, so the agent adapts instead of failing silently."""
    from roborun.transport import CAPABILITY_MATRIX
    rb = _get_rosbridge(_get_robot_host())
    available = {
        "dds": _check_dds(),
        "rosbridge": bool(rb and rb.is_connected),
        "native": False,
    }
    try:
        import rclpy  # noqa: F401
        available["native"] = True
    except ImportError:
        pass
    active = next((k for k in ("native", "rosbridge", "dds") if available[k]), None)
    return {"ok": True, "available": available, "active": active,
            "matrix": CAPABILITY_MATRIX,
            "note": "services/actions/params need rosbridge or native; "
                    "DDS direct covers discovery + pub/sub on common types"}


# ── New introspection tools (Phase 4c — ros-mcp-server parity) ─────────────────

def _tool_get_topic_details(args: dict) -> dict:
    topic = args.get("topic", "")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge for topic details"}
    try:
        pubs = rb.call_service("/rosapi/publishers", args={"topic": topic}, timeout=3.0)
        subs = rb.call_service("/rosapi/subscribers", args={"topic": topic}, timeout=3.0)
        ttype = rb.call_service("/rosapi/topic_type", args={"topic": topic}, timeout=3.0)
        return {
            "ok": True, "topic": topic,
            "type": ttype.get("type", "unknown") if ttype else "unknown",
            "publishers": pubs.get("publishers", []) if pubs else [],
            "subscribers": subs.get("subscribers", []) if subs else [],
        }
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _tool_get_node_details(args: dict) -> dict:
    node = args.get("node", "")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        disc = _discover()
        for n in disc["nodes"]:
            if n["name"] == node:
                return {"ok": True, "node": node, "topics": n.get("topics", []),
                        "source": "dds"}
        return {"ok": False, "error": "Node not found via DDS"}
    try:
        details = rb.call_service("/rosapi/node_details", args={"node": node}, timeout=3.0)
        if details:
            return {"ok": True, "node": node,
                    "publishing": details.get("publishing", []),
                    "subscribing": details.get("subscribing", []),
                    "services": details.get("services", [])}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": False, "error": f"Node {node} not found"}


def _tool_get_service_details(args: dict) -> dict:
    service = args.get("service", "")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge"}
    try:
        stype = rb.call_service("/rosapi/service_type", args={"service": service}, timeout=3.0)
        if stype:
            type_name = stype.get("type", "")
            req_details = rb.call_service("/rosapi/service_request_details",
                                          args={"type": type_name}, timeout=3.0)
            resp_details = rb.call_service("/rosapi/service_response_details",
                                           args={"type": type_name}, timeout=3.0)
            return {"ok": True, "service": service, "type": type_name,
                    "request": req_details, "response": resp_details}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": False, "error": f"Service {service} not found"}


def _tool_publish_for_duration(args: dict) -> dict:
    topic = args.get("topic", "")
    msg_type = args.get("type", "")
    message = args.get("message", {})
    duration = float(args.get("duration_s", 1.0))
    rate = float(args.get("rate_hz", 10.0))

    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge connection"}

    if "Twist" in msg_type and "cmd_vel" in topic:
        lin = message.get("linear", {})
        ang = message.get("angular", {})
        message["linear"]["x"] = _clamp(float(lin.get("x", 0)), _MAX_LIN)
        message["linear"]["y"] = _clamp(float(lin.get("y", 0)), _MAX_LIN)
        message["angular"]["z"] = _clamp(float(ang.get("z", 0)), _MAX_ANG)

    count = 0
    interval = 1.0 / rate
    end_time = time.time() + min(duration, 10.0)
    try:
        while time.time() < end_time:
            rb.publish(topic, msg_type, message)
            count += 1
            time.sleep(interval)
        return {"ok": True, "topic": topic, "messages_sent": count,
                "duration_s": duration, "rate_hz": rate}
    except Exception as exc:
        return {"ok": False, "error": str(exc), "messages_sent": count}


def _tool_cancel_action(args: dict) -> dict:
    action = args.get("action", "")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge"}
    try:
        cancel_topic = f"{action}/cancel"
        rb.publish(cancel_topic, "actionlib_msgs/GoalID", {})
        return {"ok": True, "action": action, "cancelled": True}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _tool_has_parameter(args: dict) -> dict:
    name = args.get("name", "")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge"}
    try:
        result = rb.call_service("/rosapi/has_param", args={"name": name}, timeout=3.0)
        return {"ok": True, "name": name, "exists": result.get("exists", False) if result else False}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _tool_delete_parameter(args: dict) -> dict:
    name = args.get("name", "")
    rb = _get_rosbridge(_get_robot_host())
    if not rb:
        return {"ok": False, "error": "Requires rosbridge"}
    try:
        rb.call_service("/rosapi/delete_param", args={"name": name}, timeout=3.0)
        return {"ok": True, "name": name, "deleted": True}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


# ── Image helpers ─────────────────────────────────────────────────────────────

def _is_image_msg(msg: dict) -> bool:
    return ("data" in msg and
            ("encoding" in msg or "format" in msg) and
            ("width" in msg or "height" in msg))


def _format_image_result(topic: str, msg: dict) -> dict:
    data = msg.get("data", "")
    if isinstance(data, str):
        return {"ok": True, "topic": topic, "image": f"data:image/jpeg;base64,{data}",
                "width": msg.get("width"), "height": msg.get("height")}
    return {"ok": True, "topic": topic, "message": msg}


# ── MCP tool definitions ─────────────────────────────────────────────────────

MCP_TOOLS = [
    # --- Discovery & connection ---
    {
        "name": "scan_robots",
        "description": "Discover all ROS robots on the network automatically. Uses DDS multicast (no ROS install needed) and/or rosbridge. Returns nodes, topics, message types, and which transports are available.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "domain_id": {"type": "integer", "description": "ROS 2 domain ID (default 0)"},
                "timeout": {"type": "number", "description": "Discovery timeout seconds (default 2)"},
            },
        },
    },
    {
        "name": "connect_to_robot",
        "description": "Connect to a robot's rosbridge server for full access (services, actions, parameters). DDS works without this, but rosbridge enables richer introspection.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "ip": {"type": "string", "description": "Robot IP address"},
                "port": {"type": "integer", "description": "Rosbridge port (default 9090)"},
            },
            "required": ["ip"],
        },
    },
    {
        "name": "get_robot_info",
        "description": "Get the detected robot type, available skills, control scheme, expected ROS topics, and transport status. Automatically classifies the robot (quadruped, drone, humanoid, arm) from discovered topics.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    # --- Topics ---
    {
        "name": "list_topics",
        "description": "List all ROS topics with message types and categories (power, actuators, camera, lidar, odometry, imu, command, etc).",
        "inputSchema": {
            "type": "object",
            "properties": {
                "category": {"type": "string", "description": "Filter by category"},
            },
        },
    },
    {
        "name": "get_topic_type",
        "description": "Get the message type for a specific topic.",
        "inputSchema": {
            "type": "object",
            "properties": {"topic": {"type": "string"}},
            "required": ["topic"],
        },
    },
    {
        "name": "get_message_details",
        "description": "Get the full field definitions for a ROS message type, so you know exactly what to publish. Example: get_message_details('geometry_msgs/Twist') returns all fields and their types.",
        "inputSchema": {
            "type": "object",
            "properties": {"type": {"type": "string", "description": "Message type e.g. geometry_msgs/Twist"}},
            "required": ["type"],
        },
    },
    {
        "name": "subscribe_once",
        "description": "Read one message from a ROS topic. Automatically handles image topics (returns JPEG). Good for checking sensor values, camera frames, battery state.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "Topic name e.g. /battery_state"},
                "type": {"type": "string", "description": "Message type (optional, auto-detected)"},
                "timeout_s": {"type": "number", "description": "Timeout seconds (default 5)"},
            },
            "required": ["topic"],
        },
    },
    {
        "name": "subscribe_for_duration",
        "description": "Subscribe to a topic for N seconds and collect all messages. Useful for sampling sensor data over time.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string"},
                "type": {"type": "string"},
                "duration_s": {"type": "number", "description": "How long to listen (default 3)"},
                "max_messages": {"type": "integer", "description": "Max messages to collect (default 50)"},
            },
            "required": ["topic"],
        },
    },
    {
        "name": "publish",
        "description": "Publish a message to any ROS topic. Works via rosbridge (any message type) or DDS direct (Twist on /cmd_vel).",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string"},
                "type": {"type": "string", "description": "ROS message type"},
                "message": {"type": "object", "description": "Message data as JSON"},
            },
            "required": ["topic", "type", "message"],
        },
    },
    # --- Movement ---
    {
        "name": "move",
        "description": "Send a velocity command to the robot. Shortcut for publishing to /cmd_vel. Works via DDS direct or rosbridge.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "linear_x": {"type": "number", "description": "Forward/back m/s"},
                "linear_y": {"type": "number", "description": "Strafe m/s (holonomic)"},
                "angular_z": {"type": "number", "description": "Turn rate rad/s"},
                "duration_s": {"type": "number", "description": "Hold for N seconds, then stop"},
                "topic": {"type": "string", "description": "Velocity topic (default /cmd_vel)"},
            },
        },
    },
    {
        "name": "estop",
        "description": "Emergency stop — immediately sends zero velocity. Bypasses all queues.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "Velocity topic (default /cmd_vel)"},
            },
        },
    },
    {
        "name": "navigate",
        "description": "Send a navigation goal. The robot will autonomously navigate to the target position using Nav2.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "Goal X meters"},
                "y": {"type": "number", "description": "Goal Y meters"},
                "yaw": {"type": "number", "description": "Goal orientation radians (default 0)"},
                "frame": {"type": "string", "description": "Reference frame (default 'map')"},
            },
            "required": ["x", "y"],
        },
    },
    # --- Services ---
    {
        "name": "get_services",
        "description": "List all available ROS services.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_service_type",
        "description": "Get the type of a specific ROS service.",
        "inputSchema": {
            "type": "object",
            "properties": {"service": {"type": "string"}},
            "required": ["service"],
        },
    },
    {
        "name": "call_service",
        "description": "Call a ROS service with arguments and get the response. Use get_services + get_service_type to discover what's available.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "service": {"type": "string", "description": "Service name e.g. /set_bool"},
                "type": {"type": "string", "description": "Service type (optional)"},
                "args": {"type": "object", "description": "Service arguments"},
                "timeout_s": {"type": "number", "description": "Timeout (default 10)"},
            },
            "required": ["service"],
        },
    },
    # --- Actions ---
    {
        "name": "get_actions",
        "description": "List all available ROS 2 action servers.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "send_action_goal",
        "description": "Send a goal to a ROS 2 action server (e.g. Nav2 navigate, manipulation). Waits for result.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {"type": "string", "description": "Action server name"},
                "type": {"type": "string", "description": "Action type"},
                "goal": {"type": "object", "description": "Goal message"},
                "timeout_s": {"type": "number", "description": "Timeout (default 30)"},
            },
            "required": ["action", "type", "goal"],
        },
    },
    # --- Parameters ---
    {
        "name": "get_parameters",
        "description": "List all ROS parameters, optionally filtered by node name.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "node": {"type": "string", "description": "Filter by node name prefix"},
            },
        },
    },
    {
        "name": "get_parameter",
        "description": "Read a specific ROS parameter value.",
        "inputSchema": {
            "type": "object",
            "properties": {"name": {"type": "string"}},
            "required": ["name"],
        },
    },
    {
        "name": "set_parameter",
        "description": "Set a ROS parameter value.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "name": {"type": "string"},
                "value": {"description": "Parameter value (any JSON type)"},
            },
            "required": ["name", "value"],
        },
    },
    # --- Nodes ---
    {
        "name": "get_nodes",
        "description": "List all active ROS nodes.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    # --- Perception ---
    {
        "name": "camera_snapshot",
        "description": "Capture a camera frame. Tries local webcam first, then robot camera via rosbridge.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "Camera topic (default /camera/image_raw/compressed)"},
            },
        },
    },
    # --- Telemetry ---
    {
        "name": "telemetry_stream",
        "description": "Tap mode: passively record robot topics into a tamper-evident MCAP run at full rate (no LLM in the loop). 'start' opens a run and taps topics; 'stop' seals it (Merkle root + RFC 3161 trusted timestamp) and indexes it for search; 'status' reports counts.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "status"]},
                "topics": {"type": "array", "items": {"type": "string"},
                           "description": "Topic names or patterns ('/camera/*'). Omit to tap everything visible."},
                "robot_id": {"type": "string", "description": "Robot identity for the run directory (default: local)"},
                "no_anchor": {"type": "boolean", "description": "Skip trusted-timestamp anchoring on stop"},
            },
            "required": ["action"],
        },
    },
    {
        "name": "get_capabilities",
        "description": "What the current transport supports right now (pub/sub/services/actions/params/types per backend: DDS direct, rosbridge, native rclpy). Call this before using services or actions so you degrade gracefully instead of erroring.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    # --- Detailed introspection (Phase 4c) ---
    {
        "name": "get_topic_details",
        "description": "Get detailed info about a topic: publishers, subscribers, and message type.",
        "inputSchema": {
            "type": "object",
            "properties": {"topic": {"type": "string"}},
            "required": ["topic"],
        },
    },
    {
        "name": "get_node_details",
        "description": "Get detailed info about a node: what it publishes, subscribes to, and services it provides.",
        "inputSchema": {
            "type": "object",
            "properties": {"node": {"type": "string"}},
            "required": ["node"],
        },
    },
    {
        "name": "get_service_details",
        "description": "Get the request/response field definitions for a service type.",
        "inputSchema": {
            "type": "object",
            "properties": {"service": {"type": "string"}},
            "required": ["service"],
        },
    },
    # --- Extended publish/action/param (Phase 4d) ---
    {
        "name": "publish_for_duration",
        "description": "Publish a message repeatedly at a given rate for N seconds. Useful for holding velocity commands or sending periodic data.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "topic": {"type": "string"},
                "type": {"type": "string", "description": "ROS message type"},
                "message": {"type": "object", "description": "Message data"},
                "duration_s": {"type": "number", "description": "How long to publish (default 1, max 10)"},
                "rate_hz": {"type": "number", "description": "Publish rate (default 10)"},
            },
            "required": ["topic", "type", "message"],
        },
    },
    {
        "name": "cancel_action_goal",
        "description": "Cancel an in-flight action goal.",
        "inputSchema": {
            "type": "object",
            "properties": {"action": {"type": "string", "description": "Action server name"}},
            "required": ["action"],
        },
    },
    {
        "name": "has_parameter",
        "description": "Check if a ROS parameter exists without throwing an error.",
        "inputSchema": {
            "type": "object",
            "properties": {"name": {"type": "string"}},
            "required": ["name"],
        },
    },
    {
        "name": "delete_parameter",
        "description": "Delete a ROS parameter.",
        "inputSchema": {
            "type": "object",
            "properties": {"name": {"type": "string"}},
            "required": ["name"],
        },
    },
]

_TOOL_HANDLERS = {
    "scan_robots": _tool_scan_robots,
    "connect_to_robot": _tool_connect,
    "get_robot_info": _tool_get_robot_info,
    "list_topics": _tool_list_topics,
    "get_topic_type": _tool_get_topic_type,
    "get_message_details": _tool_get_message_details,
    "subscribe_once": _tool_subscribe_once,
    "subscribe_for_duration": _tool_subscribe_duration,
    "publish": _tool_publish,
    "move": _tool_move,
    "estop": _tool_estop,
    "navigate": _tool_navigate,
    "get_services": _tool_get_services,
    "get_service_type": _tool_get_service_type,
    "call_service": _tool_call_service,
    "get_actions": _tool_get_actions,
    "send_action_goal": _tool_send_action_goal,
    "get_parameters": _tool_get_params,
    "get_parameter": _tool_get_param,
    "set_parameter": _tool_set_param,
    "get_nodes": _tool_get_nodes,
    "camera_snapshot": _tool_camera_snapshot,
    "telemetry_stream": _tool_telemetry_stream,
    "get_capabilities": _tool_get_capabilities,
    "get_topic_details": _tool_get_topic_details,
    "get_node_details": _tool_get_node_details,
    "get_service_details": _tool_get_service_details,
    "publish_for_duration": _tool_publish_for_duration,
    "cancel_action_goal": _tool_cancel_action,
    "has_parameter": _tool_has_parameter,
    "delete_parameter": _tool_delete_parameter,
}


def handle_tool_call(name: str, args: dict) -> dict:
    handler = _TOOL_HANDLERS.get(name)
    if handler:
        try:
            return handler(args)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    # Fall through to skills registry
    try:
        from roborun.skills import get_registry
        result = get_registry().handle_tool_call(name, args)
        if result is not None:
            return result
    except Exception:
        pass

    return {"ok": False, "error": f"Unknown tool: {name}"}


def get_all_tools() -> list[dict]:
    """All MCP tools: built-in + skills."""
    tools = list(MCP_TOOLS)
    try:
        from roborun.skills import get_registry
        tools.extend(get_registry().get_mcp_tools())
    except Exception:
        pass
    return tools


def get_mcp_manifest() -> dict:
    return {
        "name": "ros-agent",
        "version": "0.11.0",
        "description": "Direct ROS robot control — DDS + rosbridge, zero-config discovery, full topic/service/action/param access, extensible skills. No ROS install needed.",
        "tools": get_all_tools(),
    }
