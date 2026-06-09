"""ROS bridge routes — connect, topics, publish, subscribe, services, params, move."""
from __future__ import annotations

from roborun.routes import get, post, send_json, ApiError
from roborun.routes.dashboard import load_profile


def _get_ros_client(host: str | None = None):
    host = host or load_profile().get("robotIp", "")
    if not host:
        raise ApiError(400, "No robot IP configured")
    from roborun.rosbridge import get_client
    client = get_client(host)
    if not client:
        raise ApiError(503, "Not connected to rosbridge")
    return client


@get("/api/ros/topics")
def topics(h):
    from urllib.parse import parse_qs, urlparse
    qs = parse_qs(urlparse(h.path).query)
    host = qs.get("host", [None])[0] or load_profile().get("robotIp", "")
    try:
        client = _get_ros_client(host)
        topics = client.list_topics()
        send_json(h, 200, {"ok": True, "topics": topics, "count": len(topics)})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@get("/api/ros/status(?:\\?.*)?")
def status(h):
    from urllib.parse import parse_qs, urlparse
    qs = parse_qs(urlparse(h.path).query)
    host = qs.get("host", [None])[0] or load_profile().get("robotIp", "")
    try:
        from roborun.rosbridge import get_client
        client = get_client(host) if host else None
        connected = client is not None and client.is_connected
        send_json(h, 200, {"ok": True, "connected": connected, "host": host or None})
    except Exception as exc:
        send_json(h, 200, {"ok": True, "connected": False, "error": str(exc)})


@post("/api/ros/connect")
def connect(h, payload):
    host = str(payload.get("host", "")).strip() or load_profile().get("robotIp", "")
    if not host:
        raise ApiError(400, "host required")
    port = int(payload.get("port", 9090))
    try:
        from roborun.rosbridge import reset_client, get_client
        reset_client()
        client = get_client(host, port)
        if not client:
            send_json(h, 503, {"ok": False, "error": "Connection failed"})
        else:
            from roborun.ros_telemetry import get_bridge
            bridge = get_bridge()
            bridge.stop()
            bridge.start(host)
            send_json(h, 200, {"ok": True, "host": host, "port": port})
    except Exception as exc:
        send_json(h, 503, {"ok": False, "error": str(exc)})


@post("/api/ros/disconnect")
def disconnect(h, payload):
    from roborun.rosbridge import reset_client
    reset_client()
    send_json(h, 200, {"ok": True})


@post("/api/ros/publish")
def publish(h, payload):
    topic = str(payload.get("topic", "")).strip()
    msg_type = str(payload.get("type", "")).strip()
    message = payload.get("message", {})
    if not topic:
        raise ApiError(400, "topic required")
    try:
        client = _get_ros_client()
        client.publish(topic, msg_type, message)
        send_json(h, 200, {"ok": True})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@post("/api/ros/subscribe-once")
def subscribe_once(h, payload):
    topic = str(payload.get("topic", "")).strip()
    msg_type = str(payload.get("type", "")).strip()
    timeout = float(payload.get("timeout", 5000)) / 1000.0
    if not topic:
        raise ApiError(400, "topic required")
    try:
        client = _get_ros_client()
        msg = client.subscribe_once(topic, msg_type, timeout=timeout)
        send_json(h, 200, {"ok": True, "message": msg})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@post("/api/ros/service")
def service(h, payload):
    service_name = str(payload.get("service", "")).strip()
    srv_type = str(payload.get("type", "")).strip()
    args = payload.get("args", {})
    timeout = float(payload.get("timeout", 10000)) / 1000.0
    if not service_name:
        raise ApiError(400, "service required")
    try:
        client = _get_ros_client()
        result = client.call_service(service_name, srv_type, args, timeout=timeout)
        send_json(h, 200, {"ok": True, "result": result})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@post("/api/ros/action")
def action(h, payload):
    action_name = str(payload.get("action", "")).strip()
    action_type = str(payload.get("actionType", "")).strip()
    goal = payload.get("goal", {})
    timeout = float(payload.get("timeout", 30000)) / 1000.0
    if not action_name or not action_type:
        raise ApiError(400, "action and actionType required")
    try:
        client = _get_ros_client()
        result = client.send_action_goal(action_name, action_type, goal, timeout=timeout)
        send_json(h, 200, {"ok": True, "result": result})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@post("/api/ros/param/get")
def param_get(h, payload):
    node = str(payload.get("node", "")).strip()
    parameter = str(payload.get("parameter", "")).strip()
    if not node or not parameter:
        raise ApiError(400, "node and parameter required")
    try:
        client = _get_ros_client()
        value = client.get_param(node, parameter)
        send_json(h, 200, {"ok": True, "value": value})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@post("/api/ros/param/set")
def param_set(h, payload):
    node = str(payload.get("node", "")).strip()
    parameter = str(payload.get("parameter", "")).strip()
    value = payload.get("value")
    if not node or not parameter:
        raise ApiError(400, "node and parameter required")
    try:
        client = _get_ros_client()
        ok = client.set_param(node, parameter, value)
        send_json(h, 200, {"ok": ok})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@post("/api/ros/camera")
def ros_camera(h, payload):
    import base64
    topic = str(payload.get("topic", "/camera/image_raw/compressed")).strip()
    timeout = float(payload.get("timeout", 10000)) / 1000.0
    try:
        client = _get_ros_client()
        frame_bytes = client.camera_snapshot(topic, timeout=timeout)
        if not frame_bytes:
            send_json(h, 404, {"ok": False, "error": "No frame received"})
            return
        b64 = base64.b64encode(frame_bytes).decode()
        send_json(h, 200, {"ok": True, "image": f"data:image/jpeg;base64,{b64}"})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@post("/api/ros/depth")
def ros_depth(h, payload):
    topic = str(payload.get("topic", "/camera/depth/image_raw")).strip()
    timeout = float(payload.get("timeout", 5000)) / 1000.0
    try:
        client = _get_ros_client()
        dist = client.depth_distance(topic, timeout=timeout)
        send_json(h, 200, {"ok": True, "distance_m": dist})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@post("/api/ros/move")
def ros_move(h, payload):
    linear_x = float(payload.get("linear_x", 0.0))
    linear_y = float(payload.get("linear_y", 0.0))
    angular_z = float(payload.get("angular_z", 0.0))
    topic = str(payload.get("topic", "/cmd_vel"))
    try:
        client = _get_ros_client()
        client.move(linear_x, linear_y, angular_z, topic)
        send_json(h, 200, {"ok": True})
    except ApiError:
        raise
    except Exception as exc:
        send_json(h, 500, {"ok": False, "error": str(exc)})


@get("/api/ros/health")
def ros_health(h):
    from roborun.rosbridge import get_client
    from roborun.ros_mcp import _check_dds
    client = get_client(auto_connect=False)
    health = client.health if client else {"connected": False}
    health["dds_available"] = _check_dds()
    health["ok"] = health.get("connected", False) or health["dds_available"]
    send_json(h, 200, health)
