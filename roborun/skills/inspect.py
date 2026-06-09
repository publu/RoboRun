"""Inspect skill — high-level robot introspection that synthesizes raw data
into actionable summaries. These are the tools that make LLMs productive.

robot_brief: One-call overview of the entire robot state.
watch_topic: Monitor a topic until a condition is met.
diff_state: Compare robot state over time to detect changes.
"""
from __future__ import annotations

import json
import logging
import time

SKILL_ID = "inspect"
SKILL_NAME = "Inspect"
SKILL_VERSION = "1.0.0"

log = logging.getLogger(__name__)


def _call(name: str, args: dict = {}) -> dict:
    from roborun.ros_mcp import handle_tool_call
    return handle_tool_call(name, args)


def _robot_brief(args: dict) -> dict:
    brief = {"ok": True, "ts": time.time()}

    discovery = _call("list_topics")
    topics = discovery.get("topics", [])
    brief["topics_total"] = len(topics)

    categories = {
        "camera": [], "lidar": [], "imu": [], "odom": [],
        "cmd_vel": [], "battery": [], "joint": [], "tf": [], "other": [],
    }
    for t in topics:
        name = t if isinstance(t, str) else t.get("name", "")
        low = name.lower()
        if any(k in low for k in ("image", "camera", "video", "rgb")):
            categories["camera"].append(name)
        elif any(k in low for k in ("scan", "lidar", "pointcloud", "point_cloud")):
            categories["lidar"].append(name)
        elif "imu" in low:
            categories["imu"].append(name)
        elif "odom" in low:
            categories["odom"].append(name)
        elif "cmd_vel" in low or "twist" in low:
            categories["cmd_vel"].append(name)
        elif "battery" in low or "power" in low:
            categories["battery"].append(name)
        elif "joint" in low:
            categories["joint"].append(name)
        elif low.startswith("/tf"):
            categories["tf"].append(name)
        else:
            categories["other"].append(name)

    brief["categories"] = {k: v for k, v in categories.items() if v}

    capabilities = []
    if categories["camera"]:
        capabilities.append("vision")
    if categories["lidar"]:
        capabilities.append("lidar")
    if categories["imu"]:
        capabilities.append("IMU")
    if categories["odom"]:
        capabilities.append("odometry")
    if categories["cmd_vel"]:
        capabilities.append("movement")
    if categories["joint"]:
        capabilities.append("joints/arms")
    brief["capabilities"] = capabilities

    nodes = _call("get_nodes")
    brief["nodes"] = nodes.get("nodes", [])
    brief["nodes_total"] = len(brief["nodes"])

    try:
        from roborun.skills import get_registry
        reg = get_registry()
        brief["skills"] = list(reg.skills.keys())
        brief["tools_total"] = len(reg.tools)
        brief["behaviors"] = list(reg.behaviors.keys())
    except Exception:
        brief["skills"] = []

    lines = []
    lines.append(f"Connected robot: {brief['nodes_total']} nodes, {brief['topics_total']} topics")
    if capabilities:
        lines.append(f"Capabilities: {', '.join(capabilities)}")
    if categories.get("camera"):
        lines.append(f"Camera topics: {', '.join(categories['camera'][:3])}")
    if categories.get("cmd_vel"):
        lines.append("Movement: ready (cmd_vel available)")
    if brief.get("skills"):
        lines.append(f"Skills loaded: {', '.join(brief['skills'])}")
    brief["summary"] = "\n".join(lines)

    return brief


def _watch_topic(args: dict) -> dict:
    topic = args.get("topic", "")
    if not topic:
        return {"ok": False, "error": "topic required"}

    field = args.get("field", "")
    condition = args.get("condition", "exists")
    threshold = args.get("threshold")
    timeout_s = min(float(args.get("timeout_s", 30)), 120)
    poll_hz = min(float(args.get("poll_hz", 2)), 10)

    start = time.time()
    poll_interval = 1.0 / poll_hz
    samples = 0

    while (time.time() - start) < timeout_s:
        result = _call("subscribe_once", {"topic": topic, "timeout_ms": int(poll_interval * 800)})
        msg = result.get("message")
        if msg is None:
            time.sleep(poll_interval)
            continue

        samples += 1
        value = msg
        if field:
            for part in field.split("."):
                if isinstance(value, dict):
                    value = value.get(part)
                elif isinstance(value, list) and part.isdigit():
                    value = value[int(part)] if int(part) < len(value) else None
                else:
                    value = None
                    break

        triggered = False
        if condition == "exists":
            triggered = value is not None
        elif condition == "equals" and threshold is not None:
            triggered = str(value) == str(threshold)
        elif condition == "gt" and threshold is not None:
            try:
                triggered = float(value) > float(threshold)
            except (TypeError, ValueError):
                pass
        elif condition == "lt" and threshold is not None:
            try:
                triggered = float(value) < float(threshold)
            except (TypeError, ValueError):
                pass
        elif condition == "contains" and threshold is not None:
            triggered = str(threshold) in json.dumps(value, default=str)
        elif condition == "changed":
            if samples > 1:
                triggered = True

        if triggered:
            return {
                "ok": True, "triggered": True,
                "condition": condition, "field": field,
                "value": value, "samples": samples,
                "elapsed_s": round(time.time() - start, 2),
            }

        time.sleep(poll_interval)

    return {
        "ok": True, "triggered": False,
        "reason": "timeout", "timeout_s": timeout_s,
        "samples": samples,
    }


def _diff_state(args: dict) -> dict:
    delay_s = min(float(args.get("delay_s", 5)), 60)

    snap1 = _call("list_topics")
    topics1 = set(t if isinstance(t, str) else t.get("name", "") for t in snap1.get("topics", []))
    nodes1_result = _call("get_nodes")
    nodes1 = set(nodes1_result.get("nodes", []))

    time.sleep(delay_s)

    snap2 = _call("list_topics")
    topics2 = set(t if isinstance(t, str) else t.get("name", "") for t in snap2.get("topics", []))
    nodes2_result = _call("get_nodes")
    nodes2 = set(nodes2_result.get("nodes", []))

    return {
        "ok": True,
        "delay_s": delay_s,
        "topics_added": sorted(topics2 - topics1),
        "topics_removed": sorted(topics1 - topics2),
        "nodes_added": sorted(nodes2 - nodes1),
        "nodes_removed": sorted(nodes1 - nodes2),
        "changed": bool((topics2 - topics1) or (topics1 - topics2) or
                        (nodes2 - nodes1) or (nodes1 - nodes2)),
    }


def register(registry) -> None:
    registry.add_tool(
        name="robot_brief",
        description=(
            "Get a complete overview of the connected robot in one call. "
            "Returns categorized topics, capabilities, loaded skills, "
            "and a human-readable summary. Best first tool to call."
        ),
        input_schema={"type": "object", "properties": {}},
        handler=_robot_brief,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="watch_topic",
        description=(
            "Monitor a topic until a condition is met. Conditions: "
            "'exists' (any message), 'gt'/'lt' (numeric threshold), "
            "'equals', 'contains', 'changed'. Returns when triggered or on timeout."
        ),
        input_schema={
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "ROS topic to watch"},
                "field": {"type": "string", "description": "Dot-path to field in message (e.g. 'data' or 'pose.position.x')"},
                "condition": {
                    "type": "string",
                    "enum": ["exists", "gt", "lt", "equals", "contains", "changed"],
                    "description": "Trigger condition (default: exists)",
                },
                "threshold": {"description": "Value to compare against for gt/lt/equals/contains"},
                "timeout_s": {"type": "number", "description": "Max wait time in seconds (default 30, max 120)"},
                "poll_hz": {"type": "number", "description": "Polling rate in Hz (default 2, max 10)"},
            },
            "required": ["topic"],
        },
        handler=_watch_topic,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="diff_state",
        description=(
            "Compare robot state over time. Takes two snapshots of the ROS graph "
            "separated by delay_s seconds and reports what changed — "
            "new/removed topics and nodes. Useful for debugging launches."
        ),
        input_schema={
            "type": "object",
            "properties": {
                "delay_s": {"type": "number", "description": "Seconds between snapshots (default 5, max 60)"},
            },
        },
        handler=_diff_state,
        skill_id=SKILL_ID,
    )
