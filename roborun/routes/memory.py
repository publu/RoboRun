"""Spatial memory, timeline, depth, telemetry, trajectory routes."""
from __future__ import annotations

import json
from urllib.parse import parse_qs, urlparse

from roborun.routes import get, post, send_json
from roborun.routes._singletons import get_memory, get_webcam, get_simulator
from roborun.routes.tasks import log_event


@get("/api/memory/stats")
def stats(h):
    send_json(h, 200, {"ok": True, **get_memory().stats()})


@get("/api/memory/list(?:\\?.*)?")
def list_memories(h):
    qs = parse_qs(urlparse(h.path).query)
    robot = qs.get("robot_id", [None])[0]
    limit = int(qs.get("limit", ["50"])[0])
    send_json(h, 200, {"ok": True, "memories": get_memory().list_memories(limit=limit, robot_id=robot)})


@get("/api/memory/thumb/(?P<mid>.+)")
def thumbnail(h, mid):
    data = get_memory().get_thumbnail(mid)
    if data:
        h.send_response(200)
        h.send_header("Content-Type", "image/jpeg")
        h.send_header("Content-Length", str(len(data)))
        h.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        h.end_headers()
        h.wfile.write(data)
    else:
        send_json(h, 404, {"ok": False, "error": "not found"})


@post("/api/memory/store")
def store(h, payload):
    mem = get_memory()
    wc = get_webcam()
    frame = wc.snapshot()
    embedding = None
    dets = None
    if frame is not None:
        if wc._clip:
            try:
                embedding = wc._clip.embed_image(frame)
            except Exception:
                pass
        dets = wc.get_detections()
    mid = mem.store(
        frame=frame, embedding=embedding, detections=dets,
        x=payload.get("x"), y=payload.get("y"), z=payload.get("z"),
        robot_id=payload.get("robot_id", "local"),
        metadata=payload.get("metadata"),
    )
    log_event("memory_stored", f"Memory {mid}")
    send_json(h, 200, {"ok": True, "id": mid})


@post("/api/memory/search")
def search(h, payload):
    mem = get_memory()
    query = payload.get("query", "")
    mode = payload.get("mode", "clip")
    top_k = int(payload.get("top_k", 10))
    robot_id = payload.get("robot_id")
    if mode == "clip" and query:
        wc = get_webcam()
        if wc._clip is None:
            from roborun.models import CLIPMatcher
            wc._clip = CLIPMatcher()
        emb = wc._clip.embed_text(query)
        results = mem.search_clip(emb, top_k=top_k, robot_id=robot_id)
    elif mode == "nearby":
        results = mem.search_nearby(
            x=float(payload.get("x", 0)), y=float(payload.get("y", 0)),
            z=float(payload.get("z")) if payload.get("z") is not None else None,
            radius=float(payload.get("radius", 2.0)), top_k=top_k, robot_id=robot_id,
        )
    elif mode == "yolo" and query:
        results = mem.search_yolo(query, top_k=top_k, robot_id=robot_id)
    else:
        results = mem.list_memories(limit=top_k, robot_id=robot_id)
    send_json(h, 200, {"ok": True, "results": results})


@post("/api/memory/delete")
def delete(h, payload):
    ok = get_memory().delete(payload.get("id", ""))
    send_json(h, 200, {"ok": ok})


# ── Timeline ─────────────────────────────────────────────────────────────────

@get("/api/timeline(?:\\?.*)?")
def timeline(h):
    qs = parse_qs(urlparse(h.path).query)
    limit = min(int(qs.get("limit", ["30"])[0]), 100)
    since = float(qs.get("since", ["0"])[0])
    memories = get_memory().list_memories(
        limit=limit, source="timeline",
        since=since if since > 0 else None,
    )
    send_json(h, 200, {"ok": True, "entries": memories, "count": len(memories)})


@post("/api/timeline/settings")
def timeline_settings(h, payload):
    wc = get_webcam()
    if "enabled" in payload:
        wc._timeline_enabled = bool(payload["enabled"])
    if "interval" in payload:
        wc._timeline_interval = max(1.0, min(30.0, float(payload["interval"])))
    send_json(h, 200, {"ok": True, "enabled": wc._timeline_enabled,
                        "interval": wc._timeline_interval})


# ── Telemetry ────────────────────────────────────────────────────────────────

@get("/api/telemetry")
def telemetry(h):
    from roborun.telemetry import TelemetryBus
    try:
        sim = get_simulator()
        robot_id = "sim" if sim and sim.is_running else "local"
    except Exception:
        robot_id = "local"
    latest = TelemetryBus.get().get_latest(robot_id)
    send_json(h, 200, {"ok": True, "telemetry": latest, "robot_id": robot_id})


@get("/api/telemetry/history(?:\\?.*)?")
def telemetry_history(h):
    from roborun.telemetry import TelemetryBus
    qs = parse_qs(urlparse(h.path).query)
    robot_id = qs.get("robot_id", [None])[0]
    channel = qs.get("channel", [None])[0]
    limit = int(qs.get("limit", ["200"])[0])
    data = TelemetryBus.get().get_history(robot_id, channel, limit)
    send_json(h, 200, {"ok": True, "data": data, "count": len(data)})


@get("/api/telemetry/ws-info")
def telemetry_ws(h):
    send_json(h, 200, {"ok": True, "url": "ws://127.0.0.1:8766"})


# ── Trajectory ───────────────────────────────────────────────────────────────

@get("/api/trajectory")
def trajectory(h):
    from roborun.trajectory import TrajectoryRecorder
    rec = TrajectoryRecorder.get()
    traj = rec.get_trajectory(limit=2000)
    send_json(h, 200, {"ok": True, **rec.get_state(), "trajectory": traj})


@post("/api/trajectory/start")
def trajectory_start(h, payload):
    from roborun.trajectory import TrajectoryRecorder
    send_json(h, 200, TrajectoryRecorder.get().start())


@post("/api/trajectory/stop")
def trajectory_stop(h, payload):
    from roborun.trajectory import TrajectoryRecorder
    send_json(h, 200, TrajectoryRecorder.get().stop())


@post("/api/trajectory/clear")
def trajectory_clear(h, payload):
    from roborun.trajectory import TrajectoryRecorder
    send_json(h, 200, TrajectoryRecorder.get().clear())


# ── Depth ────────────────────────────────────────────────────────────────────

@get("/api/depth-frame")
def depth_frame(h):
    from roborun.depth import DepthProcessor
    send_json(h, 200, DepthProcessor.get().get_heatmap())


@get("/api/pointcloud")
def pointcloud(h):
    from roborun.depth import DepthProcessor
    send_json(h, 200, DepthProcessor.get().get_pointcloud())


# ── 3D Scene ─────────────────────────────────────────────────────────────────

@get("/api/scene3d")
def scene3d(h):
    from roborun.routes._singletons import get_scene_builder
    sb = get_scene_builder()
    scene = sb.get_scene()
    scene["is_running"] = sb.is_running
    scene["available"] = sb._available
    scene["has_depth"] = sb._depth_estimator is not None
    scene["last_error"] = sb._last_error
    scene["loop_state"] = sb._loop_state
    send_json(h, 200, scene)


@post("/api/scene3d/start")
def scene3d_start(h, payload):
    from roborun.routes._singletons import get_scene_builder
    sb = get_scene_builder()
    sb._webcam_ref = get_webcam()
    send_json(h, 200, sb.start())


@post("/api/scene3d/stop")
def scene3d_stop(h, payload):
    from roborun.routes._singletons import get_scene_builder
    send_json(h, 200, get_scene_builder().stop())


@post("/api/scene3d/clear")
def scene3d_clear(h, payload):
    from roborun.routes._singletons import get_scene_builder
    send_json(h, 200, get_scene_builder().clear())


# ── Robot Type ───────────────────────────────────────────────────────────────

@get("/api/robot-type")
def robot_type(h):
    from roborun.robot_types import detect_type, get_profile
    from roborun.routes.dashboard import load_profile as lp
    profile = lp()
    sim_type = ""
    try:
        sim = get_simulator()
        if sim.is_running:
            sim_type = sim.get_state().get("robot_type", "")
    except Exception:
        pass
    rtype = detect_type(blueprint=profile.get("blueprint", ""), sim_robot_type=sim_type)
    send_json(h, 200, {"ok": True, **get_profile(rtype)})
