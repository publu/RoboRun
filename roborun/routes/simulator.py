"""Simulator routes."""
from __future__ import annotations

from roborun.routes import get, post, send_json
from roborun.routes._singletons import get_webcam
from roborun.routes.tasks import log_event

_SIM_UNAVAILABLE = {"ok": False, "error": "Simulator not available (mujoco not installed)"}
_DEFAULT_STATE = {"running": False, "state": "idle", "robot": "", "robot_type": "",
                  "fps": 0, "frame_count": 0, "sim_time": 0,
                  "position": {"x": 0, "y": 0, "z": 0},
                  "orientation": {"w": 1, "x": 0, "y": 0, "z": 0},
                  "has_policy": False, "has_drone_ctrl": False}


def _get_sim():
    try:
        from roborun.routes._singletons import get_simulator
        return get_simulator()
    except Exception:
        return None


@get("/api/sim/robots")
def list_robots(h):
    sim = _get_sim()
    if sim:
        send_json(h, 200, {"ok": True, "robots": sim.list_robots()})
    else:
        send_json(h, 200, {"ok": True, "robots": []})


@get("/api/sim/state")
def state(h):
    sim = _get_sim()
    if sim:
        send_json(h, 200, {"ok": True, **sim.get_state()})
    else:
        send_json(h, 200, {"ok": True, **_DEFAULT_STATE})


@post("/api/sim/start")
def start(h, payload):
    sim = _get_sim()
    if not sim:
        send_json(h, 200, _SIM_UNAVAILABLE)
        return
    get_webcam().stop()
    robot = payload.get("robot", "unitree_go1")
    result = sim.start(robot_id=robot)
    if result.get("ok"):
        log_event("sim_started", f"Sim: {robot}")
    send_json(h, 200, result)


@post("/api/sim/stop")
def stop(h, payload):
    sim = _get_sim()
    if not sim:
        send_json(h, 200, _SIM_UNAVAILABLE)
        return
    result = sim.stop()
    log_event("sim_stopped", "Simulator stopped")
    send_json(h, 200, result)


@post("/api/sim/reset")
def reset(h, payload):
    sim = _get_sim()
    if not sim:
        send_json(h, 200, _SIM_UNAVAILABLE)
        return
    result = sim.reset()
    if result.get("ok"):
        log_event("sim_reset", "Simulator robot reset")
    send_json(h, 200, result)


@post("/api/sim/move")
def move(h, payload):
    sim = _get_sim()
    if not sim:
        send_json(h, 200, _SIM_UNAVAILABLE)
        return
    sim.set_cmd_vel(
        forward=float(payload.get("forward", 0)),
        left=float(payload.get("left", 0)),
        turn=float(payload.get("turn", 0)),
    )
    send_json(h, 200, {"ok": True})


@post("/api/sim/waypoint")
def waypoint(h, payload):
    sim = _get_sim()
    if not sim:
        send_json(h, 200, _SIM_UNAVAILABLE)
        return
    result = sim.set_waypoint(
        x=float(payload.get("x", 0)),
        y=float(payload.get("y", 0)),
        z=float(payload.get("z", 2.0)),
    )
    send_json(h, 200, result)


@post("/api/sim/altitude")
def altitude(h, payload):
    sim = _get_sim()
    if not sim:
        send_json(h, 200, _SIM_UNAVAILABLE)
        return
    result = sim.set_altitude(float(payload.get("altitude", 2.0)))
    send_json(h, 200, result)
