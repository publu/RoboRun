"""SimBackend — sense a running MuJoCo sim through the handle contract.

Today `robot.move()` reaches the sim (`set_cmd_vel`) but `pose()/lidar()/see()`
read only the arena or ROS — so a MuJoCo-only session is blind through the handle
(LOCAL_SIM_SPEC gap). This backend closes that: it reads `MjData` directly and
exposes the same schema the arena/ros_telemetry provide:

  pose()  -> {x, z, heading, y}   handle frame (x fwd, z = -mujoco_y, CCW yaw)
  lidar() -> 36 floats, metres, [0] dead ahead, CCW   (mj_ray raycasts)
  move(forward, strafe, turn, climb)  -> set_cmd_vel on the runner
  state() -> joints + sim_time (mirror of SimulatorRunner.get_telemetry)

Pure read over `mujoco`; no rendering. Wrap the running `SimulatorRunner`.
"""
from __future__ import annotations

import math
from typing import Any

HANDLE_SECTORS = 36


class SimBackend:
    def __init__(self, runner: Any) -> None:
        self._runner = runner  # SimulatorRunner

    def is_active(self) -> bool:
        return bool(getattr(self._runner, "is_running", False))

    # ── primitives ────────────────────────────────────────────────────────
    def pose(self) -> dict[str, float] | None:
        d = getattr(self._runner, "_data", None)
        if d is None or len(d.qpos) < 7:
            return None
        x, y, z = float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2])
        w, qx, qy, qz = (float(v) for v in d.qpos[3:7])
        heading = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        # handle frame: x forward, z = -mujoco_y (CCW), y = height (aerial)
        return {"x": x, "z": -y, "heading": heading, "y": z}

    def lidar(self, max_range: float = 10.0, eye_h: float = 0.4) -> list[float]:
        import mujoco
        import numpy as np
        m = getattr(self._runner, "_model", None)
        d = getattr(self._runner, "_data", None)
        if m is None or d is None:
            return [max_range] * HANDLE_SECTORS
        px, py, pz = float(d.qpos[0]), float(d.qpos[1]), eye_h
        w, qx, qy, qz = (float(v) for v in d.qpos[3:7])
        yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        out: list[float] = []
        geomid = np.zeros(1, dtype=np.int32)
        for k in range(HANDLE_SECTORS):
            a = yaw + (2 * math.pi * k / HANDLE_SECTORS)  # CCW, [0] dead ahead
            vec = np.array([math.cos(a), math.sin(a), 0.0], dtype=np.float64)
            pnt = np.array([px, py, pz], dtype=np.float64)
            dist = mujoco.mj_ray(m, d, pnt, vec, None, 1, -1, geomid)
            out.append(float(dist) if dist >= 0 else max_range)
        return out

    def see(self, label: str | None = None) -> list:
        # MuJoCo scenes here have no labelled semantic objects; ground-truth
        # `see()` arrives with the level/scene metadata (LOCAL_SIM Phase 1).
        return []

    def move(self, forward: float = 0.0, strafe: float = 0.0,
             turn: float = 0.0, climb: float = 0.0) -> None:
        self._runner.set_cmd_vel(forward, strafe, turn)

    def state(self) -> dict[str, Any]:
        try:
            return self._runner.get_telemetry()
        except Exception:
            return {}
