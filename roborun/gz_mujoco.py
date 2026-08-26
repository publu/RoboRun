"""MuJoCo-backed gz world emulator (GAZEBO_RUNNER_SPEC, in-code).

The gz integration (`gz.GzRunner`) drives a world over a service/topic surface
(`/world/<w>/control`, `/create`, `/clock`, `/odom`). This object presents that
exact surface backed by a **real MuJoCo sim**, so the whole gz code path runs
against real physics with no gz binary and no ROS — the deterministic lockstep
(`multi_step`), entity spawn tracking, and odom readback all exercise live
dynamics. Swapping this for a real ros_gz transport is the only change to go live.
"""
from __future__ import annotations

from typing import Any

from roborun.mjx_env import SANITY_XML


class MujocoGzWorld:
    def __init__(self, model_xml: str = SANITY_XML, world: str = "sim") -> None:
        import mujoco
        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_string(model_xml)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.world = world
        self.spawned: list[dict] = []
        self.stepped = 0
        self.seed = 0

    # ── the gz transport surface GzRunner uses ──────────────────────────────
    def topics(self) -> dict[str, str]:
        return {
            "/clock": "rosgraph_msgs/Clock",
            f"/world/{self.world}/control": "gz.msgs.WorldControl",
            f"/world/{self.world}/create": "gz.msgs.EntityFactory",
            "/odom": "nav_msgs/Odometry",
            "/cmd_vel": "geometry_msgs/Twist",
        }

    def call_service(self, name: str, args: dict | None = None,
                     timeout: float = 5.0) -> dict:
        args = args or {}
        if name.endswith("/control"):
            if args.get("reset"):
                self._mj.mj_resetData(self.model, self.data)
                self.seed = int(args.get("seed", 0))
                # deterministic per-seed qpos jitter
                import numpy as np
                rng = np.random.default_rng(self.seed)
                self.data.qpos[:] += rng.uniform(-0.01, 0.01, size=self.model.nq)
                self._mj.mj_forward(self.model, self.data)
            n = int(args.get("multi_step", 0))
            for _ in range(n):
                self._mj.mj_step(self.model, self.data)
            self.stepped += n
            return {"ok": True, "sim_time": float(self.data.time)}
        if name.endswith("/create"):
            self.spawned.append(args)
            return {"ok": True}
        return {"ok": True}

    # ── odom readback (what ros_telemetry would map to the handle) ──────────
    def odom(self) -> dict:
        q = self.data.qpos
        return {"x": float(q[0]),
                "y": float(q[1]) if self.model.nq > 1 else 0.0,
                "sim_time": float(self.data.time)}
