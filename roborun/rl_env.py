"""RoboRunEnv — the episode wrapper over the handle (LOCAL_SIM_SPEC Phase 2).

RoboRun is a control loop today; RL needs reset/step/reward/done. This wraps any
backend (SimBackend / arena / ros) + a task into a Gymnasium-style env with a
**flat, fixed-size** observation (PufferLib's emulation lesson):

    obs = concat(pose[3], lidar[36], see_padded[N*K], last_action[4], state_scalars)

A task is `(reset_fn, reward_fn, done_fn)` defined once against the handle, so it
runs on every backend and embodiment by construction (the contract).

Gymnasium is optional: if installed we subclass `gymnasium.Env`; otherwise we
expose the same `reset/step` duck-type so tests + simple loops work with no dep.
"""
from __future__ import annotations

from typing import Any, Callable

try:  # optional
    import gymnasium as gym
    _Base = gym.Env
except Exception:  # pragma: no cover - gymnasium not installed
    gym = None
    _Base = object

import numpy as np

SEE_N = 5          # max detections in the obs (pad/truncate) — fixed shape
SEE_K = 4          # per-detection features: cx, cy, dist, present-mask
ACTION_DIM = 4     # forward, strafe, turn, climb


def obs_dim() -> int:
    return 3 + 36 + SEE_N * SEE_K + ACTION_DIM + 1  # +1 sim_time-ish scalar


def _encode_see(things: list) -> np.ndarray:
    out = np.zeros((SEE_N, SEE_K), dtype=np.float32)
    for i, t in enumerate(things[:SEE_N]):
        cx = getattr(t, "cx", None)
        if cx is None and isinstance(t, dict):
            cx = t.get("cx", 0.0)
        dist = getattr(t, "dist", None)
        if dist is None and isinstance(t, dict):
            dist = t.get("distance")
        out[i] = [float(cx or 0.0),
                  float(getattr(t, "cy", 0.0) if not isinstance(t, dict) else t.get("cy", 0.0)),
                  float(dist or 0.0), 1.0]  # presence mask
    return out.flatten()


class RoboRunEnv(_Base):
    """task = (reset_fn, reward_fn, done_fn); each takes the backend (+ env)."""

    def __init__(self, backend: Any, task: tuple[Callable, Callable, Callable],
                 profile: str = "dog", max_steps: int = 500) -> None:
        self.backend = backend
        self.reset_fn, self.reward_fn, self.done_fn = task
        self.profile = profile
        self.max_steps = max_steps
        self._last_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._steps = 0
        # capability gate: drone-only climb is masked off otherwise
        self._climb_ok = profile == "drone"
        if gym is not None:
            self.observation_space = gym.spaces.Box(-np.inf, np.inf,
                                                    (obs_dim(),), np.float32)
            self.action_space = gym.spaces.Box(-1.0, 1.0, (ACTION_DIM,), np.float32)

    def _obs(self) -> np.ndarray:
        p = self.backend.pose() or {"x": 0, "z": 0, "heading": 0}
        pose = np.array([p.get("x", 0), p.get("z", 0), p.get("heading", 0)], np.float32)
        lidar = np.asarray(self.backend.lidar(), np.float32)
        if lidar.shape[0] != 36:
            lidar = np.zeros(36, np.float32)
        see = _encode_see(self.backend.see())
        st = self.backend.state() if hasattr(self.backend, "state") else {}
        scalar = np.array([float(st.get("sim_time", 0.0))], np.float32)
        return np.concatenate([pose, lidar, see, self._last_action, scalar])

    def reset(self, seed: int | None = None, options=None):
        if seed is not None and gym is not None:
            super().reset(seed=seed)
        self._steps = 0
        self._last_action[:] = 0
        if self.reset_fn:
            self.reset_fn(self.backend)
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, np.float32), -1.0, 1.0)
        if not self._climb_ok:
            a[3] = 0.0  # capability gate
        self.backend.move(float(a[0]), float(a[1]), float(a[2]), float(a[3]))
        self._last_action = a
        self._steps += 1
        obs = self._obs()
        reward = float(self.reward_fn(self.backend)) if self.reward_fn else 0.0
        terminated = bool(self.done_fn(self.backend)) if self.done_fn else False
        truncated = self._steps >= self.max_steps
        return obs, reward, terminated, truncated, {}
