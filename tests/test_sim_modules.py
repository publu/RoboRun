"""SimBackend (MjData read), RoboRunEnv (episode wrapper), gz level mapping."""
from __future__ import annotations

import numpy as np
import pytest

from roborun.rl_env import RoboRunEnv, obs_dim, ACTION_DIM
from roborun import gz


# ── SimBackend against a real headless MuJoCo sim ───────────────────────────

@pytest.fixture()
def sim():
    mujoco = pytest.importorskip("mujoco")
    from roborun.simulator import SimulatorRunner
    r = SimulatorRunner()
    r.start("unitree_go1", render=False)
    import time
    time.sleep(0.6)
    yield r
    r.stop(); time.sleep(0.3)


def test_sim_backend_pose_and_lidar_schema(sim):
    from roborun.sim_backend import SimBackend
    b = SimBackend(sim)
    p = b.pose()
    assert set(["x", "z", "heading"]).issubset(p)         # handle frame
    lid = b.lidar()
    assert len(lid) == 36 and all(isinstance(v, float) for v in lid)
    assert b.is_active()


# ── RoboRunEnv with a fake backend (no sim/gym needed) ──────────────────────

class FakeBackend:
    def __init__(self): self.x = 0.0; self.moves = 0
    def pose(self): return {"x": self.x, "z": 0.0, "heading": 0.0}
    def lidar(self): return [5.0] * 36
    def see(self): return []
    def state(self): return {"sim_time": self.moves * 0.1}
    def move(self, f, s, t, c): self.x += f; self.moves += 1


def _task(goal=2.0):
    reset = lambda b: setattr(b, "x", 0.0)
    reward = lambda b: -abs(goal - b.pose()["x"])
    done = lambda b: b.pose()["x"] >= goal
    return (reset, reward, done)


def test_env_obs_is_fixed_size():
    env = RoboRunEnv(FakeBackend(), _task(), profile="dog", max_steps=10)
    obs, _ = env.reset(seed=1)
    assert obs.shape == (obs_dim(),)


def test_env_step_terminates_on_goal():
    env = RoboRunEnv(FakeBackend(), _task(goal=1.0), profile="dog", max_steps=50)
    env.reset()
    term = False
    for _ in range(50):
        _, r, term, trunc, _ = env.step(np.array([0.5, 0, 0, 0], np.float32))
        if term or trunc:
            break
    assert term  # reached goal before truncation


def test_env_capability_gates_climb_for_nondrone():
    b = FakeBackend()
    env = RoboRunEnv(b, _task(), profile="dog")
    env.reset()
    # climb requested but profile=dog → masked; backend still moves forward
    env.step(np.array([0.2, 0, 0, 1.0], np.float32))
    assert b.moves == 1


# ── gz level → SDF mapping (pure) ───────────────────────────────────────────

def test_gz_level_to_spawns():
    level = {"robot": "dog", "spawn": {"x": 1.0, "z": 2.0, "heading": 0.5},
             "props": [{"kind": "box", "x": 3.0, "z": 0.0}],
             "walls": [[[0, 0], [1, 0]]]}
    spawns = gz.level_to_spawns(level, seed=7)
    names = [s["name"] for s in spawns]
    assert "robot" in names and "prop_0" in names and "wall_0" in names
    robot = next(s for s in spawns if s["name"] == "robot")
    assert robot["type"] == "go2"
    assert robot["pose"]["y"] == -2.0  # handle z → -mujoco/gz y


def test_gz_detect_world_none_without_transport():
    assert gz.detect_world(transport=None) is None
    assert gz.GzClock("default", transport=None).step(1) is None


# ── gz capability matrix + GzRunner dry-run (no ROS/gz needed) ───────────────

def test_gazebo_in_capability_matrix():
    from roborun.transport import CAPABILITY_MATRIX
    assert "gazebo" in CAPABILITY_MATRIX
    g = CAPABILITY_MATRIX["gazebo"]
    assert g["clock_control"] and g["spawn"] and g["services"]


def test_gz_runner_dry_run_without_world():
    r = gz.GzRunner(transport=None)
    assert r.attach() is False
    out = r.run_level({"robot": "wheeled", "props": [{"kind": "box", "x": 1, "z": 1}]})
    assert out["status"] == "no-world"
    assert any(s["name"] == "robot" for s in out["plan"])


def test_gz_runner_runs_against_mock_world():
    """Exercise the full GzRunner.run_level 'ran' path with a fake gz service
    surface (no ROS needed) — proves detect→spawn→lockstep orchestration."""
    calls = {"create": 0, "control": 0}

    class FakeTransport:
        def topics(self):
            return {"/clock": "rosgraph_msgs/Clock",
                    "/world/lobby/control": "srv", "/world/lobby/create": "srv"}
        def call_service(self, name, args=None, timeout=5.0):
            if name.endswith("/create"): calls["create"] += 1
            elif name.endswith("/control"): calls["control"] += 1
            return {"ok": True}

    r = gz.GzRunner(transport=FakeTransport())
    assert r.attach() is True and r.world == "lobby"
    out = r.run_level({"robot": "dog", "props": [{"kind": "box", "x": 1, "z": 1}]},
                      seed=3, steps=10)
    assert out["status"] == "ran" and out["world"] == "lobby" and out["seed"] == 3
    assert calls["create"] >= 2          # robot + prop spawned
    assert calls["control"] >= 10        # lockstep steps issued
