"""gz integration against a MuJoCo-backed world — real physics, no gz binary."""
from __future__ import annotations

import pytest

pytest.importorskip("mujoco")

from roborun import gz
from roborun.gz_mujoco import MujocoGzWorld


def test_gzrunner_steps_real_physics():
    world = MujocoGzWorld()
    runner = gz.GzRunner(transport=world)
    assert runner.attach() is True
    assert runner.world == "sim"

    out = runner.run_level(
        {"robot": "wheeled", "props": [{"kind": "box", "x": 1, "z": 1}]},
        seed=3, steps=25)
    assert out["status"] == "ran" and out["seed"] == 3
    # real MuJoCo physics actually stepped
    assert world.stepped >= 25
    assert world.data.time > 0
    # entities were spawned via the create service
    assert len(world.spawned) >= 2  # robot + prop


def test_determinism_via_seed():
    def final_x(seed):
        w = MujocoGzWorld()
        r = gz.GzRunner(transport=w)
        r.attach()
        r.run_level({"robot": "wheeled"}, seed=seed, steps=20)
        return w.odom()["x"]

    assert final_x(7) == final_x(7)      # same seed → identical
    assert final_x(7) != final_x(99)     # different seed → different jitter


def test_detect_world_uses_clock():
    w = MujocoGzWorld(world="lobby")
    info = gz.detect_world(transport=w)
    assert info is not None and info["world"] == "lobby"
