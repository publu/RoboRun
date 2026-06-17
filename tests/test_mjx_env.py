"""MJX vectorized env (LOCAL_SIM Phase 3): N worlds, batched step, determinism."""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("jax")
pytest.importorskip("mujoco.mjx")

from roborun.mjx_env import MJXVecEnv, make_vec, SANITY_XML, available


def test_available():
    assert available()


def test_vectorized_step_shapes():
    env = make_vec(SANITY_XML, n_envs=8)
    obs = env.reset(seed=0)
    assert obs.shape[0] == 8                     # 8 parallel worlds
    ctrl = np.ones((8, env.nu), np.float32)
    obs2 = env.step(ctrl)
    assert obs2.shape[0] == 8
    # a push moved the cart (qpos changed)
    assert not np.allclose(np.asarray(obs[:, : env.nq]), np.asarray(obs2[:, : env.nq]))


def test_determinism_same_seed():
    e1 = make_vec(SANITY_XML, n_envs=4); e1.reset(seed=7)
    e2 = make_vec(SANITY_XML, n_envs=4); e2.reset(seed=7)
    ctrl = np.full((4, e1.nu), 0.5, np.float32)
    for _ in range(10):
        o1 = np.asarray(e1.step(ctrl))
        o2 = np.asarray(e2.step(ctrl))
    assert np.allclose(o1, o2, atol=1e-6)        # same seed+ctrl → identical

    e3 = make_vec(SANITY_XML, n_envs=4); e3.reset(seed=8)
    o3 = np.asarray(e3.step(ctrl))
    assert not np.allclose(o1, o3)               # different seed → different jitter


def test_measure_sps_and_rollout():
    env = make_vec(SANITY_XML, n_envs=16)
    r = env.measure_sps(steps=20)
    assert r["sps"] > 0 and r["n_envs"] == 16

    from roborun.mjx_env import rollout
    obs = rollout(env, lambda o: np.zeros((16, env.nu), np.float32), steps=5, seed=1)
    assert obs.shape[0] == 16
