"""Capstone: a scored scenario over N vectorized MJX worlds (Simulate ↔ Define).

Proves the full loop — vectorized MJX rollout → reward reduction → a sealed,
scored `scenario` record with a pass-rate — end to end on CPU.
"""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("jax")
pytest.importorskip("mujoco.mjx")

from roborun.mjx_env import make_vec, score_vectorized, SANITY_XML
from roborun import scenario as S


@pytest.fixture(autouse=True)
def _isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))


def test_score_vectorized_shapes():
    env = make_vec(SANITY_XML, n_envs=32)
    # push right; reward = +1 if cart moved (qpos[0] > 0), else -1
    r = score_vectorized(
        env,
        policy_fn=lambda o: np.ones((32, env.nu), np.float32),
        reward_fn=lambda o: np.where(np.asarray(o)[:, 0] > 0, 1.0, -1.0),
        steps=15, seed=0)
    assert r["n"] == 32
    assert r["pass_rate"] == 1.0  # a steady push moves every cart right


def test_mjx_sweep_records_a_scored_scenario():
    env = make_vec(SANITY_XML, n_envs=16)
    seed = 5
    with S.scenario("mjx-reach", suite="sim", seed=seed,
                    params={"n_envs": 16}) as run:
        r = score_vectorized(
            env,
            policy_fn=lambda o: np.ones((16, env.nu), np.float32),
            reward_fn=lambda o: np.where(np.asarray(o)[:, 0] > 0, 1.0, -1.0),
            steps=15, seed=seed)
        run.metric("worlds", r["n"])
        run.evaluate("reward", mean=r["mean"], std=r["std"])
        run.passed() if r["pass_rate"] >= 0.9 else run.failed("low pass-rate")

    rows = S.list_results(suite="sim")
    assert len(rows) == 1
    rec = rows[0]
    assert rec["outcome"] == "passed"
    assert rec["seed"] == seed
    assert rec["metrics"]["worlds"] == 16
    assert "reward" in rec["evaluation"]
