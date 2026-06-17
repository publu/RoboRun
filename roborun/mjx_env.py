"""MJX vectorized backend (LOCAL_SIM_SPEC Phase 3 — the Simulate crux).

PufferLib's lesson applied to physical sim: one wrapper, N parallel worlds, flat
fixed obs. MJX (MuJoCo-on-JAX) batches `MjData` and `jax.vmap`s the step, so the
same contract primitives run across thousands of worlds on CPU or GPU. This is the
"thousands of tests in parallel" gap vs Antioch, on our substrate.

Optional dep (`mujoco-mjx`, `jax`). Throughput scales with device; CPU is modest,
GPU is where it pays. The contract (pose/lidar schema) rides on top, so policies
trained here stay portable to the single-instance sim and real robots.
"""
from __future__ import annotations

from typing import Any, Callable


def available() -> bool:
    try:
        import jax  # noqa: F401
        import mujoco.mjx  # noqa: F401
        return True
    except Exception:
        return False


class MJXVecEnv:
    """N MuJoCo worlds stepped in lockstep via jax.vmap. Deterministic given the
    same seed + control stream (the replay/A-B contract)."""

    def __init__(self, model: Any, n_envs: int = 256, ctrl_substeps: int = 4) -> None:
        import jax
        import mujoco
        from mujoco import mjx
        self._jax = jax
        self._mjx = mjx
        self.n = n_envs
        self.ctrl_substeps = ctrl_substeps
        self._mjx_model = mjx.put_model(model)
        self.nq = int(model.nq)
        self.nv = int(model.nv)
        self.nu = int(model.nu)
        d0 = mjx.make_data(model)
        # batch the data across n worlds
        self._data = jax.vmap(lambda _: d0)(jax.numpy.arange(n_envs))
        # jit a vmapped multi-substep step
        def _step(data, ctrl):
            data = data.replace(ctrl=ctrl)
            for _ in range(ctrl_substeps):
                data = mjx.step(self._mjx_model, data)
            return data
        self._step_fn = jax.jit(jax.vmap(_step))

    def reset(self, seed: int = 0):
        """Reset all worlds; seed sets per-world qpos jitter deterministically."""
        import jax
        import jax.numpy as jnp
        key = jax.random.PRNGKey(seed)
        jitter = jax.random.uniform(key, (self.n, self.nq), minval=-0.01, maxval=0.01)
        qpos0 = jnp.broadcast_to(self._data.qpos[0], (self.n, self.nq)) + jitter
        self._data = self._data.replace(
            qpos=qpos0,
            qvel=jnp.zeros((self.n, self.nv)),
            ctrl=jnp.zeros((self.n, self.nu)))
        return self.obs()

    def step(self, ctrl):
        """ctrl: [n_envs, nu] in the model's actuator space. Returns obs [n, *]."""
        import jax.numpy as jnp
        self._data = self._step_fn(self._data, jnp.asarray(ctrl))
        return self.obs()

    def obs(self):
        """Flat per-world obs: concat(qpos, qvel). Subclass/task maps to the
        handle's pose/lidar contract; this is the raw vectorized state."""
        import jax.numpy as jnp
        return jnp.concatenate([self._data.qpos, self._data.qvel], axis=1)


    def measure_sps(self, steps: int = 100, seed: int = 0) -> dict:
        """Steps-per-second on the present device (CPU here; GPU pays off). The
        budget the LOCAL_SIM/PERF specs ask for — measured, not asserted."""
        import time
        import numpy as np
        self.reset(seed)
        ctrl = np.zeros((self.n, self.nu), np.float32)
        self.step(ctrl)  # warm jit
        t0 = time.time()
        for _ in range(steps):
            self.step(ctrl)
        dt = time.time() - t0
        sps = self.n * steps / dt if dt > 0 else 0.0
        return {"n_envs": self.n, "steps": steps, "seconds": round(dt, 3),
                "sps": int(sps)}


def rollout(env: MJXVecEnv, policy_fn: Callable, steps: int, seed: int = 0):
    """Run a vectorized rollout: policy_fn(obs)->ctrl[n,nu]. Returns the final
    obs. Single code path whether n=1 (a CleanRL-style env) or n=10k."""
    import numpy as np
    obs = env.reset(seed)
    for _ in range(steps):
        obs = env.step(np.asarray(policy_fn(obs)))
    return obs


def make_vec(model_xml: str, n_envs: int = 256) -> MJXVecEnv:
    import mujoco
    model = mujoco.MjModel.from_xml_string(model_xml)
    return MJXVecEnv(model, n_envs=n_envs)


def score_vectorized(env: MJXVecEnv, policy_fn: Callable, reward_fn: Callable,
                     steps: int, seed: int = 0) -> dict:
    """Run N worlds and reduce to a pass-rate — the bridge from Simulate (MJX) to
    Define (`scenario`). `reward_fn(obs)->float[n]` scores each world; a world
    "passes" when its final reward >= 0. Returns per-world + aggregate, so the
    scenario layer can record a sealed, scored MJX sweep.

        with scenario("mjx-reach", suite="sim", seed=seed) as run:
            r = score_vectorized(env, policy, reward, steps, seed)
            run.metric("worlds", r["n"]); run.evaluate("reward", mean=r["mean"])
            run.passed() if r["pass_rate"] >= 0.9 else run.failed("low pass-rate")
    """
    import numpy as np
    obs = rollout(env, policy_fn, steps, seed)
    rewards = np.asarray(reward_fn(obs)).reshape(-1)
    passed = int((rewards >= 0).sum())
    n = int(rewards.shape[0])
    return {"n": n, "passed": passed,
            "pass_rate": round(passed / n, 3) if n else 0.0,
            "mean": float(rewards.mean()), "std": float(rewards.std())}


# A minimal model for smoke tests / the "Ocean"-style sanity env (PufferLib idea):
# a single actuated slider — trains/sanity-checks in seconds, no assets.
SANITY_XML = """
<mujoco>
  <option timestep="0.01"/>
  <worldbody>
    <body name="cart" pos="0 0 0">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator><motor joint="slide" gear="10"/></actuator>
</mujoco>
"""
