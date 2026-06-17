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


def make_vec(model_xml: str, n_envs: int = 256) -> MJXVecEnv:
    import mujoco
    model = mujoco.MjModel.from_xml_string(model_xml)
    return MJXVecEnv(model, n_envs=n_envs)


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
