"""Built-in runnable scenarios so the /scenarios board is live out of the box.

Importing this registers a few `@scenario_def`s that run headlessly (no hardware):
a couple of deterministic smoke checks, plus a real vectorized-MJX reach when the
`[mjx]` extra is present. They give the board, suites, and the agent something to
run and score immediately.
"""
from __future__ import annotations

from roborun.scenario_defs import scenario_def


@scenario_def("smoke_pass", suite="demo", tags=["smoke"])
def smoke_pass(ctx):
    """Always-green smoke check — proves the run→score→seal loop end to end."""
    ctx.run.metric("ok", True)
    ctx.run.evaluate("health", uptime=1.0)
    ctx.run.passed(seed=ctx.seed)


@scenario_def("threshold_gain", suite="demo", tags=["tuning"],
              params={"gain": 1.0, "min_gain": 0.8})
def threshold_gain(ctx):
    """Passes when params['gain'] >= params['min_gain'] — a tunable A/B target."""
    g, lo = ctx.params["gain"], ctx.params["min_gain"]
    ctx.run.metric("gain", g)
    ctx.run.passed() if g >= lo else ctx.run.failed(f"gain {g} < {lo}")


def _register_mjx() -> None:
    try:
        from roborun.mjx_env import available
        if not available():
            return
    except Exception:
        return

    @scenario_def("mjx_reach", suite="sim", tags=["mjx", "vectorized"],
                  params={"n_envs": 16, "steps": 15})
    def mjx_reach(ctx):
        """Real vectorized physics: push N MuJoCo carts right; pass if ≥90% move."""
        import numpy as np
        from roborun.mjx_env import make_vec, score_vectorized, SANITY_XML
        n = int(ctx.params["n_envs"])
        env = make_vec(SANITY_XML, n_envs=n)
        r = score_vectorized(
            env,
            policy_fn=lambda o: np.ones((n, env.nu), np.float32),
            reward_fn=lambda o: np.where(np.asarray(o)[:, 0] > 0, 1.0, -1.0),
            steps=int(ctx.params["steps"]), seed=ctx.seed or 0)
        ctx.run.metric("worlds", r["n"])
        ctx.run.evaluate("reward", mean=round(r["mean"], 3), std=round(r["std"], 3))
        ctx.run.passed() if r["pass_rate"] >= 0.9 else ctx.run.failed(
            f"only {int(r['pass_rate']*100)}% reached")


_register_mjx()
