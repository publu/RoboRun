#!/usr/bin/env python3
"""Bench: MJX vectorized throughput (LOCAL_SIM Phase 3 / Simulate crux).

Reports steps-per-second on the present device across batch sizes. CPU is modest;
a GPU is where 10k–100k SPS lands. Run:  python scripts/bench_mjx.py
"""
from __future__ import annotations

from roborun.mjx_env import make_vec, SANITY_XML, available


def main() -> int:
    if not available():
        print("mjx/jax not installed — pip install 'ros-agent[mjx]'")
        return 1
    import jax
    print(f"device: {jax.devices()[0].platform}")
    for n in (64, 256, 1024, 4096):
        env = make_vec(SANITY_XML, n_envs=n)
        r = env.measure_sps(steps=100)
        print(f"  n_envs={r['n_envs']:>5}  {r['sps']:>9,} SPS  ({r['seconds']}s/100 steps)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
