#!/usr/bin/env python3
"""Bench: recorder seal latency (PERF #2). Async seal must return ~instantly.

    python scripts/bench_seal.py
"""
from __future__ import annotations

import tempfile
import time
from pathlib import Path

from roborun.recorder import RunRecorder


def bench(n_msgs: int = 200) -> float:
    root = Path(tempfile.mkdtemp())
    rec = RunRecorder(robot_id="bench", root=root, checkpoint_interval=0.05)
    t0 = time.time()
    for i in range(n_msgs):
        rec.write_pose(float(i), 0.0, 0.0, ts=t0 + i * 0.05)
    t_close = time.time()
    rec.close(do_anchor=True, anchor_async=True)
    dt = time.time() - t_close
    return dt


if __name__ == "__main__":
    dt = bench()
    print(f"seal returned in {dt*1000:.1f} ms (target <100 ms, async anchor)")
    raise SystemExit(0 if dt < 0.1 else 1)
