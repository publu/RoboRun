#!/usr/bin/env python3
"""Bench: CLIP recall latency vs vector count (PERF #1).

Shows where numpy matmul starts to hurt — the signal to flip on sqlite-vec.

    python scripts/bench_clip.py [n]
"""
from __future__ import annotations

import sys
import tempfile
import time
from pathlib import Path

import numpy as np

from roborun.spatial_memory import SpatialMemoryStore


def bench(n: int = 50_000, dim: int = 512) -> float:
    s = SpatialMemoryStore(db_path=Path(tempfile.mkdtemp()) / "m.db")
    rng = np.random.default_rng(0)
    for i in range(n):
        s.store(embedding=rng.standard_normal(dim).astype(np.float32),
                detections=[{"label": "x", "score": 1.0, "bbox": [0, 0, 1, 1]}],
                ts=float(i))
    q = rng.standard_normal(dim).astype(np.float32)
    s.recall(q, by="clip", k=10)  # warm cache
    t0 = time.time()
    for _ in range(20):
        s.recall(q, by="clip", k=10)
    return (time.time() - t0) / 20


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 50_000
    dt = bench(n)
    print(f"clip recall @ {n} vec: {dt*1000:.2f} ms/query (target <10 ms p95 @1M)")
