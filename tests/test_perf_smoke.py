"""Perf smoke gates (PERFORMANCE_SPEC) — loose ceilings so regressions fail CI
without flaking on slow machines. The benches under scripts/ measure precisely."""
from __future__ import annotations

import tempfile
import time
from pathlib import Path

import numpy as np

from roborun.recorder import RunRecorder
from roborun.spatial_memory import SpatialMemoryStore


def test_async_seal_returns_fast(monkeypatch):
    # Even with a slow TSA, async seal must return quickly (PERF #2).
    import roborun.recorder as R
    monkeypatch.setattr(R.anchor, "stamp_digest",
                        lambda d, **k: (time.sleep(2.0), None)[1])
    rec = RunRecorder(robot_id="b", root=Path(tempfile.mkdtemp()), checkpoint_interval=0.05)
    rec.write_pose(0, 0, 0)
    t0 = time.time()
    rec.close(do_anchor=True, anchor_async=True)
    assert time.time() - t0 < 0.5


def test_clip_recall_reasonable_at_5k():
    s = SpatialMemoryStore(db_path=Path(tempfile.mkdtemp()) / "m.db")
    rng = np.random.default_rng(0)
    for i in range(5000):
        s.store(embedding=rng.standard_normal(64).astype(np.float32), ts=float(i))
    q = rng.standard_normal(64).astype(np.float32)
    s.recall(q, by="clip", k=5)  # warm
    t0 = time.time()
    s.recall(q, by="clip", k=5)
    assert time.time() - t0 < 0.25  # generous ceiling for 5k×64 numpy


def test_list_keys_accepts_start_after_signature():
    # The cursor param exists for incremental fetch (no R2 needed to check API).
    import inspect
    from roborun.r2sync import R2Store
    assert "start_after" in inspect.signature(R2Store.list_keys).parameters
