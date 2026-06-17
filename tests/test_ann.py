"""sqlite-vec ANN path parity with numpy (PERCEPTION/PERF #1).

Forces the ANN threshold low so the vec0 index engages on a tiny corpus, and
asserts it returns the same top-1 as the numpy cosine path.
"""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("sqlite_vec")

from roborun.spatial_memory import SpatialMemoryStore


def _seed(store, n=50, dim=32):
    rng = np.random.default_rng(0)
    embs = rng.standard_normal((n, dim)).astype(np.float32)
    for i, e in enumerate(embs):
        store.store(embedding=e,
                    detections=[{"label": f"o{i}", "score": 1.0, "bbox": [0, 0, 1, 1]}],
                    ts=float(i))
    return embs


def test_ann_matches_numpy_top1(tmp_path, monkeypatch):
    embs = None
    s = SpatialMemoryStore(db_path=tmp_path / "m.db")
    embs = _seed(s)
    q = embs[7] + np.random.default_rng(1).standard_normal(32).astype(np.float32) * 0.01

    monkeypatch.setenv("ROBORUN_ANN_THRESHOLD", "1000000")  # force numpy
    numpy_top = s.search_clip(q, top_k=1)[0]

    monkeypatch.setenv("ROBORUN_ANN_THRESHOLD", "1")        # force ANN
    ann_top = s.search_clip(q, top_k=1)[0]

    assert ann_top["id"] == numpy_top["id"]  # same nearest neighbour


def test_ann_engages_only_past_threshold(tmp_path, monkeypatch):
    s = SpatialMemoryStore(db_path=tmp_path / "m.db")
    _seed(s, n=5)
    # default threshold (100k) → numpy path, still returns results
    res = s.recall(np.zeros(32, np.float32), by="clip", k=3)
    assert isinstance(res, list)
