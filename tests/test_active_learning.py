"""Active learning: curate the examples the model is uncertain about."""
from __future__ import annotations

import numpy as np

from roborun.spatial_memory import SpatialMemoryStore


def _store(tmp_path):
    s = SpatialMemoryStore(db_path=tmp_path / "m.db")
    f = np.full((32, 32, 3), 80, np.uint8)   # a frame so exports have thumbnails
    # confident, uncertain, and very-low detections
    s.store(frame=f, detections=[{"label": "person", "score": 0.95, "bbox": [0, 0, 1, 1]}], ts=1.0)
    s.store(frame=f, detections=[{"label": "person", "score": 0.45, "bbox": [0, 0, 1, 1]}], ts=2.0)  # uncertain
    s.store(frame=f, detections=[{"label": "person", "score": 0.50, "bbox": [0, 0, 1, 1]}], ts=3.0)  # uncertain
    s.store(frame=f, detections=[{"label": "forklift", "score": 0.40, "bbox": [0, 0, 1, 1]}], ts=4.0)  # uncertain
    s.store(frame=f, detections=[{"label": "person", "score": 0.10, "bbox": [0, 0, 1, 1]}], ts=5.0)  # too low
    return s


def test_search_uncertain_picks_midband(tmp_path):
    s = _store(tmp_path)
    rows = s.search_uncertain(lo=0.3, hi=0.6)
    scores = [d["score"] for r in rows for d in r["detections"]]
    assert rows and all(0.3 <= sc <= 0.6 for sc in scores)
    assert len(rows) == 3   # the two uncertain persons + the uncertain forklift


def test_recall_uncertain_with_label(tmp_path):
    s = _store(tmp_path)
    rows = s.recall("person", by="uncertain", k=10)
    assert len(rows) == 2   # only the two uncertain persons
    assert all(any(d["label"] == "person" for d in r["detections"]) for r in rows)


def test_export_uncertain_dataset(tmp_path):
    from roborun.session import export_dataset
    s = _store(tmp_path)
    r = export_dataset(s, "", str(tmp_path / "al"), by="uncertain")
    assert r["count"] == 3   # the active-learning set, ready to label
