"""Unified recall() over the spatial memory store (PERCEPTION_DATA_SPEC).

One entry point across clip/label/near/time. clip is tested with an ndarray
embedding so no vision model is needed.
"""
from __future__ import annotations

import numpy as np
import pytest

from roborun.spatial_memory import SpatialMemoryStore


@pytest.fixture()
def store(tmp_path):
    s = SpatialMemoryStore(db_path=tmp_path / "m.db")
    # three observations with distinct embeddings, labels, and positions
    s.store(embedding=np.array([1, 0, 0], dtype=np.float32),
            detections=[{"label": "mug", "score": 0.9, "bbox": [0, 0, 1, 1]}],
            x=0.0, y=0.0, ts=100.0, robot_id="r1")
    s.store(embedding=np.array([0, 1, 0], dtype=np.float32),
            detections=[{"label": "person", "score": 0.8, "bbox": [0, 0, 1, 1]}],
            x=5.0, y=5.0, ts=200.0, robot_id="r1")
    s.store(embedding=np.array([0, 0, 1], dtype=np.float32),
            detections=[{"label": "mug", "score": 0.7, "bbox": [0, 0, 1, 1]}],
            x=0.3, y=0.2, ts=300.0, robot_id="r2")
    return s


def test_recall_clip_ndarray(store):
    hits = store.recall(np.array([1, 0, 0], dtype=np.float32), by="clip", k=1)
    assert hits and hits[0]["detections"][0]["label"] == "mug"


def test_recall_label(store):
    hits = store.recall("mug", by="label", k=10)
    assert len(hits) == 2
    assert all(any(d["label"] == "mug" for d in h["detections"]) for h in hits)


def test_recall_near(store):
    hits = store.recall(by="near", k=10, x=0.0, y=0.0, radius=1.0)
    # the two mugs near origin, not the person at (5,5)
    assert len(hits) == 2


def test_recall_time(store):
    hits = store.recall(by="time", k=10, since=150.0)
    assert {h["ts"] for h in hits} == {200.0, 300.0}


def test_recall_robot_filter(store):
    assert all(h["robot_id"] == "r2" for h in store.recall("mug", by="label", robot_id="r2"))


def test_recall_unknown_mode(store):
    with pytest.raises(ValueError):
        store.recall("x", by="bogus")
