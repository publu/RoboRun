"""dimOS-style semantic navigation: recall a place from the index, then goto it."""
from __future__ import annotations

import numpy as np
import pytest

from roborun.behaviors import Robot
from roborun.spatial_memory import SpatialMemoryStore


@pytest.fixture()
def memory(tmp_path, monkeypatch):
    store = SpatialMemoryStore(db_path=tmp_path / "m.db")
    store.store(detections=[{"label": "mug", "score": 1, "bbox": [0, 0, 1, 1]}],
                x=3.0, y=2.0, ts=100.0, robot_id="r")
    store.store(detections=[{"label": "dock", "score": 1, "bbox": [0, 0, 1, 1]}],
                x=-1.0, y=4.0, ts=200.0, robot_id="r")
    monkeypatch.setattr("roborun.routes._singletons.get_memory", lambda: store, raising=False)
    return store


def test_recall_place_finds_label(memory):
    r = Robot("t")
    place = r.recall_place("mug", by="label")
    assert place is not None and place["x"] == 3.0 and place["y"] == 2.0


def test_recall_place_missing(memory):
    assert Robot("t").recall_place("spaceship", by="label") is None


def test_go_to_place_navigates_to_recalled_xy(memory, monkeypatch):
    r = Robot("t")
    target = {}
    monkeypatch.setattr(r, "goto", lambda x, z, tol=0.5: target.update(x=x, z=z) or True)
    assert r.go_to_place("dock", by="label") is True
    assert target == {"x": -1.0, "z": 4.0}  # navigated to the recalled place


def test_go_to_place_unknown_stops(memory, monkeypatch):
    r = Robot("t")
    stopped = {"v": False}
    monkeypatch.setattr(r, "stop", lambda: stopped.update(v=True))
    monkeypatch.setattr(r, "goto", lambda *a, **k: True)
    assert r.go_to_place("unicorn", by="label") is False
    assert stopped["v"]
