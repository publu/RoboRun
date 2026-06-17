"""Curate a labeled dataset from a search (Foxglove parity, sealed provenance)."""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from roborun.session import export_dataset
from roborun.spatial_memory import SpatialMemoryStore


def test_export_writes_images_and_labels(tmp_path):
    store = SpatialMemoryStore(db_path=tmp_path / "m.db")
    # frames stored with thumbnails (no MCAP needed — falls back to thumb)
    frame = np.full((48, 64, 3), 120, np.uint8)
    for i in range(3):
        store.store(frame=frame,
                    detections=[{"label": "forklift", "score": 0.9, "bbox": [0, 0, 1, 1]}],
                    ts=time.time() + i, robot_id="r1", source="production")
    store.store(frame=frame,
                detections=[{"label": "person", "score": 0.9, "bbox": [0, 0, 1, 1]}],
                ts=time.time(), robot_id="r1", source="production")

    out = tmp_path / "ds"
    r = export_dataset(store, "forklift", str(out), by="label", k=10)
    assert r["ok"] and r["count"] == 3                 # only the forklifts
    assert (out / "dataset.json").exists()
    imgs = list((out / "images").glob("*.jpg"))
    assert len(imgs) == 3
    lines = (out / "labels.jsonl").read_text().strip().splitlines()
    assert len(lines) == 3
    rec = json.loads(lines[0])
    assert rec["detections"][0]["label"] == "forklift"
    assert rec["image"].startswith("images/")
    meta = json.loads((out / "dataset.json").read_text())
    assert meta["query"] == "forklift" and meta["count"] == 3


def test_export_empty_query(tmp_path):
    store = SpatialMemoryStore(db_path=tmp_path / "m.db")
    r = export_dataset(store, "nothing", str(tmp_path / "ds"), by="label")
    assert r["ok"] and r["count"] == 0
