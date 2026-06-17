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


def test_export_writes_provenance_manifest(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    import json, time
    import numpy as np
    from roborun.recorder import RunRecorder
    from roborun.events import runs_root
    from roborun.observations import StreamingExtractor
    from roborun.spatial_memory import SpatialMemoryStore
    from roborun.session import export_dataset

    store = SpatialMemoryStore()
    rec = RunRecorder(robot_id="r1", root=runs_root(), checkpoint_interval=0.05)
    rec.extractor = StreamingExtractor(store, robot_id="r1", run_id=rec.run_id, source="production")
    t0 = time.time()
    for i in range(4):
        rec.write_pose(i * 0.1, 0, 0, ts=t0 + i)
        rec.write_detections([{"label": "pallet", "score": 1, "bbox": [0, 0, 1, 1]}], name="cam", ts=t0 + i)
        rec.write_camera(b"\xff\xd8jpeg" + bytes([i]), name="cam", ts=t0 + i)
    seal = rec.close(do_anchor=False)

    out = tmp_path / "ds"
    r = export_dataset(store, "pallet", str(out), by="label")
    assert r["count"] >= 1 and r["source_runs"] == 1
    prov = json.loads((out / "provenance.json").read_text())
    assert prov["schema"] == "roborun-dataset-provenance/1"
    assert prov["source_runs"] == 1 and prov["sealed_runs"] == 1
    assert prov["verifiable"] is True
    # each image's run traces to the sealed run's Merkle root
    run_info = list(prov["runs"].values())[0]
    assert run_info["merkle_root"] == seal["merkle_root"]
