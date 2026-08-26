"""Streaming extraction: Observations indexed live, not only on close."""
from __future__ import annotations

import numpy as np

from roborun.recorder import RunRecorder
from roborun.observations import StreamingExtractor
from roborun.spatial_memory import SpatialMemoryStore


def test_rows_appear_before_close(tmp_path):
    store = SpatialMemoryStore(db_path=tmp_path / "m.db")
    rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.05)
    rec.extractor = StreamingExtractor(store, robot_id="t", run_id=rec.run_id)

    t = 1000.0
    for i in range(5):
        rec.write_pose(float(i), 0.0, 0.0, ts=t + i)
        rec.write_detections([{"label": "person", "score": 0.9, "bbox": [0, 0, 1, 1]}],
                             ts=t + i)
        rec.write_clip(np.array([1, 0, 0], np.float32), ts=t + i)
        rec.write_camera(b"\xff\xd8jpeg", name="front", ts=t + i)

    # indexed live — queryable WITHOUT closing the run
    rows = store.recall("person", by="label")
    assert len(rows) == 5
    assert rows[0]["source"] == "stream"
    assert rows[0]["source_id"] == "front"
    assert rows[0]["x"] is not None  # pose joined within tolerance

    rec.close(do_anchor=False)


def test_extractor_optional_default_off(tmp_path):
    rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.05)
    assert rec.extractor is None  # opt-in; no coupling by default
    rec.write_camera(b"\xff\xd8x", ts=1.0)  # must not raise
    rec.close(do_anchor=False)
