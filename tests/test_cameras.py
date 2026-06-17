"""Multi-camera manager: N source-tagged pipelines → per-source MCAP channels."""
from __future__ import annotations

import numpy as np

from roborun.cameras import CameraManager
from roborun.recorder import RunRecorder


class StubCam:
    def __init__(self, label):
        self.label = label
        self.started = False
    def start(self, **kw): self.started = True; return {"ok": True}
    def stop(self): self.started = False
    def snapshot(self):
        return np.full((48, 64, 3), 100, np.uint8)
    def get_detections(self):
        return [{"label": self.label, "score": 0.9, "bbox": [0, 0, 1, 1]}]


def test_register_and_list():
    m = CameraManager()
    m.register("front", StubCam("person"))
    m.register("rear", StubCam("car"))
    assert m.sources() == ["front", "rear"]
    res = m.start_all()
    assert res["front"]["ok"] and res["rear"]["ok"]


def test_snapshot_and_detections_route_by_source():
    m = CameraManager()
    m.register("front", StubCam("person"))
    m.register("rear", StubCam("car"))
    assert m.detections("front")[0]["label"] == "person"
    assert m.detections("rear")[0]["label"] == "car"
    assert m.snapshot("front") is not None
    assert m.snapshot("nope") is None


def test_record_into_writes_per_source_channels(tmp_path):
    m = CameraManager()
    m.register("front", StubCam("person"))
    m.register("rear", StubCam("car"))
    rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.05)
    n = m.record_into(rec)
    seal = rec.close(do_anchor=False)
    assert n == 2
    mc = seal["message_counts"]
    assert mc.get("/camera/front") and mc.get("/camera/rear")
    assert mc.get("/detections/front") and mc.get("/detections/rear")
