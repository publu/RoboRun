"""The unified system: source → YOLO+CLIP → index → search anyone over time.

Uses a synthetic camera + a fake embedder (color→vector) so the full loop runs
with no torch/hardware, proving sim/robot/production share one pipeline + search.
"""
from __future__ import annotations

import time

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from roborun.session import PerceptionSession, search, MODES, open_source
from roborun.synthetic_camera import SyntheticCamera
from roborun.spatial_memory import SpatialMemoryStore


def _fake_embed(frame):
    # mean BGR as a 3-vector → distinct "semantic" signature per object color
    return frame.reshape(-1, 3).mean(axis=0).astype(np.float32)


def test_modes_enumerated():
    assert MODES == ("sim", "robot", "production")
    assert isinstance(open_source("sim", label="person"), SyntheticCamera)


def test_session_indexes_yolo_and_clip(tmp_path):
    store = SpatialMemoryStore(db_path=tmp_path / "m.db")
    cam = SyntheticCamera(label="person", noise=2.0)
    sess = PerceptionSession(cam, store, mode="production", source_id="lobby",
                             embed_fn=_fake_embed, hz=20)
    sess.start()
    try:
        time.sleep(1.0)
    finally:
        sess.stop()
    assert sess.indexed > 3
    # label search across history
    hits = search(store, "person", by="label")
    assert hits and hits[0]["detections"][0]["label"] == "person"
    assert hits[0]["source"] == "production" and hits[0]["source_id"] == "lobby"


def test_search_over_time_window(tmp_path):
    store = SpatialMemoryStore(db_path=tmp_path / "m.db")
    # plant observations at distinct past times (different colors/labels)
    store.store(embedding=np.array([200, 0, 0], np.float32),
                detections=[{"label": "person", "score": 1, "bbox": [0, 0, 1, 1]}],
                ts=1000.0, robot_id="r", source="robot")
    store.store(embedding=np.array([0, 0, 200], np.float32),
                detections=[{"label": "person", "score": 1, "bbox": [0, 0, 1, 1]}],
                ts=5000.0, robot_id="r", source="sim")

    # "who was seen after t=2000" → only the second
    recent = search(store, np.array([0, 0, 200], np.float32), by="clip", since=2000.0)
    assert len(recent) == 1 and recent[0]["ts"] == 5000.0
    # semantic search across ALL time returns the closest by color
    red = search(store, np.array([255, 0, 0], np.float32), by="clip", k=1)
    assert red[0]["ts"] == 1000.0


def test_record_path_writes_per_source_channels(tmp_path):
    from roborun.recorder import RunRecorder
    store = SpatialMemoryStore(db_path=tmp_path / "m.db")
    rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.05)
    cam = SyntheticCamera(label="car")
    sess = PerceptionSession(cam, store, mode="sim", source_id="front",
                             recorder=rec, embed_fn=_fake_embed, hz=20)
    cam.start()
    try:
        time.sleep(0.5)
        for _ in range(5):
            sess.tick()
    finally:
        cam.stop()
    seal = rec.close(do_anchor=False)
    mc = seal["message_counts"]
    assert mc.get("/camera/front") and mc.get("/detections/front")
    assert mc.get("/clip/embeddings")  # CLIP recorded for search


def test_search_route(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    import threading, json, urllib.request
    from contextlib import closing
    # seed a memory the route's store will read
    from roborun.routes._singletons import get_memory
    store = get_memory()
    store.store(detections=[{"label": "forklift", "score": 1, "bbox": [0, 0, 1, 1]}],
                ts=4242.0, robot_id="r", source="production")
    from roborun import server as srv
    from http.server import ThreadingHTTPServer
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), srv.Handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True); t.start()
    base = f"http://127.0.0.1:{httpd.server_address[1]}"
    try:
        req = urllib.request.Request(base + "/api/search",
            data=json.dumps({"query": "forklift", "by": "label"}).encode(),
            headers={"Content-Type": "application/json"})
        with closing(urllib.request.urlopen(req, timeout=5)) as r:
            d = json.loads(r.read())
        assert d["ok"] and d["results"][0]["detections"][0]["label"] == "forklift"
    finally:
        httpd.shutdown()
