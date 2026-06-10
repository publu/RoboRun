"""Observation store + extractor tests — indexed search, MCAP derivation, fleet."""
import sqlite3
import time

import numpy as np
import pytest

from roborun.recorder import RunRecorder
from roborun.spatial_memory import SpatialMemoryStore
from roborun.observations import extract_run, get_frame


@pytest.fixture
def store(tmp_path):
    return SpatialMemoryStore(db_path=tmp_path / "mem.db")


def test_yolo_search_is_indexed_lookup(store):
    for i in range(30):
        store.store(detections=[{"label": "Person" if i % 3 else "forklift",
                                 "score": 0.8, "bbox": [0, 0, 1, 1]}],
                    robot_id="r1", ts=time.time() + i)
    hits = store.search_yolo("forklift")
    assert len(hits) == 10
    assert all(d["label"] == "forklift" for h in hits for d in h["matching_detections"])
    # case-insensitive exact match via normalized labels
    assert len(store.search_yolo("PERSON")) == 20
    # substring fallback still works
    assert len(store.search_yolo("fork")) == 10


def test_detections_table_normalized(store):
    store.store(detections=[{"label": "cup", "score": 0.5, "bbox": [1, 2, 3, 4]},
                            {"label": "mug", "score": 0.6, "bbox": [5, 6, 7, 8]}])
    conn = sqlite3.connect(store._db_path)
    rows = conn.execute("SELECT label, score FROM detections ORDER BY label").fetchall()
    assert rows == [("cup", 0.5), ("mug", 0.6)]
    # the label index exists
    plan = conn.execute(
        "EXPLAIN QUERY PLAN SELECT * FROM detections WHERE label = 'cup'").fetchall()
    assert any("idx_det_label" in str(r) for r in plan)


def test_clip_and_nearby(store):
    base = np.zeros(512, dtype=np.float32); base[0] = 1.0
    other = np.zeros(512, dtype=np.float32); other[1] = 1.0
    store.store(embedding=base, x=0.0, y=0.0)
    store.store(embedding=other, x=10.0, y=10.0)
    hits = store.search_clip(base, top_k=1)
    assert len(hits) == 1 and hits[0]["score"] > 0.99
    near = store.search_nearby(0.0, 0.0, radius=1.0)
    assert len(near) == 1


def test_v1_migration(tmp_path):
    import json
    db = tmp_path / "old.db"
    conn = sqlite3.connect(db)
    conn.execute("""CREATE TABLE memories (
        id TEXT PRIMARY KEY, robot_id TEXT NOT NULL DEFAULT 'local',
        ts REAL NOT NULL, x REAL, y REAL, z REAL,
        thumbnail BLOB, embedding BLOB, detections TEXT, metadata TEXT)""")
    conn.execute("INSERT INTO memories VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                 ("old1", "go2", 123.0, 1, 2, 3, None,
                  np.ones(4, dtype=np.float32).tobytes(),
                  json.dumps([{"label": "Dog", "score": 0.7, "bbox": [0, 0, 1, 1]}]),
                  None))
    conn.commit(); conn.close()
    store = SpatialMemoryStore(db_path=db)
    hits = store.search_yolo("dog")
    assert len(hits) == 1 and hits[0]["id"] == "old1"


def test_extract_run_joins_channels(tmp_path):
    rec = RunRecorder(robot_id="go2", root=tmp_path / "runs", checkpoint_interval=0.01)
    t0 = time.time()
    for i in range(10):
        ts = t0 + i
        rec.write_camera(b"\xff\xd8" + bytes([i]) * 32, ts=ts)
        rec.write_detections([{"label": "chair", "score": 0.9, "bbox": [0, 0, 1, 1]}], ts=ts)
        vec = np.zeros(8, dtype=np.float32); vec[i % 8] = 1.0
        rec.write_clip(vec, ts=ts)
        rec.write_pose(float(i), float(i * 2), ts=ts)
    rec.close(do_anchor=False)

    store = SpatialMemoryStore(db_path=tmp_path / "mem.db")
    result = extract_run(rec.mcap_path, store, thumbnails=False)
    assert result["observations"] == 10

    hits = store.search_yolo("chair")
    assert len(hits) == 10
    h = hits[0]
    assert h["run_id"] == rec.run_id
    assert h["frame_ref"]["topic"] == "/camera/webcam"
    assert h["x"] is not None  # pose joined on
    # frame_ref resolves back into the sealed MCAP
    frame = get_frame(rec.mcap_path, h["frame_ref"]["topic"], h["frame_ref"]["log_time"])
    assert frame is not None and frame.startswith(b"\xff\xd8")
    # the hot index is derived: rebuilding from cold MCAP works
    store2 = SpatialMemoryStore(db_path=tmp_path / "mem2.db")
    assert extract_run(rec.mcap_path, store2, thumbnails=False)["observations"] == 10


def test_parquet_export_and_fleet_query(tmp_path):
    duckdb = pytest.importorskip("duckdb")
    from roborun.observations import export_parquet, fleet_query
    store = SpatialMemoryStore(db_path=tmp_path / "mem.db")
    for robot in ("go2-01", "go2-02"):
        for i in range(5):
            store.store(detections=[{"label": "forklift", "score": 0.9, "bbox": [0, 0, 1, 1]}],
                        robot_id=robot, x=float(i), y=0.0,
                        embedding=np.random.rand(16).astype(np.float32))
    out = export_parquet(store, robot_id="go2-01", out_root=tmp_path / "index")
    assert out["ok"] and out["rows"] == 10
    fq = fleet_query("SELECT robot_id, count(*) AS n FROM fleet "
                     "WHERE label = 'forklift' GROUP BY robot_id ORDER BY robot_id",
                     cache_dir=tmp_path / "index", sync_r2=False)
    assert fq["ok"]
    assert [(r["robot_id"], r["n"]) for r in fq["rows"]] == [("go2-01", 5), ("go2-02", 5)]


def test_fleet_clip_search(tmp_path):
    pytest.importorskip("duckdb")
    from roborun.observations import export_parquet, fleet_search_clip
    store = SpatialMemoryStore(db_path=tmp_path / "mem.db")
    target = np.zeros(16, dtype=np.float32); target[0] = 1.0
    store.store(embedding=target, robot_id="a",
                detections=[{"label": "mug", "score": 1.0, "bbox": [0, 0, 1, 1]}])
    store.store(embedding=np.random.rand(16).astype(np.float32) * 0.1, robot_id="b")
    export_parquet(store, robot_id="a", out_root=tmp_path / "index")
    r = fleet_search_clip(target, top_k=1, cache_dir=tmp_path / "index")
    assert r["ok"] and r["rows"][0]["robot_id"] == "a"
