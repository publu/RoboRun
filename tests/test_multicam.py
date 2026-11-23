"""Multi-camera source_id: additive column, migration, and recall filter."""
from __future__ import annotations

import sqlite3

import numpy as np

from roborun.spatial_memory import SpatialMemoryStore


def test_source_id_round_trips(tmp_path):
    s = SpatialMemoryStore(db_path=tmp_path / "m.db")
    s.store(embedding=np.array([1, 0], np.float32),
            detections=[{"label": "mug", "score": 1.0, "bbox": [0, 0, 1, 1]}],
            source_id="front", ts=1.0)
    s.store(embedding=np.array([0, 1], np.float32),
            detections=[{"label": "mug", "score": 1.0, "bbox": [0, 0, 1, 1]}],
            source_id="rear", ts=2.0)
    front = s.recall("mug", by="label", source_id="front")
    assert len(front) == 1 and front[0]["source_id"] == "front"
    assert {r["source_id"] for r in s.recall("mug", by="label")} == {"front", "rear"}


def test_migration_adds_column_to_old_db(tmp_path):
    # build a db WITHOUT source_id (simulate an older schema), then open with the store
    p = tmp_path / "old.db"
    con = sqlite3.connect(p)
    con.execute("CREATE TABLE observations (id TEXT PRIMARY KEY, robot_id TEXT, "
                "run_id TEXT, ts REAL, x REAL, y REAL, z REAL, frame_id TEXT, "
                "frame_topic TEXT, frame_log_time INTEGER, thumbnail BLOB, "
                "embedding BLOB, source TEXT, metadata TEXT)")
    con.execute("INSERT INTO observations (id, robot_id, ts) VALUES ('a','r',1.0)")
    con.commit(); con.close()

    s = SpatialMemoryStore(db_path=p)  # migration runs in __init__
    cols = {r[1] for r in s._conn.execute("PRAGMA table_info(observations)")}
    assert "source_id" in cols
    # existing row still queryable, source_id null
    hit = s.recall(by="time", since=0.0)
    assert hit and hit[0]["source_id"] is None
