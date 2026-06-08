"""Spatial memory store — CLIP-searchable, geo-indexed, multi-robot.

Stores frames (as JPEG thumbnails), CLIP embeddings, YOLO detections,
and x/y/z coordinates in a single SQLite database. Embeddings are cached
in a numpy matrix for fast cosine similarity search.

Thumbnails can optionally be synced to an S3-compatible bucket (AWS S3,
MinIO, R2, etc.) while the search index always stays local for speed.

Usage:
    store = SpatialMemoryStore()
    store.store(frame, embedding, detections, x=1.5, y=3.2, z=0, robot_id="go2-01")
    results = store.search_clip("red mug", clip_model, top_k=5)
    results = store.search_nearby(x=1.5, y=3.2, radius=2.0)
    results = store.search_yolo("person")

    # With S3 backend for thumbnails:
    store = SpatialMemoryStore(s3_bucket="my-bucket", s3_prefix="roborun/")
"""

from __future__ import annotations

import json
import sqlite3
import time
import uuid
from pathlib import Path
from threading import RLock
from typing import Any

import cv2
import numpy as np


DB_DIR = Path(".roborun")
DB_PATH = DB_DIR / "spatial_memory.db"
THUMB_SIZE = (320, 240)
THUMB_QUALITY = 70

_SCHEMA = """
CREATE TABLE IF NOT EXISTS memories (
    id          TEXT PRIMARY KEY,
    robot_id    TEXT NOT NULL DEFAULT 'local',
    ts          REAL NOT NULL,
    x           REAL,
    y           REAL,
    z           REAL,
    thumbnail   BLOB,
    embedding   BLOB,
    detections  TEXT,
    metadata    TEXT
);
CREATE INDEX IF NOT EXISTS idx_memories_robot ON memories(robot_id);
CREATE INDEX IF NOT EXISTS idx_memories_ts ON memories(ts DESC);
CREATE INDEX IF NOT EXISTS idx_memories_xyz ON memories(x, y, z);
"""


class SpatialMemoryStore:

    def __init__(
        self,
        db_path: str | Path | None = None,
        s3_bucket: str | None = None,
        s3_prefix: str = "roborun/memories/",
        s3_endpoint: str | None = None,
    ) -> None:
        self._db_path = Path(db_path) if db_path else DB_PATH
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = RLock()
        self._conn = sqlite3.connect(str(self._db_path), check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._conn.executescript(_SCHEMA)
        self._conn.commit()
        self._emb_cache: np.ndarray | None = None
        self._id_cache: list[str] = []
        self._cache_dirty = True

        self._s3 = None
        self._s3_bucket = s3_bucket
        self._s3_prefix = s3_prefix
        if s3_bucket:
            try:
                import boto3
                kwargs: dict[str, Any] = {}
                if s3_endpoint:
                    kwargs["endpoint_url"] = s3_endpoint
                self._s3 = boto3.client("s3", **kwargs)
            except ImportError:
                pass

    def store(
        self,
        frame: np.ndarray | None = None,
        embedding: np.ndarray | None = None,
        detections: list[dict] | None = None,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        robot_id: str = "local",
        metadata: dict | None = None,
    ) -> str:
        mid = str(uuid.uuid4())[:12]
        thumb_blob = None
        if frame is not None:
            small = cv2.resize(frame, THUMB_SIZE, interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, THUMB_QUALITY])
            if ok:
                thumb_blob = buf.tobytes()

        emb_blob = None
        if embedding is not None:
            emb_blob = embedding.astype(np.float32).tobytes()

        det_json = json.dumps(detections) if detections else None
        meta_json = json.dumps(metadata) if metadata else None

        s3_stored = False
        if thumb_blob and self._s3 and self._s3_bucket:
            try:
                key = f"{self._s3_prefix}{mid}.jpg"
                self._s3.put_object(Bucket=self._s3_bucket, Key=key, Body=thumb_blob, ContentType="image/jpeg")
                s3_stored = True
            except Exception:
                pass

        with self._lock:
            self._conn.execute(
                "INSERT INTO memories (id, robot_id, ts, x, y, z, thumbnail, embedding, detections, metadata) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (mid, robot_id, time.time(), x, y, z,
                 None if s3_stored else thumb_blob,
                 emb_blob, det_json, meta_json),
            )
            self._conn.commit()
            self._cache_dirty = True
        return mid

    def _rebuild_cache(self) -> None:
        rows = self._conn.execute(
            "SELECT id, embedding FROM memories WHERE embedding IS NOT NULL ORDER BY ts"
        ).fetchall()
        if not rows:
            self._emb_cache = None
            self._id_cache = []
            self._cache_dirty = False
            return
        ids = []
        vecs = []
        for r in rows:
            ids.append(r["id"])
            vecs.append(np.frombuffer(r["embedding"], dtype=np.float32))
        self._id_cache = ids
        self._emb_cache = np.vstack(vecs)
        norms = np.linalg.norm(self._emb_cache, axis=1, keepdims=True)
        norms[norms == 0] = 1
        self._emb_cache = self._emb_cache / norms
        self._cache_dirty = False

    def search_clip(
        self, query_embedding: np.ndarray, top_k: int = 10, robot_id: str | None = None
    ) -> list[dict]:
        with self._lock:
            if self._cache_dirty:
                self._rebuild_cache()
            if self._emb_cache is None or len(self._id_cache) == 0:
                return []

            qvec = query_embedding.astype(np.float32).flatten()
            qnorm = np.linalg.norm(qvec)
            if qnorm > 0:
                qvec = qvec / qnorm

            scores = self._emb_cache @ qvec
            top_idx = np.argsort(scores)[::-1][:top_k * 3]

            results = []
            for idx in top_idx:
                mid = self._id_cache[idx]
                row = self._conn.execute(
                    "SELECT id, robot_id, ts, x, y, z, detections, metadata FROM memories WHERE id = ?",
                    (mid,),
                ).fetchone()
                if row is None:
                    continue
                if robot_id and row["robot_id"] != robot_id:
                    continue
                results.append(self._row_to_dict(row, score=float(scores[idx])))
                if len(results) >= top_k:
                    break
            return results

    def search_nearby(
        self,
        x: float,
        y: float,
        z: float | None = None,
        radius: float = 2.0,
        top_k: int = 20,
        robot_id: str | None = None,
    ) -> list[dict]:
        with self._lock:
            params: list[Any] = []
            where = ["x IS NOT NULL", "y IS NOT NULL"]
            where.append(f"(x - ?) * (x - ?) + (y - ?) * (y - ?) <= ? * ?")
            params.extend([x, x, y, y, radius, radius])
            if z is not None:
                where.append("ABS(COALESCE(z, 0) - ?) <= ?")
                params.extend([z, radius])
            if robot_id:
                where.append("robot_id = ?")
                params.append(robot_id)
            sql = f"SELECT id, robot_id, ts, x, y, z, detections, metadata FROM memories WHERE {' AND '.join(where)} ORDER BY ts DESC LIMIT ?"
            params.append(top_k)
            rows = self._conn.execute(sql, params).fetchall()
            return [self._row_to_dict(r, distance=self._dist(r, x, y, z)) for r in rows]

    def search_yolo(
        self, label: str, top_k: int = 20, robot_id: str | None = None
    ) -> list[dict]:
        with self._lock:
            pattern = f'%"{label}"%'
            params: list[Any] = [pattern]
            where = "detections LIKE ?"
            if robot_id:
                where += " AND robot_id = ?"
                params.append(robot_id)
            params.append(top_k)
            rows = self._conn.execute(
                f"SELECT id, robot_id, ts, x, y, z, detections, metadata FROM memories WHERE {where} ORDER BY ts DESC LIMIT ?",
                params,
            ).fetchall()
            results = []
            for r in rows:
                dets = json.loads(r["detections"]) if r["detections"] else []
                matching = [d for d in dets if label.lower() in d.get("label", "").lower()]
                if matching:
                    results.append(self._row_to_dict(r, matching_detections=matching))
            return results

    def get_thumbnail(self, memory_id: str) -> bytes | None:
        with self._lock:
            row = self._conn.execute("SELECT thumbnail FROM memories WHERE id = ?", (memory_id,)).fetchone()
            if not row:
                return None
            if row["thumbnail"]:
                return row["thumbnail"]
        if self._s3 and self._s3_bucket:
            try:
                key = f"{self._s3_prefix}{memory_id}.jpg"
                resp = self._s3.get_object(Bucket=self._s3_bucket, Key=key)
                return resp["Body"].read()
            except Exception:
                pass
        return None

    def list_memories(
        self, limit: int = 50, robot_id: str | None = None, since: float | None = None,
        source: str | None = None,
    ) -> list[dict]:
        with self._lock:
            params: list[Any] = []
            where_parts = []
            if robot_id:
                where_parts.append("robot_id = ?")
                params.append(robot_id)
            if since:
                where_parts.append("ts > ?")
                params.append(since)
            if source:
                where_parts.append("json_extract(metadata, '$.source') = ?")
                params.append(source)
            where = f"WHERE {' AND '.join(where_parts)}" if where_parts else ""
            params.append(limit)
            rows = self._conn.execute(
                f"SELECT id, robot_id, ts, x, y, z, detections, metadata FROM memories {where} ORDER BY ts DESC LIMIT ?",
                params,
            ).fetchall()
            return [self._row_to_dict(r) for r in rows]

    def delete(self, memory_id: str) -> bool:
        with self._lock:
            cur = self._conn.execute("DELETE FROM memories WHERE id = ?", (memory_id,))
            self._conn.commit()
            if cur.rowcount > 0:
                self._cache_dirty = True
                return True
            return False

    def stats(self) -> dict:
        with self._lock:
            total = self._conn.execute("SELECT COUNT(*) FROM memories").fetchone()[0]
            with_emb = self._conn.execute("SELECT COUNT(*) FROM memories WHERE embedding IS NOT NULL").fetchone()[0]
            with_pos = self._conn.execute("SELECT COUNT(*) FROM memories WHERE x IS NOT NULL").fetchone()[0]
            robots = [r[0] for r in self._conn.execute("SELECT DISTINCT robot_id FROM memories").fetchall()]
            return {"total": total, "with_embeddings": with_emb, "with_position": with_pos, "robots": robots}

    @staticmethod
    def _row_to_dict(row: sqlite3.Row, **extra: Any) -> dict:
        d = {
            "id": row["id"],
            "robot_id": row["robot_id"],
            "ts": row["ts"],
            "x": row["x"],
            "y": row["y"],
            "z": row["z"],
            "detections": json.loads(row["detections"]) if row["detections"] else [],
            "metadata": json.loads(row["metadata"]) if row["metadata"] else {},
        }
        d.update(extra)
        return d

    @staticmethod
    def _dist(row: sqlite3.Row, x: float, y: float, z: float | None) -> float:
        dx = (row["x"] or 0) - x
        dy = (row["y"] or 0) - y
        dz = ((row["z"] or 0) - z) if z is not None else 0
        return float(np.sqrt(dx * dx + dy * dy + dz * dz))
