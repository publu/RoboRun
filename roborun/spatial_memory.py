"""Spatial memory store — the hot query index over Observations.

Upgraded per the architecture spec (§3.2): same SQLite engine, fixed schema.

  * Detections live in a normalized child table with an index on `label`,
    so `search_yolo` is an indexed lookup, not a `LIKE '%…%'` table scan.
  * CLIP search stays an in-memory numpy cosine matmul (fine to ~1M vectors;
    an embedded ANN like sqlite-vec is the documented later step).
  * Spatial and time filters ride composite indexes.
  * Each row can carry a frame_ref (run_id + topic + log_time) pointing into
    the sealed MCAP run it was extracted from — the store is a derived,
    disposable index; the MCAP is the source of truth.

The store is rebuildable any time from cold MCAP via
`roborun.observations.extract_run`.

Usage:
    store = SpatialMemoryStore()
    store.store(frame, embedding, detections, x=1.5, y=3.2, robot_id="go2-01")
    results = store.search_clip(query_embedding, top_k=5)
    results = store.search_nearby(x=1.5, y=3.2, radius=2.0)
    results = store.search_yolo("person")
"""

from __future__ import annotations

import json
import os
import sqlite3
import time
import uuid
from pathlib import Path
from threading import RLock
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:  # numpy only backs the CLIP paths; the SQLite index runs without it
    import numpy as np

DB_DIR = Path(".roborun")
DB_PATH = DB_DIR / "spatial_memory.db"
THUMB_SIZE = (320, 240)
THUMB_QUALITY = 70

_SCHEMA = """
CREATE TABLE IF NOT EXISTS observations (
    id              TEXT PRIMARY KEY,
    robot_id        TEXT NOT NULL DEFAULT 'local',
    run_id          TEXT,
    ts              REAL NOT NULL,
    x               REAL,
    y               REAL,
    z               REAL,
    frame_id        TEXT,
    frame_topic     TEXT,
    frame_log_time  INTEGER,
    thumbnail       BLOB,
    embedding       BLOB,
    source          TEXT,
    source_id       TEXT,
    metadata        TEXT
);
CREATE TABLE IF NOT EXISTS detections (
    obs_id  TEXT NOT NULL,
    label   TEXT NOT NULL,
    score   REAL,
    x1 REAL, y1 REAL, x2 REAL, y2 REAL
);
CREATE INDEX IF NOT EXISTS idx_det_label    ON detections(label, obs_id);
CREATE INDEX IF NOT EXISTS idx_det_obs      ON detections(obs_id);
CREATE INDEX IF NOT EXISTS idx_obs_robot_ts ON observations(robot_id, ts DESC);
CREATE INDEX IF NOT EXISTS idx_obs_ts       ON observations(ts DESC);
CREATE INDEX IF NOT EXISTS idx_obs_xy       ON observations(x, y);
CREATE INDEX IF NOT EXISTS idx_obs_run      ON observations(run_id);
"""

_OBS_COLS = ("id, robot_id, run_id, ts, x, y, z, frame_id, frame_topic, "
             "frame_log_time, source, source_id, metadata")


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
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.executescript(_SCHEMA)
        self._migrate_v1()
        self._migrate_source_id()
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

    def _migrate_source_id(self) -> None:
        """Additively add the multi-camera source_id column to older dbs."""
        cols = {r[1] for r in self._conn.execute("PRAGMA table_info(observations)")}
        if "source_id" not in cols:
            self._conn.execute("ALTER TABLE observations ADD COLUMN source_id TEXT")
        # index created here (after the column is guaranteed to exist)
        self._conn.execute("CREATE INDEX IF NOT EXISTS idx_obs_source "
                           "ON observations(source_id, ts DESC)")

    def _migrate_v1(self) -> None:
        """Copy rows from the old `memories` table (detections as JSON) once."""
        tables = {r[0] for r in self._conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table'")}
        if "memories" not in tables:
            return
        if self._conn.execute("SELECT COUNT(*) FROM observations").fetchone()[0]:
            return
        rows = self._conn.execute(
            "SELECT id, robot_id, ts, x, y, z, thumbnail, embedding, detections, metadata "
            "FROM memories").fetchall()
        for r in rows:
            self._conn.execute(
                "INSERT OR IGNORE INTO observations "
                "(id, robot_id, ts, x, y, z, thumbnail, embedding, source, metadata) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, 'migrated-v1', ?)",
                (r["id"], r["robot_id"], r["ts"], r["x"], r["y"], r["z"],
                 r["thumbnail"], r["embedding"], r["metadata"]))
            for d in json.loads(r["detections"]) if r["detections"] else []:
                self._insert_detection(r["id"], d)

    def _insert_detection(self, obs_id: str, det: dict) -> None:
        bbox = det.get("bbox") or [None] * 4
        self._conn.execute(
            "INSERT INTO detections (obs_id, label, score, x1, y1, x2, y2) "
            "VALUES (?, ?, ?, ?, ?, ?, ?)",
            (obs_id, str(det.get("label", "")).lower(),
             det.get("score", det.get("confidence")),
             bbox[0], bbox[1], bbox[2], bbox[3]))

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
        ts: float | None = None,
        run_id: str | None = None,
        frame_topic: str | None = None,
        frame_log_time: int | None = None,
        source: str | None = None,
        source_id: str | None = None,
        thumbnail: bytes | None = None,
    ) -> str:
        mid = str(uuid.uuid4())[:12]
        thumb_blob = thumbnail
        if thumb_blob is None and frame is not None:
            import cv2
            small = cv2.resize(frame, THUMB_SIZE, interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, THUMB_QUALITY])
            if ok:
                thumb_blob = buf.tobytes()

        if embedding is not None:
            import numpy as np
            emb_blob = embedding.astype(np.float32).tobytes()
        else:
            emb_blob = None
        meta_json = json.dumps(metadata) if metadata else None

        s3_stored = False
        if thumb_blob and self._s3 and self._s3_bucket:
            try:
                key = f"{self._s3_prefix}{mid}.jpg"
                self._s3.put_object(Bucket=self._s3_bucket, Key=key,
                                    Body=thumb_blob, ContentType="image/jpeg")
                s3_stored = True
            except Exception:
                pass

        with self._lock:
            self._conn.execute(
                "INSERT INTO observations (id, robot_id, run_id, ts, x, y, z, "
                "frame_id, frame_topic, frame_log_time, thumbnail, embedding, source, source_id, metadata) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (mid, robot_id, run_id, ts or time.time(), x, y, z,
                 None, frame_topic, frame_log_time,
                 None if s3_stored else thumb_blob,
                 emb_blob, source, source_id, meta_json),
            )
            for d in detections or []:
                self._insert_detection(mid, d)
            self._conn.commit()
            self._cache_dirty = True
        return mid

    # ── CLIP: numpy cosine by default, sqlite-vec ANN past a threshold ────

    def _ann_available(self) -> bool:
        try:
            import sqlite_vec  # noqa: F401
            return True
        except Exception:
            return False

    def _ann_threshold(self) -> int:
        return int(os.environ.get("ROBORUN_ANN_THRESHOLD", "100000"))

    def _ensure_vec_table(self, dim: int) -> bool:
        """(Re)build the vec0 ANN index from observations (normalized → L2 on
        unit vectors is monotonic with cosine). Returns True if usable."""
        import numpy as np
        import sqlite_vec
        try:
            self._conn.enable_load_extension(True)
            sqlite_vec.load(self._conn)
            self._conn.enable_load_extension(False)
        except Exception:
            return False
        if getattr(self, "_vec_dim", None) != dim:
            self._conn.execute("DROP TABLE IF EXISTS vec_obs")
            self._conn.execute(
                f"CREATE VIRTUAL TABLE vec_obs USING vec0(obs_id TEXT, emb float[{dim}])")
            self._vec_dim = dim
            self._vec_built = False
        if not getattr(self, "_vec_built", False) or self._cache_dirty:
            self._conn.execute("DELETE FROM vec_obs")
            rows = self._conn.execute(
                "SELECT id, embedding FROM observations WHERE embedding IS NOT NULL"
            ).fetchall()
            for r in rows:
                v = np.frombuffer(r["embedding"], dtype=np.float32)
                if v.shape[0] != dim:
                    continue
                n = np.linalg.norm(v) or 1.0
                self._conn.execute("INSERT INTO vec_obs(obs_id, emb) VALUES (?, ?)",
                                   (r["id"], (v / n).astype(np.float32).tobytes()))
            self._conn.commit()
            self._vec_built = True
        return True

    def _search_clip_ann(self, qvec, top_k: int, robot_id: str | None) -> list[dict]:
        import numpy as np
        q = qvec.astype(np.float32).flatten()
        dim = q.shape[0]
        if not self._ensure_vec_table(dim):
            raise RuntimeError("vec table unavailable")
        n = np.linalg.norm(q) or 1.0
        q = (q / n).astype(np.float32)
        hits = self._conn.execute(
            "SELECT obs_id, distance FROM vec_obs WHERE emb MATCH ? "
            "ORDER BY distance LIMIT ?", (q.tobytes(), top_k * 3)).fetchall()
        ids = [h["obs_id"] for h in hits]
        rows = self._fetch_rows(ids)
        dets = self._fetch_detections(ids)
        out = []
        for h in hits:
            row = rows.get(h["obs_id"])
            if row is None or (robot_id and row["robot_id"] != robot_id):
                continue
            # L2 on unit vectors d² = 2(1-cos) → cos = 1 - d²/2
            out.append(self._row_to_dict(row, dets.get(row["id"], []),
                                         score=float(1.0 - h["distance"] ** 2 / 2)))
            if len(out) >= top_k:
                break
        return out

    def _rebuild_cache(self) -> None:
        rows = self._conn.execute(
            "SELECT id, embedding FROM observations WHERE embedding IS NOT NULL ORDER BY ts"
        ).fetchall()
        if not rows:
            self._emb_cache = None
            self._id_cache = []
            self._cache_dirty = False
            return
        import numpy as np
        ids, vecs = [], []
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
            # sqlite-vec ANN past the threshold; numpy is the guaranteed fallback.
            count = self._conn.execute(
                "SELECT COUNT(*) FROM observations WHERE embedding IS NOT NULL"
            ).fetchone()[0]
            if count >= self._ann_threshold() and self._ann_available():
                try:
                    return self._search_clip_ann(query_embedding, top_k, robot_id)
                except Exception:
                    pass  # fall through to numpy

            if self._cache_dirty:
                self._rebuild_cache()
            if self._emb_cache is None or len(self._id_cache) == 0:
                return []

            import numpy as np
            qvec = query_embedding.astype(np.float32).flatten()
            qnorm = np.linalg.norm(qvec)
            if qnorm > 0:
                qvec = qvec / qnorm

            scores = self._emb_cache @ qvec
            top_idx = np.argsort(scores)[::-1][:top_k * 3]
            candidates = [self._id_cache[i] for i in top_idx]
            rows = self._fetch_rows(candidates)
            dets = self._fetch_detections(candidates)

            results = []
            for i in top_idx:
                row = rows.get(self._id_cache[i])
                if row is None:
                    continue
                if robot_id and row["robot_id"] != robot_id:
                    continue
                results.append(self._row_to_dict(
                    row, dets.get(row["id"], []), score=float(scores[i])))
                if len(results) >= top_k:
                    break
            return results

    # ── indexed label search (the LIKE-scan fix) ──────────────────────────

    def search_yolo(
        self, label: str, top_k: int = 20, robot_id: str | None = None
    ) -> list[dict]:
        with self._lock:
            q = label.lower().strip()
            params: list[Any] = [q]
            where = "d.label = ?"
            if robot_id:
                where += " AND o.robot_id = ?"
                params.append(robot_id)
            sql = (f"SELECT DISTINCT {', '.join('o.' + c.strip() for c in _OBS_COLS.split(','))} "
                   f"FROM observations o JOIN detections d ON d.obs_id = o.id "
                   f"WHERE {where} ORDER BY o.ts DESC LIMIT ?")
            rows = self._conn.execute(sql, params + [top_k]).fetchall()
            if not rows:
                # substring fallback — still over the small detections table
                params[0] = f"%{q}%"
                rows = self._conn.execute(
                    sql.replace("d.label = ?", "d.label LIKE ?"),
                    params + [top_k]).fetchall()
            ids = [r["id"] for r in rows]
            dets = self._fetch_detections(ids)
            results = []
            for r in rows:
                all_dets = dets.get(r["id"], [])
                matching = [d for d in all_dets if q in d["label"]]
                results.append(self._row_to_dict(r, all_dets,
                                                 matching_detections=matching))
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
            where = ["x IS NOT NULL", "y IS NOT NULL",
                     "x BETWEEN ? AND ?", "y BETWEEN ? AND ?",
                     "(x - ?) * (x - ?) + (y - ?) * (y - ?) <= ? * ?"]
            params.extend([x - radius, x + radius, y - radius, y + radius,
                           x, x, y, y, radius, radius])
            if z is not None:
                where.append("ABS(COALESCE(z, 0) - ?) <= ?")
                params.extend([z, radius])
            if robot_id:
                where.append("robot_id = ?")
                params.append(robot_id)
            sql = (f"SELECT {_OBS_COLS} FROM observations WHERE {' AND '.join(where)} "
                   f"ORDER BY ts DESC LIMIT ?")
            params.append(top_k)
            rows = self._conn.execute(sql, params).fetchall()
            dets = self._fetch_detections([r["id"] for r in rows])
            return [self._row_to_dict(r, dets.get(r["id"], []),
                                      distance=self._dist(r, x, y, z)) for r in rows]

    # ── time-range search ─────────────────────────────────────────────────
    def search_time(self, since: float | None = None, until: float | None = None,
                    top_k: int = 20, robot_id: str | None = None) -> list[dict]:
        with self._lock:
            where, params = ["1=1"], []  # type: ignore[var-annotated]
            if since is not None:
                where.append("ts >= ?"); params.append(since)
            if until is not None:
                where.append("ts <= ?"); params.append(until)
            if robot_id:
                where.append("robot_id = ?"); params.append(robot_id)
            sql = (f"SELECT {_OBS_COLS} FROM observations WHERE {' AND '.join(where)} "
                   f"ORDER BY ts DESC LIMIT ?")
            params.append(top_k)
            rows = self._conn.execute(sql, params).fetchall()
            dets = self._fetch_detections([r["id"] for r in rows])
            return [self._row_to_dict(r, dets.get(r["id"], [])) for r in rows]

    # ── unified retrieval: "RAG for everything" (PERCEPTION_DATA_SPEC) ──────
    def recall(self, query: Any = None, by: str = "clip", k: int = 10,
               robot_id: str | None = None, source_id: str | None = None,
               **kw: Any) -> list[dict]:
        """One entry over every index the agent/behaviors call:
          by="clip"  → CLIP semantic (query: text str or np.ndarray embedding)
          by="label" → indexed YOLO label (query: str)
          by="near"  → spatial (kw: x, y[, z, radius])
          by="time"  → time range (kw: since, until — unix seconds)
        `source_id` filters to one camera angle. Returns Observation dicts
        (frame_ref → pull full frame from MCAP)."""
        by = (by or "clip").lower()
        # over-fetch when filtering by camera so k results survive the filter
        kk = k * 4 if source_id else k
        if by == "label":
            rows = self.search_yolo(str(query), top_k=kk, robot_id=robot_id)
        elif by == "near":
            rows = self.search_nearby(float(kw["x"]), float(kw["y"]),
                                      kw.get("z"), float(kw.get("radius", 2.0)),
                                      top_k=kk, robot_id=robot_id)
        elif by == "time":
            rows = self.search_time(kw.get("since"), kw.get("until"),
                                    top_k=kk, robot_id=robot_id)
        elif by == "clip":
            import numpy as np
            emb = query
            if not isinstance(emb, np.ndarray):
                from roborun.models import CLIPMatcher
                emb = CLIPMatcher().embed_text(str(query))
            rows = self.search_clip(emb, top_k=kk, robot_id=robot_id)
        else:
            raise ValueError(f"unknown recall mode {by!r} (clip|label|near|time)")
        if source_id:
            rows = [r for r in rows if r.get("source_id") == source_id][:k]
        return rows

    def list_memories(
        self, limit: int = 50, robot_id: str | None = None, since: float | None = None,
        source: str | None = None, run_id: str | None = None,
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
                where_parts.append("source = ?")
                params.append(source)
            if run_id:
                where_parts.append("run_id = ?")
                params.append(run_id)
            where = f"WHERE {' AND '.join(where_parts)}" if where_parts else ""
            params.append(limit)
            rows = self._conn.execute(
                f"SELECT {_OBS_COLS} FROM observations {where} ORDER BY ts DESC LIMIT ?",
                params,
            ).fetchall()
            dets = self._fetch_detections([r["id"] for r in rows])
            return [self._row_to_dict(r, dets.get(r["id"], [])) for r in rows]

    def get_thumbnail(self, memory_id: str) -> bytes | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT thumbnail FROM observations WHERE id = ?", (memory_id,)).fetchone()
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

    def delete(self, memory_id: str) -> bool:
        with self._lock:
            cur = self._conn.execute("DELETE FROM observations WHERE id = ?", (memory_id,))
            self._conn.execute("DELETE FROM detections WHERE obs_id = ?", (memory_id,))
            self._conn.commit()
            if cur.rowcount > 0:
                self._cache_dirty = True
                return True
            return False

    def stats(self) -> dict:
        with self._lock:
            total = self._conn.execute("SELECT COUNT(*) FROM observations").fetchone()[0]
            with_emb = self._conn.execute(
                "SELECT COUNT(*) FROM observations WHERE embedding IS NOT NULL").fetchone()[0]
            with_pos = self._conn.execute(
                "SELECT COUNT(*) FROM observations WHERE x IS NOT NULL").fetchone()[0]
            det_count = self._conn.execute("SELECT COUNT(*) FROM detections").fetchone()[0]
            robots = [r[0] for r in self._conn.execute(
                "SELECT DISTINCT robot_id FROM observations").fetchall()]
            runs = self._conn.execute(
                "SELECT COUNT(DISTINCT run_id) FROM observations WHERE run_id IS NOT NULL"
            ).fetchone()[0]
            return {"total": total, "with_embeddings": with_emb,
                    "with_position": with_pos, "detections": det_count,
                    "robots": robots, "runs": runs, "db": str(self._db_path)}

    # ── export hooks (see roborun.observations) ───────────────────────────

    def iter_export_rows(self, robot_id: str | None = None,
                         since: float | None = None,
                         include_embeddings: bool = True):
        """Yield flat (observation × detection) rows for Parquet export."""
        with self._lock:
            params: list[Any] = []
            where = []
            if robot_id:
                where.append("o.robot_id = ?")
                params.append(robot_id)
            if since:
                where.append("o.ts > ?")
                params.append(since)
            clause = f"WHERE {' AND '.join(where)}" if where else ""
            rows = self._conn.execute(
                f"SELECT o.id, o.robot_id, o.run_id, o.ts, o.x, o.y, o.z, "
                f"o.frame_topic, o.frame_log_time, o.source, o.embedding, "
                f"d.label, d.score "
                f"FROM observations o LEFT JOIN detections d ON d.obs_id = o.id "
                f"{clause} ORDER BY o.ts", params).fetchall()
        for r in rows:
            emb = None
            if include_embeddings and r["embedding"]:
                import numpy as np
                emb = np.frombuffer(r["embedding"], dtype=np.float32).tolist()
            yield {
                "obs_id": r["id"], "robot_id": r["robot_id"], "run_id": r["run_id"],
                "ts": r["ts"], "x": r["x"], "y": r["y"], "z": r["z"],
                "frame_topic": r["frame_topic"], "frame_log_time": r["frame_log_time"],
                "source": r["source"], "label": r["label"], "score": r["score"],
                "embedding": emb,
            }

    # ── helpers ───────────────────────────────────────────────────────────

    def _fetch_rows(self, ids: list[str]) -> dict[str, sqlite3.Row]:
        if not ids:
            return {}
        ph = ",".join("?" * len(ids))
        rows = self._conn.execute(
            f"SELECT {_OBS_COLS} FROM observations WHERE id IN ({ph})", ids).fetchall()
        return {r["id"]: r for r in rows}

    def _fetch_detections(self, ids: list[str]) -> dict[str, list[dict]]:
        if not ids:
            return {}
        ph = ",".join("?" * len(ids))
        rows = self._conn.execute(
            f"SELECT obs_id, label, score, x1, y1, x2, y2 "
            f"FROM detections WHERE obs_id IN ({ph})", ids).fetchall()
        out: dict[str, list[dict]] = {}
        for r in rows:
            det = {"label": r["label"], "score": r["score"]}
            if r["x1"] is not None:
                det["bbox"] = [r["x1"], r["y1"], r["x2"], r["y2"]]
            out.setdefault(r["obs_id"], []).append(det)
        return out

    @staticmethod
    def _row_to_dict(row: sqlite3.Row, detections: list[dict], **extra: Any) -> dict:
        d = {
            "id": row["id"],
            "robot_id": row["robot_id"],
            "run_id": row["run_id"],
            "ts": row["ts"],
            "x": row["x"],
            "y": row["y"],
            "z": row["z"],
            "detections": detections,
            "frame_ref": ({"run_id": row["run_id"], "topic": row["frame_topic"],
                           "log_time": row["frame_log_time"]}
                          if row["frame_topic"] else None),
            "source": row["source"],
            "source_id": (row["source_id"] if "source_id" in row.keys() else None),
            "metadata": json.loads(row["metadata"]) if row["metadata"] else {},
        }
        d.update(extra)
        return d

    @staticmethod
    def _dist(row: sqlite3.Row, x: float, y: float, z: float | None) -> float:
        dx = (row["x"] or 0) - x
        dy = (row["y"] or 0) - y
        dz = ((row["z"] or 0) - z) if z is not None else 0
        return float((dx * dx + dy * dy + dz * dz) ** 0.5)
