"""Observations — derive the hot index from cold MCAP, export it, query the fleet.

The Observation is the join key across all storage tiers (spec §3.2):

    Observation { obs_id, robot_id, run_id, ts, pose, detections[],
                  clip_embedding, frame_ref{topic, log_time}, source }

This module owns the derivations:

  * extract_run(mcap, store)  — stream a sealed run's MCAP into the SQLite hot
    store (roborun.spatial_memory). The index is derived and disposable; the
    MCAP is the source of truth and the index can be rebuilt from it any time.
  * export_parquet(store, …)  — flatten Observation rows to Parquet, the shared
    fleet index layout: index/<robot_id>/<date>.parquet (embedded DuckDB writes
    the file; no service).
  * fleet_query(sql)          — DuckDB over the union of every robot's Parquet
    exports. With R2 configured the shared prefix is synced to a local cache
    first, so queries never need a DuckDB network extension.

No brokers, no database servers — every engine here is a library reading files.
"""
from __future__ import annotations

import base64
import bisect
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# How far apart (seconds) a detection/embedding/pose may be from the camera
# frame it gets joined onto.
JOIN_TOLERANCE = 0.75


def _decode_messages(mcap_path: Path) -> dict[str, list[tuple[float, str, dict]]]:
    """Read every JSON message, grouped by kind: camera/detections/clip/pose/other."""
    from mcap.reader import make_reader

    groups: dict[str, list[tuple[float, str, dict]]] = {
        "camera": [], "detections": [], "clip": [], "pose": [], "events": [],
    }
    with open(mcap_path, "rb") as fh:
        reader = make_reader(fh)
        for _schema, channel, message in reader.iter_messages():
            if channel.message_encoding != "json":
                continue
            try:
                obj = json.loads(message.data)
            except Exception:
                continue
            ts = message.log_time / 1e9
            t = channel.topic
            if t.startswith("/camera/"):
                groups["camera"].append((ts, t, obj))
            elif t.startswith("/detections/"):
                groups["detections"].append((ts, t, obj))
            elif t == "/clip/embeddings":
                groups["clip"].append((ts, t, obj))
            elif t in ("/pose", "/odom") or t.startswith("/tf"):
                groups["pose"].append((ts, t, obj))
            elif t == "/agent/events":
                groups["events"].append((ts, t, obj))
    for v in groups.values():
        v.sort(key=lambda m: m[0])
    return groups


def _nearest(items: list[tuple[float, str, dict]], ts: float,
             tolerance: float = JOIN_TOLERANCE) -> dict | None:
    if not items:
        return None
    times = [m[0] for m in items]
    i = bisect.bisect_left(times, ts)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(items):
            dt = abs(items[j][0] - ts)
            if dt <= tolerance and (best is None or dt < best[0]):
                best = (dt, items[j][2])
    return best[1] if best else None


def _pose_xyz(pose_msg: dict | None) -> tuple[float | None, float | None, float | None]:
    if not pose_msg:
        return None, None, None
    pose = pose_msg.get("pose") or {}
    if "pose" in pose:  # nav_msgs/Odometry shape: pose.pose.position
        pose = pose["pose"]
    pos = pose.get("position") or {}
    if not pos and "position" in pose_msg:
        pos = pose_msg["position"]
    return pos.get("x"), pos.get("y"), pos.get("z")


def extract_run(mcap_path: str | Path, store=None,
                robot_id: str | None = None, thumbnails: bool = True) -> dict[str, Any]:
    """MCAP → Observation rows in the hot store. Runs on run close (spec §2.3).

    Camera frames anchor observations; the nearest detections, CLIP embedding,
    and pose within JOIN_TOLERANCE attach to each. Detection-only stretches
    (no camera channel) still produce rows so label search covers them.
    """
    import numpy as np

    mcap_path = Path(mcap_path)
    if store is None:
        from roborun.spatial_memory import SpatialMemoryStore
        store = SpatialMemoryStore()
    robot_id = robot_id or mcap_path.parent.name or "local"
    run_id = mcap_path.stem

    groups = _decode_messages(mcap_path)
    inserted = 0

    def _thumb(camera_obj: dict) -> bytes | None:
        if not thumbnails:
            return None
        try:
            jpeg = base64.b64decode(camera_obj.get("data", ""))
            import cv2
            arr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if arr is None:
                return jpeg or None
            small = cv2.resize(arr, (320, 240), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return buf.tobytes() if ok else jpeg
        except Exception:
            return None

    if groups["camera"]:
        for ts, topic, cam in groups["camera"]:
            det_msg = _nearest(groups["detections"], ts)
            clip_msg = _nearest(groups["clip"], ts)
            pose_msg = _nearest(groups["pose"], ts)
            x, y, z = _pose_xyz(pose_msg)
            embedding = None
            if clip_msg and clip_msg.get("vec"):
                embedding = np.frombuffer(
                    base64.b64decode(clip_msg["vec"]), dtype=np.float32)
            store.store(
                frame=None, thumbnail=_thumb(cam),
                embedding=embedding,
                detections=(det_msg or {}).get("detections") or [],
                x=x, y=y, z=z, robot_id=robot_id, ts=ts,
                run_id=run_id, frame_topic=topic,
                frame_log_time=int(ts * 1e9), source="mcap",
            )
            inserted += 1
    else:
        for ts, topic, det_msg in groups["detections"]:
            pose_msg = _nearest(groups["pose"], ts)
            x, y, z = _pose_xyz(pose_msg)
            store.store(
                frame=None, detections=det_msg.get("detections") or [],
                x=x, y=y, z=z, robot_id=robot_id, ts=ts,
                run_id=run_id, frame_topic=topic,
                frame_log_time=int(ts * 1e9), source="mcap",
            )
            inserted += 1

    return {"ok": True, "run": run_id, "robot_id": robot_id,
            "observations": inserted,
            "channels": {k: len(v) for k, v in groups.items() if v}}


def get_frame(mcap_path: str | Path, frame_topic: str,
              log_time: int, tolerance_ns: int = int(0.5e9)) -> bytes | None:
    """Pull the full-resolution frame a frame_ref points at, straight from MCAP."""
    from mcap.reader import make_reader
    with open(Path(mcap_path), "rb") as fh:
        reader = make_reader(fh)
        for _s, channel, message in reader.iter_messages(
                topics=[frame_topic],
                start_time=log_time - tolerance_ns,
                end_time=log_time + tolerance_ns):
            try:
                return base64.b64decode(json.loads(message.data).get("data", ""))
            except Exception:
                return None
    return None


# ── Parquet export + fleet queries (embedded DuckDB, no service) ─────────

def index_root() -> Path:
    import os
    base = os.environ.get("ROBORUN_STATE_DIR")
    root = Path(base) if base else Path.home() / ".roborun"
    return root / "index"


def export_parquet(store=None, robot_id: str = "local",
                   out_root: Path | None = None,
                   since: float | None = None,
                   include_embeddings: bool = True) -> dict[str, Any]:
    """Flatten Observation rows to index/<robot_id>/<date>.parquet."""
    try:
        import duckdb
    except ImportError:
        return {"ok": False, "error": "duckdb not installed (pip install 'ros-agent[fleet]')"}
    if store is None:
        from roborun.spatial_memory import SpatialMemoryStore
        store = SpatialMemoryStore()

    rows = list(store.iter_export_rows(robot_id=None, since=since,
                                       include_embeddings=include_embeddings))
    if not rows:
        return {"ok": False, "error": "no observations to export"}

    out_dir = (out_root or index_root()) / robot_id
    out_dir.mkdir(parents=True, exist_ok=True)
    date = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    out_path = out_dir / f"{date}.parquet"

    con = duckdb.connect()
    con.execute("""
        CREATE TABLE obs (
            obs_id VARCHAR, robot_id VARCHAR, run_id VARCHAR, ts DOUBLE,
            x DOUBLE, y DOUBLE, z DOUBLE,
            frame_topic VARCHAR, frame_log_time BIGINT, source VARCHAR,
            label VARCHAR, score DOUBLE, embedding FLOAT[]
        )""")
    con.executemany(
        "INSERT INTO obs VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        [(r["obs_id"], r["robot_id"], r["run_id"], r["ts"], r["x"], r["y"],
          r["z"], r["frame_topic"], r["frame_log_time"], r["source"],
          r["label"], r["score"], r["embedding"]) for r in rows])
    con.execute(f"COPY obs TO '{out_path}' (FORMAT PARQUET)")
    con.close()
    return {"ok": True, "parquet": str(out_path), "rows": len(rows),
            "key": f"index/{robot_id}/{date}.parquet"}


def fleet_query(sql: str, cache_dir: Path | None = None,
                sync_r2: bool = True) -> dict[str, Any]:
    """Run analytics SQL over every robot's Parquet index.

    The table is exposed as `fleet` (one row per observation × detection;
    columns: obs_id, robot_id, run_id, ts, x, y, z, frame_topic,
    frame_log_time, source, label, score, embedding FLOAT[]).

    Example: SELECT robot_id, count(*) FROM fleet WHERE label='forklift'
             AND ts > … GROUP BY robot_id
    """
    try:
        import duckdb
    except ImportError:
        return {"ok": False, "error": "duckdb not installed (pip install 'ros-agent[fleet]')"}

    root = cache_dir or index_root()
    if sync_r2:
        try:
            from roborun.r2sync import R2Store
            r2 = R2Store.from_env()
            if r2 is not None:
                r2.sync_down("index/", root)
        except Exception:
            pass  # offline or unconfigured: query the local index

    files = list(root.glob("*/*.parquet"))
    if not files:
        return {"ok": False, "error": f"no parquet indexes under {root}"}

    con = duckdb.connect()
    con.execute(f"CREATE VIEW fleet AS SELECT * FROM read_parquet('{root}/*/*.parquet')")
    try:
        cur = con.execute(sql)
        cols = [d[0] for d in cur.description]
        out = [dict(zip(cols, row)) for row in cur.fetchall()]
    except Exception as exc:
        con.close()
        return {"ok": False, "error": str(exc)}
    con.close()
    return {"ok": True, "rows": out, "row_count": len(out),
            "indexes": len(files)}


def fleet_search_clip(query_embedding, top_k: int = 10,
                      cache_dir: Path | None = None) -> dict[str, Any]:
    """Cross-robot CLIP search over the shared Parquet index (spec §3.3a)."""
    import numpy as np
    vec = np.asarray(query_embedding, dtype=np.float32).flatten()
    n = np.linalg.norm(vec)
    if n > 0:
        vec = vec / n
    arr = "[" + ",".join(f"{v:.6f}" for v in vec.tolist()) + "]"
    return fleet_query(
        f"SELECT obs_id, robot_id, run_id, ts, x, y, z, label, "
        f"list_cosine_similarity(embedding, {arr}::FLOAT[]) AS score "
        f"FROM fleet WHERE embedding IS NOT NULL "
        f"ORDER BY score DESC LIMIT {int(top_k)}",
        cache_dir=cache_dir)
