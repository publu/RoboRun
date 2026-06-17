"""PerceptionSession — the one system across sim / robot / production.

Whatever the source (a sim camera, a robot's ROS camera, or a production webcam),
the same loop runs: grab a frame → YOLO detections + CLIP embedding → record to the
sealed MCAP → stream into the searchable index. The store accumulates across every
run, so "find anyone/anything over time" is one query over all of history —
semantic (CLIP) + label (YOLO) + a time window — regardless of which mode produced
the data. This is the through-line the whole stack was built for.

    sess = PerceptionSession.for_mode("production", source_id="lobby")
    sess.start()
    ...
    search(sess.store, "person in red", since=yesterday)   # across all runs/modes
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable

MODES = ("sim", "robot", "production")


def default_embedder() -> Callable | None:
    """A CLIP image embedder if vision is installed, else None (YOLO-only)."""
    try:
        from roborun.models import CLIPMatcher
        clip = CLIPMatcher()
        return lambda frame: clip.embed_image(frame)
    except Exception:
        return None


def text_embedder() -> Callable | None:
    try:
        from roborun.models import CLIPMatcher
        clip = CLIPMatcher()
        return lambda text: clip.embed_text(text)
    except Exception:
        return None


def open_source(mode: str, **kw: Any):
    """Pick the frame source for a mode. All share the snapshot/get_detections
    duck-type, so the session loop is identical across modes."""
    if mode == "sim":
        from roborun.synthetic_camera import SyntheticCamera
        return SyntheticCamera(label=kw.get("label", "person"))
    if mode == "robot":
        from roborun.ros_camera import get_ros_camera
        return get_ros_camera()
    if mode == "production":
        from roborun.routes._singletons import get_webcam
        return get_webcam()
    raise ValueError(f"mode must be one of {MODES}, got {mode!r}")


class PerceptionSession:
    def __init__(self, source: Any, store: Any, robot_id: str = "local",
                 mode: str = "production", source_id: str = "cam",
                 recorder: Any = None, embed_fn: Callable | None = None,
                 hz: float = 5.0, run_id: str | None = None) -> None:
        self.source = source
        self.store = store
        self.robot_id = robot_id
        self.mode = mode
        self.source_id = source_id
        self.recorder = recorder
        self.embed_fn = embed_fn
        self.run_id = run_id
        self._dt = 1.0 / max(0.1, hz)
        self._stop = threading.Event()
        self._t: threading.Thread | None = None
        self.indexed = 0
        # so the live index tags observations with the real mode (sim/robot/production)
        ext = getattr(recorder, "extractor", None)
        if ext is not None:
            ext.source = mode

    @classmethod
    def for_mode(cls, mode: str, store: Any = None, source_id: str = "cam",
                 recorder: Any = None, embed: bool = True, **kw: Any):
        if store is None:
            from roborun.spatial_memory import SpatialMemoryStore
            store = SpatialMemoryStore()
        return cls(open_source(mode, **kw), store, mode=mode, source_id=source_id,
                   recorder=recorder,
                   embed_fn=default_embedder() if embed else None,
                   run_id=getattr(recorder, "run_id", None))

    def tick(self) -> bool:
        """One frame through YOLO+CLIP → record + index. Returns True if a frame
        was processed."""
        frame = self.source.snapshot()
        if frame is None:
            return False
        dets = self.source.get_detections()
        emb = None
        if self.embed_fn is not None:
            try:
                emb = self.embed_fn(frame)
            except Exception:
                emb = None
        ts = time.time()
        if self.recorder is not None:
            import cv2
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                self.recorder.write_camera(buf.tobytes(), name=self.source_id, ts=ts)
            if dets:
                self.recorder.write_detections(dets, name=self.source_id, ts=ts)
            if emb is not None:
                self.recorder.write_clip(emb, frame_topic=f"/camera/{self.source_id}", ts=ts)
            # streaming extractor (if attached) indexes it; else index directly
            if getattr(self.recorder, "extractor", None) is None:
                self._store(frame, emb, dets, ts)
        else:
            self._store(frame, emb, dets, ts)
        self.indexed += 1
        return True

    def _store(self, frame, emb, dets, ts) -> None:
        self.store.store(frame=frame, embedding=emb, detections=dets, ts=ts,
                         robot_id=self.robot_id, run_id=self.run_id,
                         source=self.mode, source_id=self.source_id)

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self.tick()
            except Exception:
                pass
            self._stop.wait(self._dt)

    def start(self) -> None:
        try:
            self.source.start()
        except Exception:
            pass
        self._stop.clear()
        self._t = threading.Thread(target=self._loop, daemon=True,
                                   name=f"perception-{self.source_id}")
        self._t.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self.source.stop()
        except Exception:
            pass


def export_dataset(store: Any, query: Any, out_dir: str, by: str = "label",
                   k: int = 500, since: float | None = None,
                   until: float | None = None) -> dict:
    """Curate a labeled dataset from a search (Foxglove 'curate datasets to train
    models', on our sealed substrate): search the all-time index, pull each hit's
    full frame from its MCAP, write images/ + labels.jsonl + dataset.json. Every
    image traces back to a sealed run, so the dataset's provenance is verifiable."""
    import json
    import time
    from pathlib import Path
    from roborun.observations import get_frame
    from roborun.run_series import _find_mcap

    hits = search(store, query, by=by, k=k, since=since, until=until)
    out = Path(out_dir)
    (out / "images").mkdir(parents=True, exist_ok=True)
    manifest = []
    for i, h in enumerate(hits):
        img = None
        fr = h.get("frame_ref")
        if fr and fr.get("run_id") and fr.get("topic") and fr.get("log_time"):
            mcap = _find_mcap(fr["run_id"], h.get("robot_id"))
            if mcap is not None:
                img = get_frame(mcap, fr["topic"], int(fr["log_time"]))
        if img is None and h.get("id"):
            img = store.get_thumbnail(h["id"])  # fall back to the stored thumb
        if img is None:
            continue
        name = f"{i:05d}.jpg"
        (out / "images" / name).write_bytes(img)
        manifest.append({"image": f"images/{name}",
                         "detections": h.get("detections"), "ts": h.get("ts"),
                         "robot_id": h.get("robot_id"), "source": h.get("source"),
                         "run_id": (fr or {}).get("run_id")})
    (out / "labels.jsonl").write_text("\n".join(json.dumps(m) for m in manifest))
    (out / "dataset.json").write_text(json.dumps(
        {"query": str(query), "by": by, "count": len(manifest),
         "created": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}, indent=2))
    return {"ok": True, "count": len(manifest), "dir": str(out)}


def search(store: Any, query: Any, by: str = "clip", k: int = 10,
           since: float | None = None, until: float | None = None,
           robot_id: str | None = None, source_id: str | None = None) -> list[dict]:
    """Find anything/anyone across ALL recorded history, in any mode.
      by="clip"  semantic (query: text → CLIP, or an embedding)
      by="label" YOLO label · by="near" place · by="time" just a window
    `since`/`until` (unix seconds) bound the time window — "who was here last week"."""
    if by == "clip" and isinstance(query, str):
        emb = text_embedder()
        if emb is not None:
            query = emb(query)
    return store.recall(query, by=by, k=k, since=since, until=until,
                        robot_id=robot_id, source_id=source_id)
