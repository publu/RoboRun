"""Multi-camera runtime (PERCEPTION_DATA_SPEC) — N source-tagged pipelines.

Ingestion today picks one camera; this manages several at once, each tagged with a
`source_id` so its frames/detections land on per-named MCAP channels
(`/camera/<source_id>`, `/detections/<source_id>`) and Observations carry the
source. Pipelines are duck-typed (`start/stop/snapshot/get_detections`), so the
manager works with `WebcamPipeline`, `RosCameraPipeline`, or a test stub — the
physical camera is the only part this can't supply.
"""
from __future__ import annotations

import threading
from typing import Any


class CameraManager:
    _instance: "CameraManager | None" = None

    def __init__(self) -> None:
        self._sources: dict[str, Any] = {}
        self._lock = threading.Lock()

    @classmethod
    def get(cls) -> "CameraManager":
        if cls._instance is None:
            cls._instance = CameraManager()
        return cls._instance

    def register(self, source_id: str, pipeline: Any) -> None:
        with self._lock:
            self._sources[source_id] = pipeline

    def remove(self, source_id: str) -> None:
        with self._lock:
            p = self._sources.pop(source_id, None)
        if p is not None:
            try:
                p.stop()
            except Exception:
                pass

    def sources(self) -> list[str]:
        with self._lock:
            return sorted(self._sources)

    def start_all(self, **kw: Any) -> dict[str, Any]:
        out = {}
        for sid, p in list(self._sources.items()):
            try:
                out[sid] = p.start(**kw)
            except Exception as exc:
                out[sid] = {"ok": False, "error": str(exc)}
        return out

    def stop_all(self) -> None:
        for p in list(self._sources.values()):
            try:
                p.stop()
            except Exception:
                pass

    def snapshot(self, source_id: str):
        with self._lock:
            p = self._sources.get(source_id)
        return p.snapshot() if p is not None else None

    def detections(self, source_id: str) -> list[dict]:
        with self._lock:
            p = self._sources.get(source_id)
        return p.get_detections() if p is not None else []

    def record_into(self, recorder, encode=True) -> int:
        """Tap every source's current frame + detections into the recorder on
        per-source channels. Returns how many sources contributed."""
        import cv2
        n = 0
        for sid in self.sources():
            frame = self.snapshot(sid)
            if frame is None:
                continue
            dets = self.detections(sid)
            if encode:
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    recorder.write_camera(buf.tobytes(), name=sid)
            if dets:
                recorder.write_detections(dets, name=sid)
            n += 1
        return n
