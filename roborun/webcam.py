"""Webcam capture and processing pipeline for RoboRun."""

from __future__ import annotations

import json
import time
from pathlib import Path
from threading import Event, RLock, Thread
from typing import Any

import cv2
import numpy as np

from roborun.models import CLIPMatcher, Detection, JEPAEncoder, YOLODetector

FRAME_PATH = Path("/tmp/roborun_frame.jpg")
STATE_PATH = Path("/tmp/roborun_state.json")

_FONT = cv2.FONT_HERSHEY_SIMPLEX


class WebcamPipeline:
    """Captures webcam frames and runs selected models in real time."""

    def __init__(self) -> None:
        self._cap: cv2.VideoCapture | None = None
        self._lock = RLock()
        self._should_stop = Event()
        self._thread: Thread | None = None

        self._yolo: YOLODetector | None = None
        self._clip: CLIPMatcher | None = None
        self._jepa: JEPAEncoder | None = None
        self._active_models: set[str] = set()

        self._latest_frame: np.ndarray | None = None
        self._latest_detections: list[Detection] = []
        self._latest_clip_matches: list[Detection] = []
        self._latest_jepa_heatmap: np.ndarray | None = None
        self._clip_query: str = ""
        self._frame_count: int = 0
        self._fps: float = 0.0
        self._state: str = "idle"
        self._camera_index: int = 0

        self._timeline_enabled: bool = True
        self._timeline_interval: float = 3.0
        self._last_timeline_ts: float = 0.0

        self._prev_yolo_labels: set[str] = set()
        self._prev_clip_labels: set[str] = set()

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, camera_index: int | str = 0, models: list[str] | None = None) -> dict[str, Any]:
        """Source can be a camera index, a video file path, or a stream URL.
        File sources loop forever and play at native FPS."""
        if self.is_running:
            return {"ok": True, "already_running": True}

        self._camera_index = camera_index
        self._is_file_source = isinstance(camera_index, str)
        self._cap = cv2.VideoCapture(camera_index)
        if not self._cap.isOpened():
            return {"ok": False, "error": f"Cannot open source {camera_index}"}

        if self._is_file_source:
            native_fps = self._cap.get(cv2.CAP_PROP_FPS) or 30.0
            self._source_frame_interval = 1.0 / min(max(native_fps, 1.0), 60.0)
        else:
            self._source_frame_interval = 0.0
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self._active_models = set(models or [])

        if "yolo" in self._active_models:
            if self._yolo is None:
                self._yolo = YOLODetector()

        if "clip" in self._active_models:
            if self._clip is None:
                self._clip = CLIPMatcher()

        if "jepa" in self._active_models:
            if self._jepa is None:
                self._jepa = JEPAEncoder()

        self._should_stop.clear()
        self._state = "running"
        self._thread = Thread(target=self._capture_loop, daemon=True, name="WebcamPipeline")
        self._thread.start()

        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return {"ok": True, "resolution": f"{w}x{h}", "models": list(self._active_models)}

    def stop(self) -> dict[str, Any]:
        self._should_stop.set()
        self._state = "idle"
        if self._thread:
            self._thread.join(timeout=5)
            self._thread = None
        if self._cap:
            self._cap.release()
            self._cap = None
        return {"ok": True}

    def set_models(self, models: list[str]) -> dict[str, Any]:
        new = set(models)
        if "yolo" in new and self._yolo is None:
            self._yolo = YOLODetector()
        if "clip" in new and self._clip is None:
            self._clip = CLIPMatcher()
        if "jepa" in new and self._jepa is None:
            self._jepa = JEPAEncoder()
        self._active_models = new
        return {"ok": True, "models": list(self._active_models)}

    def set_clip_query(self, query: str) -> dict[str, Any]:
        self._clip_query = query.strip()
        self._latest_clip_matches = []
        return {"ok": True, "query": self._clip_query}

    def get_state(self) -> dict[str, Any]:
        return {
            "running": self.is_running,
            "state": self._state,
            "models": list(self._active_models),
            "fps": round(self._fps, 1),
            "frame_count": self._frame_count,
            "detections": len(self._latest_detections),
            "clip_query": self._clip_query,
            "clip_matches": len(self._latest_clip_matches),
            "camera": self._camera_index,
        }

    def get_detections(self) -> list[dict]:
        return [d.to_dict() for d in self._latest_detections]

    def get_clip_matches(self) -> list[dict]:
        return [d.to_dict() for d in self._latest_clip_matches]

    def snapshot(self) -> np.ndarray | None:
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def _capture_loop(self) -> None:
        fps_window: list[float] = []
        clip_interval = 10

        try:
            while not self._should_stop.is_set():
                t0 = time.monotonic()

                ret, frame = self._cap.read()
                if not ret:
                    if getattr(self, "_is_file_source", False):
                        self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop footage
                        continue
                    time.sleep(0.01)
                    continue

                self._frame_count += 1
                with self._lock:
                    self._latest_frame = frame

                detections: list[Detection] = []
                clip_matches: list[Detection] = []

                if "yolo" in self._active_models and self._yolo:
                    try:
                        detections = self._yolo.detect(frame)
                        self._latest_detections = detections
                    except Exception:
                        self._active_models.discard("yolo")

                if "clip" in self._active_models and self._clip and self._clip_query:
                    if self._frame_count % clip_interval == 0:
                        try:
                            targets = detections if detections else [
                                Detection(bbox=(0, 0, frame.shape[1], frame.shape[0]),
                                          label="full_frame", confidence=1.0)
                            ]
                            clip_matches = self._clip.match_detections(
                                self._clip_query, frame, targets, threshold=0.15,
                            )
                            self._latest_clip_matches = clip_matches
                            cur_clip = {d.label for d in clip_matches}
                            clip_in = cur_clip - self._prev_clip_labels
                            clip_out = self._prev_clip_labels - cur_clip
                            self._prev_clip_labels = cur_clip
                            try:
                                from roborun.events import emit
                                if clip_in:
                                    emit("detection", "clip", f"CLIP match: {self._clip_query}", {
                                        "action": "in", "query": self._clip_query,
                                        "labels": sorted(clip_in),
                                    })
                                if clip_out:
                                    emit("detection", "clip", f"CLIP lost: {self._clip_query}", {
                                        "action": "out", "query": self._clip_query,
                                        "labels": sorted(clip_out),
                                    })
                            except Exception:
                                pass
                        except Exception:
                            self._active_models.discard("clip")

                if "jepa" in self._active_models and self._jepa:
                    if self._frame_count % 5 == 0:
                        try:
                            features = self._jepa.encode(frame)
                            self._latest_jepa_heatmap = self._features_to_heatmap(features, frame.shape[:2])
                        except Exception:
                            self._active_models.discard("jepa")

                annotated = self._annotate(frame, detections, self._latest_clip_matches)
                ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    FRAME_PATH.write_bytes(buf.tobytes())

                self._write_state()
                self._maybe_timeline(frame, detections)

                elapsed = time.monotonic() - t0
                fps_window.append(elapsed)
                if len(fps_window) > 30:
                    fps_window.pop(0)
                self._fps = 1.0 / (sum(fps_window) / len(fps_window)) if fps_window else 0

                target = getattr(self, "_source_frame_interval", 0.0) or (1.0 / 30.0)
                sleep_dur = max(0, target - elapsed)
                if sleep_dur > 0:
                    time.sleep(sleep_dur)
        except Exception:
            pass
        finally:
            self._state = "idle"

    def _annotate(
        self, frame: np.ndarray, detections: list[Detection], clip_matches: list[Detection],
    ) -> np.ndarray:
        out = frame.copy()
        h, w = out.shape[:2]

        clip_ids = {d.track_id for d in clip_matches if d.track_id >= 0}

        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det.bbox]
            is_match = det.track_id in clip_ids
            color = (0, 255, 120) if is_match else (255, 200, 0)
            thickness = 3 if is_match else 2
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness)

            tag = f"#{det.track_id} {det.label} {det.confidence:.0%}"
            if is_match:
                match = next((m for m in clip_matches if m.track_id == det.track_id), None)
                if match and match.clip_score is not None:
                    tag += f" MATCH {match.clip_score:.0%}"

            (tw, th), _ = cv2.getTextSize(tag, _FONT, 0.5, 1)
            ty = max(y1 - 6, th + 4)
            cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 4, ty + 2), color, -1)
            cv2.putText(out, tag, (x1 + 2, ty), _FONT, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        model_tags = " + ".join(sorted(self._active_models)) or "no models"
        overlay_lines = [
            f"ros-agent | {model_tags}",
            f"{self._fps:.0f} fps | {len(detections)} objects",
        ]
        if self._clip_query:
            overlay_lines.append(f"CLIP: \"{self._clip_query}\" ({len(clip_matches)} match)")

        y0 = 22
        for line in overlay_lines:
            (lw, lh), _ = cv2.getTextSize(line, _FONT, 0.55, 2)
            cv2.rectangle(out, (8, y0 - lh - 4), (14 + lw, y0 + 4), (20, 20, 20), -1)
            cv2.putText(out, line, (10, y0), _FONT, 0.55, (0, 255, 180), 2, cv2.LINE_AA)
            y0 += lh + 10

        # JEPA attention heatmap overlay
        if self._latest_jepa_heatmap is not None and "jepa" in self._active_models:
            heatmap = cv2.applyColorMap(self._latest_jepa_heatmap, cv2.COLORMAP_INFERNO)
            out = cv2.addWeighted(out, 0.6, heatmap, 0.4, 0)
            cv2.putText(out, "JEPA", (w - 70, 24), _FONT, 0.55, (200, 120, 255), 2, cv2.LINE_AA)

        return out

    @staticmethod
    def _features_to_heatmap(features: np.ndarray, target_size: tuple[int, int]) -> np.ndarray:
        h, w = target_size
        # features shape: (num_patches, dim) or (1+num_patches, dim) with CLS token
        if features.ndim == 2:
            n_patches = features.shape[0]
            # skip CLS token if present (common in ViT: 1 + 14*14 = 197)
            grid_size = int(np.sqrt(n_patches))
            if grid_size * grid_size < n_patches:
                features = features[1:]  # drop CLS
                n_patches = features.shape[0]
                grid_size = int(np.sqrt(n_patches))
            activation = np.linalg.norm(features, axis=-1)
            activation = activation[:grid_size * grid_size].reshape(grid_size, grid_size)
        elif features.ndim == 1:
            activation = features.reshape(1, 1)
        else:
            activation = np.linalg.norm(features, axis=-1)
            if activation.ndim > 2:
                activation = activation.mean(axis=tuple(range(activation.ndim - 2)))

        lo, hi = activation.min(), activation.max()
        if hi > lo:
            activation = (activation - lo) / (hi - lo)
        else:
            activation = np.zeros_like(activation)
        heatmap = (activation * 255).astype(np.uint8)
        return cv2.resize(heatmap, (w, h), interpolation=cv2.INTER_LINEAR)

    def _maybe_timeline(self, frame: np.ndarray, detections: list[Detection]) -> None:
        if not self._timeline_enabled:
            return
        now = time.monotonic()
        if now - self._last_timeline_ts < self._timeline_interval:
            return
        self._last_timeline_ts = now
        frame_copy = frame.copy()
        det_dicts = [d.to_dict() for d in detections]
        current_labels = {d["label"] for d in det_dicts}
        entered = current_labels - self._prev_yolo_labels
        exited = self._prev_yolo_labels - current_labels
        self._prev_yolo_labels = current_labels
        try:
            from roborun.events import emit
            if entered:
                emit("detection", "yolo", ", ".join(sorted(entered)), {
                    "action": "in",
                    "labels": sorted(entered),
                })
            if exited:
                emit("detection", "yolo", ", ".join(sorted(exited)), {
                    "action": "out",
                    "labels": sorted(exited),
                })
        except Exception:
            pass
        Thread(target=self._store_timeline, args=(frame_copy, det_dicts), daemon=True).start()

    @staticmethod
    def _store_timeline(frame: np.ndarray, det_dicts: list[dict]) -> None:
        try:
            from roborun.server import _get_memory
            mem = _get_memory()
            mem.store(
                frame=frame,
                detections=det_dicts,
                metadata={"source": "timeline"},
            )
        except Exception:
            pass

    def _write_state(self) -> None:
        state = {
            "mode": "webcam",
            "state": self._state,
            "models": list(self._active_models),
            "fps": round(self._fps, 1),
            "objects": len(self._latest_detections),
            "classes": list({d.label for d in self._latest_detections}),
            "clip_query": self._clip_query,
            "clip_matches": len(self._latest_clip_matches),
            "ts": time.time(),
        }
        try:
            STATE_PATH.write_text(json.dumps(state))
        except Exception:
            pass
