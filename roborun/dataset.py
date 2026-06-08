"""Dataset collection for RoboRun — record episodes from webcam or robot."""

from __future__ import annotations

import json
import time
import uuid
from pathlib import Path
from threading import RLock
from typing import Any

import cv2
import numpy as np

DATASETS_ROOT = Path.cwd() / "datasets"


class Episode:
    """A single recorded episode (sequence of frames + metadata)."""

    def __init__(self, dataset_name: str, episode_id: str | None = None) -> None:
        self.dataset_name = dataset_name
        self.episode_id = episode_id or uuid.uuid4().hex[:12]
        self.root = DATASETS_ROOT / dataset_name / self.episode_id
        self.frames_dir = self.root / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.metadata: dict[str, Any] = {
            "episode_id": self.episode_id,
            "dataset": dataset_name,
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "frames": [],
            "annotations": [],
        }
        self._frame_count = 0

    def add_frame(
        self,
        frame: np.ndarray,
        detections: list[dict] | None = None,
        action: dict | None = None,
        timestamp: float | None = None,
    ) -> int:
        ts = timestamp or time.time()
        idx = self._frame_count
        self._frame_count += 1

        fname = f"{idx:06d}.jpg"
        cv2.imwrite(str(self.frames_dir / fname), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

        entry: dict[str, Any] = {
            "index": idx,
            "file": fname,
            "timestamp": ts,
        }
        if detections:
            entry["detections"] = detections
        if action:
            entry["action"] = action

        self.metadata["frames"].append(entry)
        return idx

    def add_annotation(self, frame_index: int, label: str, data: dict | None = None) -> None:
        self.metadata["annotations"].append({
            "frame_index": frame_index,
            "label": label,
            "data": data or {},
            "timestamp": time.time(),
        })

    def save(self) -> Path:
        self.metadata["total_frames"] = self._frame_count
        self.metadata["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        meta_path = self.root / "episode.json"
        meta_path.write_text(json.dumps(self.metadata, indent=2))
        return self.root

    @property
    def frame_count(self) -> int:
        return self._frame_count


class DatasetCollector:
    """Manages dataset recording sessions."""

    def __init__(self) -> None:
        self._lock = RLock()
        self._current_episode: Episode | None = None
        self._recording = False

    @property
    def is_recording(self) -> bool:
        return self._recording

    def start_recording(self, dataset_name: str = "default") -> dict[str, Any]:
        with self._lock:
            if self._recording:
                return {"ok": False, "error": "Already recording"}
            self._current_episode = Episode(dataset_name)
            self._recording = True
            return {
                "ok": True,
                "episode_id": self._current_episode.episode_id,
                "dataset": dataset_name,
                "path": str(self._current_episode.root),
            }

    def stop_recording(self) -> dict[str, Any]:
        with self._lock:
            if not self._recording or not self._current_episode:
                return {"ok": False, "error": "Not recording"}
            path = self._current_episode.save()
            result = {
                "ok": True,
                "episode_id": self._current_episode.episode_id,
                "frames": self._current_episode.frame_count,
                "path": str(path),
            }
            self._recording = False
            self._current_episode = None
            return result

    def record_frame(
        self,
        frame: np.ndarray,
        detections: list[dict] | None = None,
        action: dict | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if not self._recording or not self._current_episode:
                return {"ok": False}
            idx = self._current_episode.add_frame(frame, detections, action)
            return {"ok": True, "frame_index": idx}

    def annotate(self, frame_index: int, label: str, data: dict | None = None) -> dict[str, Any]:
        with self._lock:
            if not self._current_episode:
                return {"ok": False, "error": "No active episode"}
            self._current_episode.add_annotation(frame_index, label, data)
            return {"ok": True}

    def list_datasets(self) -> dict[str, Any]:
        DATASETS_ROOT.mkdir(parents=True, exist_ok=True)
        datasets = []
        for ds_dir in sorted(DATASETS_ROOT.iterdir()):
            if not ds_dir.is_dir():
                continue
            episodes = []
            for ep_dir in sorted(ds_dir.iterdir()):
                meta = ep_dir / "episode.json"
                if meta.exists():
                    try:
                        info = json.loads(meta.read_text())
                        episodes.append({
                            "episode_id": info.get("episode_id", ep_dir.name),
                            "frames": info.get("total_frames", 0),
                            "created_at": info.get("created_at", ""),
                        })
                    except Exception:
                        pass
            datasets.append({
                "name": ds_dir.name,
                "episodes": len(episodes),
                "details": episodes,
            })
        return {"ok": True, "datasets": datasets}

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            if not self._recording or not self._current_episode:
                return {"recording": False}
            return {
                "recording": True,
                "episode_id": self._current_episode.episode_id,
                "dataset": self._current_episode.dataset_name,
                "frames": self._current_episode.frame_count,
                "started_at": self._current_episode.metadata.get("created_at", ""),
            }
