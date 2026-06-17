"""Synthetic camera (PERCEPTION_DATA_SPEC) — a software camera with no hardware.

A drop-in for `WebcamPipeline` (same `start/stop/snapshot/get_detections` surface)
that renders a procedural scene with **sensor noise + lens distortion**, and emits
ground-truth detections for the objects it drew. Lets the full multi-camera +
perception + record path run with no physical camera *and* no model download —
useful for CI, demos, and as a deterministic test source.
"""
from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np


class SyntheticCamera:
    def __init__(self, width: int = 320, height: int = 240, fps: int = 15,
                 noise: float = 6.0, distortion: float = 0.0,
                 objects: list[dict] | None = None, label: str = "box") -> None:
        self.w, self.h, self.fps = width, height, fps
        self.noise, self.distortion = noise, distortion
        self.label = label
        # objects: [{label, w, h, speed}] moving left→right; default one box
        self._objs = objects or [{"label": label, "w": 40, "h": 40, "speed": 4}]
        self._frame: np.ndarray | None = None
        self._dets: list[dict] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t: threading.Thread | None = None
        self._i = 0

    def start(self, *a: Any, **kw: Any) -> dict:
        self._stop.clear()
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()
        return {"ok": True, "synthetic": True, "resolution": f"{self.w}x{self.h}"}

    def stop(self, *a: Any, **kw: Any) -> dict:
        self._stop.set()
        return {"ok": True}

    def snapshot(self) -> np.ndarray | None:
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def get_detections(self) -> list[dict]:
        with self._lock:
            return list(self._dets)

    def get_clip_matches(self) -> list[dict]:
        return []

    def _render(self) -> tuple[np.ndarray, list[dict]]:
        import cv2
        img = np.full((self.h, self.w, 3), 28, np.uint8)
        dets = []
        for k, o in enumerate(self._objs):
            x = (self._i * o["speed"]) % (self.w - o["w"])
            y = self.h // 2 - o["h"] // 2
            color = [(60, 200, 90), (200, 90, 60), (90, 120, 220)][k % 3]
            cv2.rectangle(img, (x, y), (x + o["w"], y + o["h"]), color, -1)
            dets.append({"label": o["label"], "score": 0.95,
                         "bbox": [x, y, x + o["w"], y + o["h"]]})
        # lens distortion (barrel) then sensor noise — realism knobs
        if self.distortion:
            img = self._barrel(img, self.distortion)
        if self.noise:
            img = np.clip(img.astype(np.int16) +
                          np.random.normal(0, self.noise, img.shape).astype(np.int16),
                          0, 255).astype(np.uint8)
        return img, dets

    @staticmethod
    def _barrel(img: np.ndarray, k: float) -> np.ndarray:
        import cv2
        h, w = img.shape[:2]
        cam = np.array([[w, 0, w / 2], [0, w, h / 2], [0, 0, 1]], np.float32)
        dist = np.array([k, 0, 0, 0], np.float32)
        return cv2.undistort(img, cam, dist)

    def _loop(self) -> None:
        dt = 1.0 / max(1, self.fps)
        while not self._stop.is_set():
            img, dets = self._render()
            with self._lock:
                self._frame, self._dets = img, dets
            self._i += 1
            time.sleep(dt)
