"""Depth processor — generates heatmaps and point clouds from depth data."""

from __future__ import annotations

import base64
import threading
import time
from typing import Any

import cv2
import numpy as np

HEATMAP_PATH = "/tmp/roborun_depth.jpg"
DOWNSAMPLE_POINTS = 5000


class DepthProcessor:
    _instance: DepthProcessor | None = None
    _lock = threading.Lock()

    def __init__(self) -> None:
        self._depth_frame: np.ndarray | None = None
        self._rgb_frame: np.ndarray | None = None
        self._heatmap: bytes | None = None
        self._pointcloud: list[list[float]] | None = None
        self._buf_lock = threading.Lock()
        self._last_update: float = 0.0

    @classmethod
    def get(cls) -> DepthProcessor:
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    def update(self, depth: np.ndarray, rgb: np.ndarray | None = None) -> None:
        with self._buf_lock:
            self._depth_frame = depth.copy()
            if rgb is not None:
                self._rgb_frame = rgb.copy()
            self._last_update = time.time()
            self._heatmap = None
            self._pointcloud = None

    def get_heatmap(self) -> dict[str, Any]:
        with self._buf_lock:
            if self._depth_frame is None:
                return {"ok": False, "error": "No depth data"}
            if self._heatmap is None:
                self._heatmap = self._render_heatmap(self._depth_frame)
        b64 = base64.b64encode(self._heatmap).decode()
        return {
            "ok": True,
            "image": f"data:image/jpeg;base64,{b64}",
            "min_m": float(np.nanmin(self._depth_frame)),
            "max_m": float(np.nanmax(self._depth_frame)),
            "mean_m": float(np.nanmean(self._depth_frame)),
        }

    def get_pointcloud(self, fx: float = 525.0, fy: float = 525.0,
                       cx: float = 0, cy: float = 0) -> dict[str, Any]:
        with self._buf_lock:
            if self._depth_frame is None:
                return {"ok": False, "error": "No depth data"}
            depth = self._depth_frame
            rgb = self._rgb_frame

        h, w = depth.shape[:2]
        if cx == 0:
            cx = w / 2.0
        if cy == 0:
            cy = h / 2.0

        step = max(1, int(np.sqrt(h * w / DOWNSAMPLE_POINTS)))
        ys = np.arange(0, h, step)
        xs = np.arange(0, w, step)
        yy, xx = np.meshgrid(ys, xs, indexing="ij")

        d = depth[yy, xx].astype(np.float64)
        valid = (d > 0.01) & (d < 20.0) & np.isfinite(d)

        z = d[valid]
        x = (xx[valid] - cx) * z / fx
        y = (yy[valid] - cy) * z / fy

        if rgb is not None and rgb.shape[:2] == depth.shape[:2]:
            colors = rgb[yy[valid], xx[valid]]
            if colors.shape[1] == 3:
                r, g, b = colors[:, 2], colors[:, 1], colors[:, 0]
            else:
                r = g = b = np.full(len(z), 128)
        else:
            norm = np.clip((z - z.min()) / max(z.ptp(), 0.01), 0, 1)
            r = (norm * 255).astype(np.uint8)
            g = ((1 - norm) * 200).astype(np.uint8)
            b = np.full(len(z), 100, dtype=np.uint8)

        points = np.column_stack([x, -y, -z, r, g, b]).tolist()
        return {"ok": True, "points": points, "count": len(points)}

    @staticmethod
    def _render_heatmap(depth: np.ndarray) -> bytes:
        valid = depth[np.isfinite(depth) & (depth > 0)]
        if len(valid) == 0:
            blank = np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
            _, buf = cv2.imencode(".jpg", blank)
            return buf.tobytes()
        vmin, vmax = np.percentile(valid, [2, 98])
        norm = np.clip((depth - vmin) / max(vmax - vmin, 0.01), 0, 1)
        norm = (norm * 255).astype(np.uint8)
        colored = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)

        h, w = colored.shape[:2]
        min_str = f"Min: {vmin:.2f}m"
        max_str = f"Max: {vmax:.2f}m"
        mean_str = f"{np.mean(valid):.2f}m"
        cv2.putText(colored, min_str, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(colored, max_str, (w - 120, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(colored, mean_str, (w // 2 - 30, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        _, buf = cv2.imencode(".jpg", colored, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return buf.tobytes()
