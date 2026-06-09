"""SceneBuilder — accumulates a colored 3D point cloud from webcam + depth estimation.

Runs in its own thread at ~1 keyframe/s. Estimates camera motion via ORB
feature matching + solvePnPRansac so the cloud grows as the camera moves.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import cv2
import numpy as np


MAX_ACCUMULATED_POINTS = 50_000
DOWNSAMPLE_PER_FRAME = 3000
JSON_POINT_CAP = 15_000


class SceneBuilder:
    _instance: SceneBuilder | None = None
    _cls_lock = threading.Lock()

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        self._depth_estimator = None
        self._available = False
        self._last_error: str | None = None
        self._loop_state: str = "idle"
        self._webcam_ref = None

        self._accumulated: np.ndarray | None = None  # Nx6 [x,y,z,r,g,b]
        self._markers: list[dict] = []
        self._camera_pos: dict = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._keyframe_count: int = 0

        # Visual odometry state
        self._prev_gray: np.ndarray | None = None
        self._prev_kps = None
        self._prev_des = None
        self._prev_depth: np.ndarray | None = None
        self._pose = np.eye(4, dtype=np.float64)  # accumulated camera pose
        self._orb = cv2.ORB_create(nfeatures=1000)
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    @classmethod
    def get(cls) -> SceneBuilder:
        if cls._instance is None:
            with cls._cls_lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> dict[str, Any]:
        if self.is_running:
            return {"ok": True, "already_running": True}
        try:
            from roborun.models import DepthEstimator
            if self._depth_estimator is None:
                self._depth_estimator = DepthEstimator()
            self._available = True
        except ImportError:
            return {"ok": False, "error": "transformers not installed — pip install ros-agent[depth]"}

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._build_loop, daemon=True, name="SceneBuilder")
        self._thread.start()
        return {"ok": True}

    def stop(self) -> dict[str, Any]:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=5)
            self._thread = None
        return {"ok": True}

    def clear(self) -> dict[str, Any]:
        with self._lock:
            self._accumulated = None
            self._markers = []
            self._camera_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
            self._keyframe_count = 0
            self._prev_gray = None
            self._prev_kps = None
            self._prev_des = None
            self._prev_depth = None
            self._pose = np.eye(4, dtype=np.float64)
        return {"ok": True}

    def get_scene(self) -> dict[str, Any]:
        with self._lock:
            if self._accumulated is None or len(self._accumulated) == 0:
                return {"ok": True, "points": [], "count": 0,
                        "markers": [], "camera": self._camera_pos, "keyframes": 0}

            pts = self._accumulated
            if len(pts) > JSON_POINT_CAP:
                idx = np.random.choice(len(pts), JSON_POINT_CAP, replace=False)
                pts = pts[idx]

            return {
                "ok": True,
                "points": pts.tolist(),
                "count": len(pts),
                "markers": list(self._markers),
                "camera": dict(self._camera_pos),
                "keyframes": self._keyframe_count,
            }

    def _build_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._loop_state = "get_webcam"
                wc = self._webcam_ref
                if wc is None:
                    self._loop_state = "no_webcam_ref"
                    time.sleep(1)
                    continue
                self._loop_state = f"snapshot(wc={type(wc).__name__},running={wc.is_running})"
                frame = wc.snapshot()
                if frame is None:
                    self._loop_state = f"no_frame(latest={wc._latest_frame is not None},fc={wc._frame_count})"
                    time.sleep(0.5)
                    continue

                self._loop_state = f"depth({frame.shape})"
                depth = self._depth_estimator.estimate(frame)
                self._loop_state = "motion"
                self._estimate_motion(frame, depth)
                self._loop_state = "cloud"
                cloud = self._depth_to_cloud(frame, depth)

                detections = wc.get_detections()
                markers = self._detections_to_markers(detections, depth)

                with self._lock:
                    if self._accumulated is None:
                        self._accumulated = cloud
                    else:
                        self._accumulated = np.vstack([self._accumulated, cloud])
                        if len(self._accumulated) > MAX_ACCUMULATED_POINTS:
                            self._accumulated = self._accumulated[-MAX_ACCUMULATED_POINTS:]
                    self._markers = markers
                    t = self._pose[:3, 3]
                    self._camera_pos = {"x": float(t[0]), "y": float(t[1]), "z": float(t[2])}
                    self._keyframe_count += 1

                    # Push camera pose to telemetry
                    try:
                        from roborun.telemetry import TelemetryBus
                        TelemetryBus.get().push("scene", "camera_pose", {
                            "x": self._camera_pos["x"],
                            "y": self._camera_pos["y"],
                            "z": self._camera_pos["z"],
                        })
                    except Exception:
                        pass

            except Exception as exc:
                import traceback
                self._last_error = traceback.format_exc()
                traceback.print_exc()

            time.sleep(1.0)

    def _depth_to_cloud(self, frame: np.ndarray, depth: np.ndarray) -> np.ndarray:
        h, w = depth.shape[:2]
        fx = fy = w * 0.8
        cx, cy = w / 2.0, h / 2.0

        step = max(1, int(np.sqrt(h * w / DOWNSAMPLE_PER_FRAME)))
        ys = np.arange(0, h, step)
        xs = np.arange(0, w, step)
        yy, xx = np.meshgrid(ys, xs, indexing="ij")

        d = depth[yy, xx].astype(np.float64)
        # Scale relative depth to approximate metric (1-5m range)
        d = d * 4.0 + 0.5

        valid = (d > 0.1) & (d < 10.0) & np.isfinite(d)
        z = d[valid]
        x = (xx[valid].astype(np.float64) - cx) * z / fx
        y = (yy[valid].astype(np.float64) - cy) * z / fy

        rgb = cv2.resize(frame, (w, h)) if frame.shape[:2] != (h, w) else frame
        colors = rgb[yy[valid], xx[valid]]
        r = colors[:, 2].astype(np.float64)
        g = colors[:, 1].astype(np.float64)
        b = colors[:, 0].astype(np.float64)

        # Transform to global frame
        local = np.column_stack([x, -y, -z, np.ones(len(z))])
        global_pts = (self._pose @ local.T).T[:, :3]

        cloud = np.column_stack([global_pts[:, 0], global_pts[:, 1], global_pts[:, 2], r, g, b])
        return cloud.astype(np.float32)

    def _estimate_motion(self, frame: np.ndarray, depth: np.ndarray) -> None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (depth.shape[1], depth.shape[0]))
        kps, des = self._orb.detectAndCompute(gray, None)

        if self._prev_gray is None or self._prev_des is None or des is None or len(kps) < 10:
            self._prev_gray = gray
            self._prev_kps = kps
            self._prev_des = des
            self._prev_depth = depth
            return

        try:
            matches = self._bf.match(self._prev_des, des)
            if len(matches) < 8:
                self._prev_gray = gray
                self._prev_kps = kps
                self._prev_des = des
                self._prev_depth = depth
                return

            matches = sorted(matches, key=lambda m: m.distance)[:200]

            h, w = depth.shape[:2]
            fx = fy = w * 0.8
            cx, cy = w / 2.0, h / 2.0

            pts3d = []
            pts2d = []
            for m in matches:
                prev_kp = self._prev_kps[m.queryIdx]
                curr_kp = kps[m.trainIdx]
                px, py = int(prev_kp.pt[0]), int(prev_kp.pt[1])
                if 0 <= px < w and 0 <= py < h:
                    d = float(self._prev_depth[py, px])
                    d = d * 4.0 + 0.5
                    if 0.1 < d < 10.0:
                        x3 = (px - cx) * d / fx
                        y3 = (py - cy) * d / fy
                        pts3d.append([x3, -y3, -d])
                        pts2d.append(list(curr_kp.pt))

            if len(pts3d) >= 6:
                pts3d = np.array(pts3d, dtype=np.float64)
                pts2d = np.array(pts2d, dtype=np.float64)
                cam_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

                ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                    pts3d, pts2d, cam_matrix, None,
                    iterationsCount=100, reprojectionError=5.0, flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if ok and inliers is not None and len(inliers) >= 4:
                    R, _ = cv2.Rodrigues(rvec)
                    T = np.eye(4, dtype=np.float64)
                    T[:3, :3] = R
                    T[:3, 3] = tvec.flatten()
                    T_inv = np.linalg.inv(T)
                    self._pose = self._pose @ T_inv
        except Exception:
            pass

        self._prev_gray = gray
        self._prev_kps = kps
        self._prev_des = des
        self._prev_depth = depth

    def _detections_to_markers(self, detections: list[dict], depth: np.ndarray) -> list[dict]:
        h, w = depth.shape[:2]
        fx = fy = w * 0.8
        cx, cy = w / 2.0, h / 2.0
        markers = []

        for det in detections[:20]:
            bbox = det.get("bbox", [0, 0, 0, 0])
            x1, y1, x2, y2 = bbox
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0

            # Map to depth resolution
            sx = w / max(1, (x2 - x1 + center_x * 2) / 2)  # approximate
            px = int(center_x * w / max(1, int(det.get("_frame_w", w))))
            py = int(center_y * h / max(1, int(det.get("_frame_h", h))))
            px = min(max(0, px), w - 1)
            py = min(max(0, py), h - 1)

            d = float(depth[py, px]) * 4.0 + 0.5
            if d < 0.1 or d > 10.0:
                continue

            x3 = (px - cx) * d / fx
            y3 = (py - cy) * d / fy
            z3 = -d

            local = np.array([x3, -y3, z3, 1.0])
            glob = self._pose @ local

            markers.append({
                "label": det.get("label", "?"),
                "x": float(glob[0]),
                "y": float(glob[1]),
                "z": float(glob[2]),
                "confidence": det.get("confidence", 0),
                "track_id": det.get("track_id", -1),
            })

        return markers
