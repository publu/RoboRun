"""Spatial fusion — put every sensing into one environment frame (specs 05/09).

Detections and clouds are captured at a sensor pose (a robot's odometry, or a
fixed camera's registered extrinsics). These helpers transform local readings
into the environment's world frame and build the geometry the payoff view draws
(camera frusta). Pure math, no deps beyond stdlib — unit-testable.
"""
from __future__ import annotations

import math
from typing import Any


def transform_point(local: dict, pose: dict) -> dict:
    """A point in a sensor's local frame → the environment world frame, given the
    sensor pose {x,y[,z],yaw}. 2D rotation about yaw + translation."""
    yaw = float(pose.get("yaw", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = float(local.get("x", 0.0)), float(local.get("y", 0.0))
    return {"x": float(pose.get("x", 0.0)) + c * lx - s * ly,
            "y": float(pose.get("y", 0.0)) + s * lx + c * ly,
            "z": float(pose.get("z", 0.0)) + float(local.get("z", 0.0))}


def sensor_pose(camera: dict | None, robot_pose: dict | None) -> dict:
    """The world pose to anchor a sensor's detections (spec 05 P1):
    a fixed camera uses its registered placement; a robot camera uses the robot's
    live pose. Falls back to the origin."""
    if camera and camera.get("kind") == "fixed" and camera.get("placement"):
        return camera["placement"]
    if robot_pose:
        return robot_pose
    return {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}


def camera_frustum(placement: dict, fov: float = 1.2, length: float = 3.0) -> list[dict]:
    """The two far corners + apex of a camera's view cone, in world coords — what
    the payoff view draws to show where each camera looks (spec 09 P4)."""
    yaw = float(placement.get("yaw", 0.0))
    apex = {"x": float(placement.get("x", 0.0)), "y": float(placement.get("y", 0.0))}
    left = transform_point({"x": length, "y": math.tan(fov / 2) * length}, placement)
    right = transform_point({"x": length, "y": -math.tan(fov / 2) * length}, placement)
    return [apex, {"x": left["x"], "y": left["y"]}, {"x": right["x"], "y": right["y"]}]


def cluster_tracks(observations: list[dict], radius: float = 1.5) -> list[dict]:
    """Promote per-frame detections to environment-level object tracks (spec 05
    P3): merge same-label detections within `radius` (env-frame) into one track
    with a running-mean centroid, count, and first/last seen. `observations` are
    SpatialMemoryStore rows ({x,y,ts,detections[]})."""
    tracks: list[dict[str, Any]] = []
    for r in observations:
        x, y, ts = r.get("x"), r.get("y"), (r.get("ts") or 0.0)
        if x is None or y is None:
            continue
        for d in (r.get("detections") or []):
            label = d.get("label") or "object"
            hit = None
            for t in tracks:
                if t["label"] == label and math.hypot(t["x"] - x, t["y"] - y) <= radius:
                    hit = t
                    break
            if hit:
                hit["count"] += 1
                n = hit["count"]
                hit["x"] += (x - hit["x"]) / n
                hit["y"] += (y - hit["y"]) / n
                hit["last_ts"] = max(hit["last_ts"], ts)
                hit["first_ts"] = min(hit["first_ts"], ts)
            else:
                tracks.append({"label": label, "x": float(x), "y": float(y),
                               "count": 1, "first_ts": ts, "last_ts": ts})
    tracks.sort(key=lambda t: -t["count"])
    return tracks
