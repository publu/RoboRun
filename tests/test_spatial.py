"""Env-frame fusion + object tracks (platform specs 05/09)."""
import math

from roborun import spatial as sp
from roborun.spatial_memory import SpatialMemoryStore


def test_transform_point_rotation_translation():
    # 1m ahead of a sensor at (10,0) facing +90° → (10,1)
    p = sp.transform_point({"x": 1, "y": 0}, {"x": 10, "y": 0, "yaw": math.pi / 2})
    assert abs(p["x"] - 10) < 1e-6 and abs(p["y"] - 1) < 1e-6


def test_sensor_pose_fixed_vs_robot():
    cam = {"kind": "fixed", "placement": {"x": 5, "y": 3, "yaw": 0}}
    assert sp.sensor_pose(cam, {"x": 9, "y": 9})["x"] == 5      # fixed wins
    assert sp.sensor_pose({"kind": "robot"}, {"x": 9, "y": 9})["x"] == 9


def test_camera_frustum_apex_at_placement():
    f = sp.camera_frustum({"x": 5, "y": 3, "yaw": 0})
    assert f[0] == {"x": 5.0, "y": 3.0} and len(f) == 3


def test_cluster_tracks_merges_by_label_and_proximity():
    obs = [
        {"x": 1, "y": 1, "ts": 1, "detections": [{"label": "forklift"}]},
        {"x": 1.2, "y": 0.9, "ts": 2, "detections": [{"label": "forklift"}]},
        {"x": 20, "y": 20, "ts": 3, "detections": [{"label": "forklift"}]},
        {"x": 1, "y": 1, "ts": 2, "detections": [{"label": "person"}]},
    ]
    tr = sp.cluster_tracks(obs, radius=1.5)
    fk = [t for t in tr if t["label"] == "forklift"]
    assert len(fk) == 2 and max(t["count"] for t in fk) == 2
    assert any(t["label"] == "person" for t in tr)


def test_store_object_tracks(tmp_path):
    s = SpatialMemoryStore(db_path=tmp_path / "t.db")
    s.store(robot_id="r", ts=1.0, x=2.0, y=2.0,
            detections=[{"label": "pallet", "score": 0.9, "bbox": [0, 0, 9, 9]}])
    s.store(robot_id="r", ts=2.0, x=2.1, y=1.9,
            detections=[{"label": "pallet", "score": 0.8, "bbox": [0, 0, 9, 9]}])
    tracks = s.object_tracks(radius=1.0)
    pallets = [t for t in tracks if t["label"] == "pallet"]
    assert len(pallets) == 1 and pallets[0]["count"] == 2
