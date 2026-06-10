"""Robot-camera perception: frames in, robot.see() out — no robot needed."""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

import roborun.ros_camera as rc
from roborun.ros_camera import RosCameraPipeline


class FakeDetector:
    def detect(self, frame, track=True):
        class D:
            def to_dict(self):
                return {"bbox": [100, 100, 300, 400], "label": "person",
                        "confidence": 0.9, "track_id": 1}
        return [D()]


def _jpeg(w=640, h=480):
    frame = np.full((h, w, 3), 90, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", frame)
    assert ok
    return buf.tobytes()


def test_ingest_decodes_and_detects(tmp_path, monkeypatch):
    monkeypatch.setattr(rc, "FRAME_PATH", tmp_path / "frame.jpg")
    cam = RosCameraPipeline()
    cam._detector = FakeDetector()
    cam.ingest_jpeg(_jpeg())
    assert cam.is_active()
    assert cam.snapshot().shape == (480, 640, 3)
    assert cam.get_detections()[0]["label"] == "person"
    assert (tmp_path / "frame.jpg").exists()   # deck + ask(image=True) see the robot's view


def test_garbage_frames_ignored(monkeypatch, tmp_path):
    monkeypatch.setattr(rc, "FRAME_PATH", tmp_path / "frame.jpg")
    cam = RosCameraPipeline()
    cam.ingest_jpeg(b"not a jpeg")
    assert not cam.is_active()


def test_robot_see_prefers_robot_camera(monkeypatch, tmp_path):
    monkeypatch.setattr(rc, "FRAME_PATH", tmp_path / "frame.jpg")
    cam = RosCameraPipeline()
    cam._detector = FakeDetector()
    cam.ingest_jpeg(_jpeg())
    monkeypatch.setattr(rc, "_pipeline", cam)

    import roborun.arena as arena_mod
    monkeypatch.setattr(arena_mod, "_arena", None)  # arena inactive

    from roborun.behaviors import Robot
    things = Robot("t").see("person")
    assert len(things) == 1
    t = things[0]
    assert abs(t.cx - 200 / 640) < 0.01   # normalized to the ROBOT's frame dims
    assert abs(t.h - 300 / 480) < 0.01
