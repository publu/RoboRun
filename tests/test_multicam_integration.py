"""Multi-camera RUNTIME end-to-end on synthetic video (no physical camera).

Runs two real WebcamPipeline instances on generated mp4 files through the
CameraManager, then taps them into a recorder on per-source channels. Proves the
multi-camera runtime works with real pipelines + real YOLO — the only thing a
physical camera adds is the pixels.
"""
from __future__ import annotations

import time

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("ultralytics")


def _make_video(path, n=20, w=160, h=120, color=(0, 200, 0)):
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vw = cv2.VideoWriter(str(path), fourcc, 15.0, (w, h))
    for i in range(n):
        frame = np.full((h, w, 3), 30, np.uint8)
        x = (i * 6) % (w - 30)
        cv2.rectangle(frame, (x, 40), (x + 30, 80), color, -1)
        vw.write(frame)
    vw.release()


def test_two_synthetic_cameras_record_per_source(tmp_path):
    from roborun.webcam import WebcamPipeline
    from roborun.cameras import CameraManager
    from roborun.recorder import RunRecorder

    f1, f2 = tmp_path / "front.mp4", tmp_path / "rear.mp4"
    _make_video(f1, color=(0, 200, 0))
    _make_video(f2, color=(0, 0, 200))

    mgr = CameraManager()
    p1, p2 = WebcamPipeline(), WebcamPipeline()
    # each real pipeline opens its own synthetic source (YOLO only, lightweight)
    assert p1.start(str(f1), models=["yolo"])["ok"]
    assert p2.start(str(f2), models=["yolo"])["ok"]
    mgr.register("front", p1)
    mgr.register("rear", p2)
    try:
        # poll until both real pipelines have produced a frame (YOLO load + 2 caps)
        deadline = time.time() + 15
        while time.time() < deadline:
            if mgr.snapshot("front") is not None and mgr.snapshot("rear") is not None:
                break
            time.sleep(0.3)
        assert mgr.snapshot("front") is not None
        assert mgr.snapshot("rear") is not None

        rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.05)
        contributed = mgr.record_into(rec)
        seal = rec.close(do_anchor=False)
    finally:
        mgr.stop_all()

    assert contributed == 2
    mc = seal["message_counts"]
    assert mc.get("/camera/front") and mc.get("/camera/rear")  # per-source channels
