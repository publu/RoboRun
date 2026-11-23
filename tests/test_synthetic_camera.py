"""Synthetic camera: a hardware-free camera through the full perception path."""
from __future__ import annotations

import time

import pytest

pytest.importorskip("cv2")

from roborun.synthetic_camera import SyntheticCamera
from roborun.cameras import CameraManager
from roborun.recorder import RunRecorder


def test_produces_frames_and_groundtruth_detections():
    cam = SyntheticCamera(noise=4.0, distortion=0.05, label="person")
    cam.start()
    try:
        time.sleep(0.4)
        f = cam.snapshot()
        assert f is not None and f.shape == (240, 320, 3)
        dets = cam.get_detections()
        assert dets and dets[0]["label"] == "person"
    finally:
        cam.stop()


def test_two_synthetic_cams_record_per_source(tmp_path):
    mgr = CameraManager()
    mgr.register("front", SyntheticCamera(label="person"))
    mgr.register("rear", SyntheticCamera(label="car"))
    mgr.start_all()
    try:
        time.sleep(0.4)
        rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.05)
        n = mgr.record_into(rec)
        seal = rec.close(do_anchor=False)
    finally:
        mgr.stop_all()
    assert n == 2
    mc = seal["message_counts"]
    assert mc.get("/camera/front") and mc.get("/camera/rear")
    assert mc.get("/detections/front") and mc.get("/detections/rear")


def test_setup_script_exists_and_is_sh(tmp_path):
    from pathlib import Path
    p = Path(__file__).resolve().parent.parent / "scripts" / "setup_gz.sh"
    assert p.exists()
    assert "osrf/simulation" in p.read_text()
