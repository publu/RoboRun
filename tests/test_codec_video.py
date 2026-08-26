"""Codec video channel (RECORD_EVERYTHING/PERF #5): H.264 << per-frame JPEG."""
from __future__ import annotations

import time

import numpy as np
import pytest

pytest.importorskip("av")

import cv2

from roborun.recorder import RunRecorder, SCHEMAS


def _frames(n=60, w=128, h=96):
    # a moving gradient — realistic temporal redundancy (where codec wins)
    for i in range(n):
        img = np.zeros((h, w, 3), np.uint8)
        img[:, (i * 2) % w:] = (i % 255)
        yield img


def test_video_schema_registered():
    assert "foxglove.CompressedVideo" in SCHEMAS


def test_h264_is_smaller_than_jpeg(tmp_path):
    frames = list(_frames())
    # JPEG size
    jpeg_bytes = sum(len(cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 85])[1])
                     for f in frames)
    # H.264 via the recorder path
    rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.05)
    t0 = time.time()
    for i, f in enumerate(frames):
        rec.write_video(f, name="front", fps=30, ts=t0 + i / 30)
    seal = rec.close(do_anchor=False)
    assert seal["message_counts"].get("/camera/front", 0) >= 1  # video packets written
    h264_bytes = rec.mcap_path.stat().st_size
    # the codec stream (whole MCAP incl. overhead) beats the raw JPEG sum handily
    assert h264_bytes < jpeg_bytes
