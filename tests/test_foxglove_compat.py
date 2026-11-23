"""Foxglove interop: our MCAP messages match Foxglove's well-known schema shapes,
so RoboRun runs open natively in Foxglove Studio (MCAP is their open standard).

Locks the field shapes in CI — a change that breaks Foxglove rendering fails here.
"""
from __future__ import annotations

import base64
import json
import time

from roborun.recorder import RunRecorder


def _messages(tmp_path):
    rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.01)
    t = time.time()
    rec.write_camera(b"\xff\xd8jpeg", name="front", ts=t)
    rec.write_pose(1.0, 2.0, 0.5, heading=0.3, ts=t)
    rec.write_scan([1.0, 2.0, 3.0], 0.0, 0.0, 0.0, ts=t)
    rec.close(do_anchor=False)
    from mcap.reader import make_reader
    out = {}
    with open(rec.mcap_path, "rb") as fh:
        for _s, channel, message in make_reader(fh).iter_messages():
            if channel.message_encoding == "json":
                out.setdefault(channel.topic, json.loads(message.data))
    return out


def _is_ts(v):
    return isinstance(v, dict) and "sec" in v and "nsec" in v


def test_compressed_image_matches_foxglove(tmp_path):
    m = _messages(tmp_path)["/camera/front"]
    assert _is_ts(m["timestamp"])            # foxglove Time {sec,nsec}
    assert isinstance(m["frame_id"], str)
    assert m["format"] == "jpeg"
    base64.b64decode(m["data"])              # bytes field is base64


def test_pose_in_frame_matches_foxglove(tmp_path):
    m = _messages(tmp_path)["/pose"]
    assert _is_ts(m["timestamp"]) and isinstance(m["frame_id"], str)
    pos = m["pose"]["position"]; ori = m["pose"]["orientation"]
    assert {"x", "y", "z"} <= set(pos) and {"x", "y", "z", "w"} <= set(ori)


def test_laser_scan_matches_foxglove(tmp_path):
    m = _messages(tmp_path)["/scan"]
    assert _is_ts(m["timestamp"]) and isinstance(m["frame_id"], str)
    assert "start_angle" in m and "end_angle" in m and isinstance(m["ranges"], list)


def test_schemas_use_foxglove_wellknown_names():
    from roborun.recorder import SCHEMAS
    for name in ("foxglove.CompressedImage", "foxglove.CompressedVideo",
                 "foxglove.PoseInFrame", "foxglove.LaserScan",
                 "foxglove.SceneUpdate", "foxglove.PointCloud"):
        assert name in SCHEMAS
