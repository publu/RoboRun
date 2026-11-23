"""Per-run telemetry series (Antioch Analyze view) from a real recorded MCAP."""
from __future__ import annotations

import math
import time

from roborun.recorder import RunRecorder
from roborun.run_series import run_series


def _record(tmp_path):
    rec = RunRecorder(robot_id="bot", root=tmp_path, checkpoint_interval=0.05)
    t0 = time.time()
    for i in range(20):
        ts = t0 + i * 0.1
        rec.write_pose(i * 0.1, 0.0, math.sin(i * 0.1), heading=0.0, ts=ts)
        rec.write_cmd(0.4, 0.0, 0.2 * math.sin(i), source="patrol", ts=ts)
        rec.write_scan([1.0 + 0.5 * math.sin(j) for j in range(36)],
                       i * 0.1, 0.0, 0.0, ts=ts)
    rec.close(do_anchor=False)
    return rec.run_id


def test_series_extracts_panels(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path.parent))
    # record under runs_root so _find_mcap locates it
    from roborun.events import runs_root
    root = runs_root()
    rec = RunRecorder(robot_id="bot", root=root, checkpoint_interval=0.05)
    t0 = time.time()
    for i in range(20):
        ts = t0 + i * 0.1
        rec.write_pose(i * 0.1, 0.0, 0.0, heading=0.0, ts=ts)
        rec.write_cmd(0.4, 0.0, 0.1, source="p", ts=ts)
        rec.write_scan([2.0] * 36, i * 0.1, 0.0, 0.0, ts=ts)
    rec.close(do_anchor=False)

    s = run_series(rec.run_id)
    assert s["ok"]
    assert len(s["trajectory"]) == 20         # actual path
    assert len(s["velocity"]) == 20           # falls back to /cmd
    assert len(s["clearance"]) == 20          # min range per scan
    assert len(s["scan"]) == 36               # latest lidar sweep
    assert s["duration_s"] > 0
    assert s["counts"]["/pose"] == 20 and s["counts"]["/cmd"] == 20


def test_missing_run(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    s = run_series("run_does_not_exist")
    assert s["ok"] is False


def test_frame_at_returns_nearest_jpeg(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    from roborun.events import runs_root
    from roborun.run_series import run_series, frame_at
    rec = RunRecorder(robot_id="bot", root=runs_root(), checkpoint_interval=0.05)
    t0 = time.time()
    for i in range(6):
        rec.write_camera(b"\xff\xd8" + bytes([i]) * 40, name="front", ts=t0 + i)
        rec.write_pose(i * 0.1, 0.0, 0.0, ts=t0 + i)
    rec.close(do_anchor=False)

    s = run_series(rec.run_id)
    assert len(s["frames"]) == 6        # scrubber timeline
    jpeg = frame_at(rec.run_id, t0 + 3.1)
    assert jpeg is not None and jpeg.startswith(b"\xff\xd8")
    assert jpeg[2] == 3                  # nearest is the i=3 frame
    assert frame_at("nope", 0) is None
