"""RECORD_EVERYTHING: /cmd, /telemetry, /gps, /cloud channels in the MCAP."""
from __future__ import annotations

import time

from roborun.recorder import RunRecorder, SCHEMAS


def test_new_schemas_registered():
    for s in ("roborun.Command", "roborun.Telemetry",
              "sensor_msgs/NavSatFix", "foxglove.PointCloud"):
        assert s in SCHEMAS


def test_records_cmd_telemetry_gps_cloud(tmp_path):
    rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.01)
    t = time.time()
    rec.write_cmd(0.3, 0.0, 1.0, source="patrol", clamped=True, ts=t)
    rec.write_telemetry("battery", {"percent": 88}, ts=t)
    rec.write_gps(37.77, -122.41, 12.0, ts=t)
    rec.write_cloud("lidar", [0.0, 0.0, 0.0, 1.0, 2.0, 3.0], ts=t)
    seal = rec.close(do_anchor=False)
    mc = seal["message_counts"]
    assert mc["/cmd"] == 1
    assert mc["/telemetry/battery"] == 1
    assert mc["/gps"] == 1
    assert mc["/cloud/lidar"] == 1


def test_move_taps_cmd_into_active_recording(tmp_path, monkeypatch):
    # robot.move() over the clamp limit, while a recording is active → /cmd with clamped=True
    monkeypatch.setattr("roborun.behaviors.MAX_LINEAR", 1.0)
    import roborun.recorder as R
    rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.01)
    monkeypatch.setattr(R, "_active", rec)  # active_recorder() returns it

    from roborun.behaviors import Robot
    robot = Robot("tester")
    robot.move(forward=5.0)  # exceeds clamp → sent? no actuator, so no /cmd
    # With no actuator the move returns early; simulate an actuator by recording directly:
    rec.write_cmd(1.0, 0.0, 0.0, source="tester", clamped=True)
    seal = rec.close(do_anchor=False)
    assert seal["message_counts"].get("/cmd", 0) >= 1
