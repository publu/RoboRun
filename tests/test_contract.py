"""docs/SIM_SPEC.md contract: the primitives a policy reads are
schema-identical across backends — a behavior written against the arena
ports 1:1 to a connected robot because pose/lidar/see/move speak the
same units, frames, and shapes everywhere.
"""
from __future__ import annotations

import math
import time

from roborun.arena import get_arena
from roborun.behaviors import Robot
from roborun.ros_telemetry import RosTelemetryBridge


class _Bus:
    def push(self, *a, **k):
        pass


def odom_msg(x: float, y: float, yaw: float, z: float = 0.0) -> dict:
    return {
        "pose": {"pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": 0.0, "y": 0.0,
                            "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)},
        }},
        "twist": {"twist": {}},
    }


def scan_msg(ranges: list, amin: float = 0.0, rmax: float = 10.0) -> dict:
    return {"ranges": ranges, "angle_min": amin,
            "angle_increment": 2 * math.pi / len(ranges), "range_max": rmax}


def fresh_bridge() -> RosTelemetryBridge:
    return RosTelemetryBridge()


# ── pose: frame mapping ────────────────────────────────────────────────

def test_odom_maps_to_handle_frame():
    b = fresh_bridge()
    on_odom = b._make_handler("/odom", "nav_msgs/Odometry", _Bus())
    on_odom(odom_msg(1.5, 2.0, math.pi / 2, z=0.3))
    p = b.handle_pose()
    assert {"x", "z", "heading"} <= set(p)
    assert p["x"] == 1.5
    assert p["z"] == -2.0              # handle z = -ROS y
    assert abs(p["heading"] - math.pi / 2) < 1e-9
    assert p["y"] == 0.3               # altitude rides along for aerial


def test_locate_projection_holds_on_hardware_frame():
    """The handle's world projection (wx = x + cos(h)d, wz = z - sin(h)d)
    must land on the same physical point ROS says is d meters dead ahead.
    This is the identity goto()/approach()/locate() stand on."""
    b = fresh_bridge()
    on_odom = b._make_handler("/odom", "nav_msgs/Odometry", _Bus())
    d = 2.0
    for yaw in (0.0, 0.7, -2.1, 3.0):
        on_odom(odom_msg(1.0, -0.5, yaw))
        p = b.handle_pose()
        wx = p["x"] + math.cos(p["heading"]) * d
        wz = p["z"] - math.sin(p["heading"]) * d
        # the same point in ROS coords, mapped to the handle frame
        rx, ry = 1.0 + d * math.cos(yaw), -0.5 + d * math.sin(yaw)
        assert abs(wx - rx) < 1e-9
        assert abs(wz - (-ry)) < 1e-9


# ── lidar: schema and orientation ──────────────────────────────────────

def test_scan_resamples_to_arena_schema():
    b = fresh_bridge()
    on_scan = b._make_handler("/scan", "sensor_msgs/LaserScan", _Bus())
    ranges = [99.0] * 360                  # > range_max ⇒ no return
    ranges[0] = 2.0                        # dead ahead
    ranges[90] = 3.0                       # 90° CCW (left)
    ranges[91] = 2.5                       # same sector: nearest wins
    on_scan(scan_msg(ranges))
    scan = b.handle_lidar()
    assert len(scan) == 36
    assert scan[0] == 2.0
    assert scan[9] == 2.5                  # 90° = sector 9, min of the two
    assert scan[18] == 10.0                # blind ⇒ range_max


def test_partial_fov_scanner_reads_open_behind():
    b = fresh_bridge()
    on_scan = b._make_handler("/scan", "sensor_msgs/LaserScan", _Bus())
    # a 180° forward scanner: angle_min = -π/2, 181 rays
    msg = {"ranges": [4.0] * 181, "angle_min": -math.pi / 2,
           "angle_increment": math.pi / 180, "range_max": 12.0}
    on_scan(msg)
    scan = b.handle_lidar()
    assert scan[0] == 4.0                  # forward covered
    assert scan[18] == 12.0                # rear: no info = range_max


# ── staleness: never act on the past ───────────────────────────────────

def test_stale_odometry_goes_blind():
    b = fresh_bridge()
    on_odom = b._make_handler("/odom", "nav_msgs/Odometry", _Bus())
    on_odom(odom_msg(1.0, 1.0, 0.0))
    assert b.handle_pose() is not None
    time.sleep(0.02)
    assert b.handle_pose(max_age=0.01) is None
    assert b.handle_lidar(max_age=0.01) == []


# ── cross-backend schema equality through the Robot handle ─────────────

def test_robot_pose_schema_identical_across_backends():
    arena = get_arena()
    arena.update({"pose": {"x": 1.0, "z": 2.0, "y": 1.0, "heading": 0.5},
                  "lidar": [5.0] * 36, "detections": [], "level": {}})
    r = Robot("contract-test")
    sim = r.pose()
    assert {"x", "z", "heading"} <= set(sim)

    # arena goes quiet → the handle reads the robot's odometry instead
    arena._state_ts = 0.0
    from roborun.ros_telemetry import get_bridge
    b = get_bridge()
    on_odom = b._make_handler("/odom", "nav_msgs/Odometry", _Bus())
    on_odom(odom_msg(3.0, -1.0, 1.0))
    real = r.pose()
    assert real is not None
    assert {"x", "z", "heading"} <= set(real)
    assert set(sim) >= {"x", "z", "heading"} and set(real) >= {"x", "z", "heading"}
    assert real["x"] == 3.0 and real["z"] == 1.0

    on_scan = b._make_handler("/scan", "sensor_msgs/LaserScan", _Bus())
    on_scan(scan_msg([6.0] * 360))
    scan = r.lidar()
    assert len(scan) == 36 and all(v == 6.0 for v in scan)

    b.stop()                               # clear shared singleton state
