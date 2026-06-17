"""LOCAL_SIM Phase 1: a MuJoCo-only session senses through the handle.

robot.pose()/lidar() must read the running sim's MjData (via SimBackend) when no
arena is open — previously they returned None/[] in a sim-only session.
"""
from __future__ import annotations

import time

import pytest


@pytest.fixture()
def running_sim():
    pytest.importorskip("mujoco")
    from roborun.simulator import SimulatorRunner
    r = SimulatorRunner()
    r.start("unitree_go1", render=False)
    time.sleep(0.6)
    yield r
    r.stop(); time.sleep(0.3)


def test_handle_pose_reads_sim(running_sim, monkeypatch):
    # point the singleton getter at our running sim
    import roborun.behaviors as B
    monkeypatch.setattr("roborun.routes._singletons.get_simulator",
                        lambda: running_sim, raising=False)
    from roborun.behaviors import Robot
    robot = Robot("t")
    p = robot.pose()
    assert p is not None and set(["x", "z", "heading"]).issubset(p)
    lid = robot.lidar()
    assert len(lid) == 36


def test_handle_blind_without_sim(monkeypatch):
    # no arena, no sim, no ros → pose None, lidar [] (go blind loudly, not crash)
    monkeypatch.setattr("roborun.routes._singletons.get_simulator",
                        lambda: type("S", (), {"is_running": False})(), raising=False)
    from roborun.behaviors import Robot
    robot = Robot("t2")
    assert robot.pose() is None
    assert robot.lidar() == []
