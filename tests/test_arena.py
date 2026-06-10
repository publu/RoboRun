"""Arena bridge: command flow, freshness, detection handoff to robot.see()."""
import time

from roborun import arena as arena_mod
from roborun.arena import ArenaState


def test_inactive_until_browser_pushes():
    a = ArenaState()
    assert not a.is_active()
    a.update({"detections": [], "pose": {"x": 0, "z": 0}})
    assert a.is_active()


def test_cmd_roundtrip_and_stale_decay(monkeypatch):
    a = ArenaState()
    a.set_cmd(0.5, 0.0, -0.3)
    assert a.cmd() == {"forward": 0.5, "strafe": 0.0, "turn": -0.3}
    # a behavior that stops ticking must not leave the dog walking
    t = time.monotonic()
    monkeypatch.setattr(time, "monotonic", lambda: t + 2.0)
    assert a.cmd() == {"forward": 0.0, "strafe": 0.0, "turn": 0.0}


def test_robot_see_uses_arena_when_active(monkeypatch):
    a = ArenaState()
    a.update({"detections": [
        {"label": "red door", "confidence": 0.95, "bbox": [600, 250, 680, 470]},
        {"label": "obstacle", "confidence": 1.0, "bbox": [400, 200, 880, 520]},
    ]})
    monkeypatch.setattr(arena_mod, "_arena", a)
    from roborun.behaviors import Robot
    things = Robot("test").see("red door")
    assert len(things) == 1
    t = things[0]
    assert abs(t.cx - 640 / 1280) < 0.01  # normalized to the virtual frame
    assert Robot("test").see("obstacle")[0].h > 0.4


def test_robot_move_drives_arena_when_active(monkeypatch):
    a = ArenaState()
    a.update({})  # browser alive
    monkeypatch.setattr(arena_mod, "_arena", a)
    from roborun.behaviors import Robot
    Robot("test").move(forward=0.8, turn=0.2)
    assert a.cmd()["forward"] == 0.8
