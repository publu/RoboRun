"""VLA adapter: any (image, instruction)->action model drives the handle."""
from __future__ import annotations

import pytest

from roborun.vla import register_vla, load_vla, list_vla, VLAPolicy


class FakeRobot:
    def __init__(self, frame=b"\xff\xd8jpeg"):
        self._frame = frame
        self.moved = None
        self.stopped = False
    def frame_jpeg(self):
        return self._frame
    def move(self, forward=0, strafe=0, turn=0, climb=0):
        self.moved = (forward, strafe, turn, climb)
    def stop(self):
        self.stopped = True


def test_register_and_load():
    register_vla("stub", lambda jpeg, instr: {"forward": 0.5, "turn": -0.2})
    assert "stub" in list_vla()
    assert callable(load_vla("stub"))


def test_unknown_vla_raises_clear():
    with pytest.raises(RuntimeError) as e:
        load_vla("groot_not_installed")
    assert "register_vla" in str(e.value)


def test_policy_infers_and_moves():
    register_vla("forward_bot", lambda jpeg, instr: {"forward": 0.6})
    r = FakeRobot()
    action = VLAPolicy("forward_bot").step(r, "drive forward")
    assert action["forward"] == 0.6 and action["turn"] == 0.0   # subset → others 0
    assert r.moved == (0.6, 0.0, 0.0, 0.0)


def test_policy_no_frame_stops():
    register_vla("any", lambda jpeg, instr: {"forward": 1.0})
    r = FakeRobot(frame=None)
    assert VLAPolicy("any").step(r, "go") == {}
    assert r.stopped and r.moved is None


def test_policy_accepts_callable_directly():
    p = VLAPolicy(lambda jpeg, instr: {"turn": 0.3})
    assert p.infer(b"x", "spin")["turn"] == 0.3
