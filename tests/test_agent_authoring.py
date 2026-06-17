"""The deck agent's vibe-robotics loop: author → run → observe → iterate.

These cover the in-process FastRobotAgent tool surface that lets the command
bar write behaviors, watch them run, and read back failures — without an LLM
or a network. Each test drives _execute_fast_tool directly.
"""
from __future__ import annotations

import os

import pytest

from roborun import agent
from roborun.behaviors import BehaviorRunner


@pytest.fixture
def behaviors_dir(tmp_path, monkeypatch):
    """Run each test in a throwaway cwd with a live BehaviorRunner."""
    monkeypatch.chdir(tmp_path)
    runner = BehaviorRunner.get()
    # Re-point the singleton at the tmp behaviors/ dir and clear prior state.
    runner.dirs = [tmp_path / "behaviors"]
    runner._mtimes.clear()
    for loops in list(runner._loops.values()):
        for loop in loops:
            loop.halt()
    runner._loops.clear()
    runner.start()
    yield tmp_path


def _x(_tool, **args):
    return agent._execute_fast_tool(_tool, args)


def test_all_authoring_tools_are_declared():
    declared = {t["name"] for t in agent._FAST_TOOLS}
    assert {
        "list_behaviors", "read_behavior", "write_behavior",
        "set_behavior", "read_timeline", "arena_status",
    } <= declared


def test_write_then_it_runs_and_lists(behaviors_dir):
    src = (
        "from roborun.behaviors import behavior\n"
        "@behavior(hz=5)\n"
        "def wander(robot):\n"
        "    robot.move(forward=0.3)\n"
    )
    out = _x("write_behavior", name="wander", source=src)
    assert "running" in out
    assert (behaviors_dir / "behaviors" / "wander.py").exists()
    assert "wander" in _x("list_behaviors")


def test_write_surfaces_runtime_error_for_iteration(behaviors_dir):
    src = (
        "from roborun.behaviors import behavior\n"
        "@behavior(hz=20)\n"
        "def boom(robot):\n"
        "    raise ValueError('kaboom')\n"
    )
    out = _x("write_behavior", name="boom", source=src)
    assert "erroring" in out and "kaboom" in out
    timeline = _x("read_timeline", limit=10, contains="boom")
    assert "kaboom" in timeline


def test_write_rejects_invalid_name(behaviors_dir):
    out = _x("write_behavior", name="Bad Name", source="x")
    assert "Not written" in out


def test_read_behavior_round_trips_source(behaviors_dir):
    src = (
        "from roborun.behaviors import behavior\n"
        "@behavior(hz=2)\n"
        "def idle(robot):\n"
        "    robot.stop()\n"
    )
    _x("write_behavior", name="idle", source=src)
    body = _x("read_behavior", name="idle")
    assert "def idle(robot)" in body


def test_set_behavior_disable_then_enable(behaviors_dir):
    src = (
        "from roborun.behaviors import behavior\n"
        "@behavior(hz=5)\n"
        "def hold(robot):\n"
        "    robot.stop()\n"
    )
    _x("write_behavior", name="hold", source=src)
    assert "disable" in _x("set_behavior", name="hold", action="disable")
    assert "[stopped]" in _x("list_behaviors")
    assert "enable" in _x("set_behavior", name="hold", action="enable")
    assert "[running]" in _x("list_behaviors")


def test_set_behavior_unknown_name(behaviors_dir):
    assert "No behavior" in _x("set_behavior", name="ghost", action="enable")


def test_arena_status_when_closed(behaviors_dir):
    # No arena page open in a test process — must degrade, not raise.
    assert "arena" in _x("arena_status").lower()
