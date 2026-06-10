"""Behavior runtime tests — load, run, hot-reload, halt."""
import time
import textwrap

import pytest

from roborun.behaviors import BehaviorRunner, Robot, Thing, write_examples


@pytest.fixture
def runner(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    r = BehaviorRunner()
    r.dirs = [tmp_path / "behaviors"]
    yield r
    r.stop()


def _write(tmp_path, body):
    d = tmp_path / "behaviors"
    d.mkdir(exist_ok=True)
    (d / "b.py").write_text(textwrap.dedent(body))


def _wait(predicate, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


COUNTER = """\
    from roborun.behaviors import behavior

    @behavior(hz=50)
    def ticker(robot):
        robot.state["n"] = robot.state.get("n", 0) + 1
"""


def test_loads_and_runs(runner, tmp_path):
    _write(tmp_path, COUNTER)
    runner._scan()
    assert _wait(lambda: runner.statuses() and runner.statuses()[0]["runs"] > 3)
    s = runner.statuses()[0]
    assert s["name"] == "ticker" and s["errors"] == 0


def test_hot_reload_swaps_function(runner, tmp_path):
    _write(tmp_path, COUNTER)
    runner._scan()
    assert _wait(lambda: runner.statuses())
    _write(tmp_path, COUNTER.replace("ticker", "ticker2"))
    runner._scan()
    assert _wait(lambda: runner.statuses() and runner.statuses()[0]["name"] == "ticker2")
    assert len(runner.statuses()) == 1


def test_broken_file_does_not_crash(runner, tmp_path):
    _write(tmp_path, "this is not python (")
    runner._scan()
    assert runner.statuses() == []
    _write(tmp_path, COUNTER)  # fix it → loads on next scan
    runner._scan()
    assert _wait(lambda: runner.statuses())


def test_erroring_behavior_keeps_running(runner, tmp_path):
    _write(tmp_path, """\
        from roborun.behaviors import behavior

        @behavior(hz=50)
        def crasher(robot):
            raise ValueError("boom")
    """)
    runner._scan()
    assert _wait(lambda: runner.statuses() and runner.statuses()[0]["errors"] >= 1)
    assert "boom" in runner.statuses()[0]["last_error"]


def test_disable_stops_loop(runner, tmp_path):
    _write(tmp_path, COUNTER)
    runner._scan()
    assert _wait(lambda: runner.statuses() and runner.statuses()[0]["runs"] > 0)
    assert runner.set_enabled("ticker", False)
    runs = runner.statuses()[0]["runs"]
    time.sleep(0.3)
    assert runner.statuses()[0]["runs"] <= runs + 1


def test_robot_handle_safe_without_server():
    robot = Robot("test")
    assert robot.see() == []          # no webcam singleton → empty, no crash
    robot.move(forward=99.0)          # clamped, no actuator → logged only
    robot.stop()
    robot.say("hello")
    robot.log("note", key="value")


def test_thing_normalization():
    t = Thing({"bbox": [320, 180, 960, 540], "label": "person",
               "confidence": 0.9, "track_id": 3}, fw=1280, fh=720)
    assert t.cx == 0.5 and t.cy == 0.5
    assert t.w == 0.5 and t.h == 0.5
    assert t.label == "person" and t.track_id == 3


def test_write_examples(tmp_path):
    target = write_examples(tmp_path / "behaviors")
    assert target is not None
    names = {p.name for p in target.glob("*.py")}
    assert {"follow_person.py", "patrol.py", "heartbeat.py"} <= names
    assert write_examples(tmp_path / "behaviors") is None  # second call: no-op
