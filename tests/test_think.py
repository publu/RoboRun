"""Async LLM on the handle: think/thought, delegate-with-tools."""
import time

import roborun.llm as llm
import roborun.ros_mcp as ros_mcp
from roborun.behaviors import Robot, _parse_action


def _wait_thought(robot, key, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        ans = robot.thought(key)
        if ans is not None:
            return ans
        time.sleep(0.02)
    raise AssertionError("thought never arrived")


def test_think_is_nonblocking_and_dedupes(monkeypatch):
    started = []

    def slow_complete(prompt, **kw):
        started.append(prompt)
        time.sleep(0.15)
        return "42"
    monkeypatch.setattr(llm, "complete", slow_complete)

    r = Robot("t1")
    t0 = time.monotonic()
    assert r.think("meaning of life?") is True
    assert time.monotonic() - t0 < 0.1      # did not block the loop
    assert r.thinking()
    assert r.think("again?") is False        # pending: re-call is a no-op
    assert r.thought() is None               # not ready yet
    assert _wait_thought(r, "default") == "42"
    assert r.thought() is None               # popped
    assert started == ["meaning of life?"]   # deduped while pending


def test_think_failure_becomes_answer(monkeypatch):
    monkeypatch.setattr(llm, "complete",
                        lambda *a, **k: (_ for _ in ()).throw(RuntimeError("no key")))
    r = Robot("t2")
    r.think("x")
    assert "[think failed" in _wait_thought(r, "default")


def test_parse_action():
    assert _parse_action('{"tool": "move", "args": {"linear_x": 1}}')["tool"] == "move"
    assert _parse_action('```json\n{"done": "ok"}\n```')["done"] == "ok"
    assert _parse_action('Sure! {"done": "report"} hope that helps')["done"] == "report"
    assert _parse_action("no json here") is None


def test_delegate_calls_tools_then_reports(monkeypatch):
    calls = []
    replies = iter([
        '{"tool": "estop", "args": {}}',
        '{"done": "stopped the robot"}',
    ])
    monkeypatch.setattr(llm, "complete", lambda *a, **k: next(replies))
    monkeypatch.setattr(ros_mcp, "handle_tool_call",
                        lambda name, args: calls.append((name, args)) or {"ok": True})

    r = Robot("t3")
    assert r.delegate("stop everything") is True
    report = _wait_thought(r, "delegate")
    assert report == "stopped the robot"
    assert calls == [("estop", {})]


def test_delegate_step_cap(monkeypatch):
    monkeypatch.setattr(llm, "complete",
                        lambda *a, **k: '{"tool": "estop", "args": {}}')
    monkeypatch.setattr(ros_mcp, "handle_tool_call", lambda n, a: {"ok": True})
    r = Robot("t4")
    r.delegate("loop forever", max_steps=2)
    assert "stopped after 2 steps" in _wait_thought(r, "delegate")
