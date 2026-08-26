"""Runnable scenarios: register → run → score → aggregate as a suite.

A fake handle is injected so these run headlessly (no browser arena), and the
control loop uses a no-op sleep so they finish instantly.
"""
from __future__ import annotations

import pytest

from roborun import scenario_defs as D


@pytest.fixture(autouse=True)
def _isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    # Start each test with a clean registry.
    monkeypatch.setattr(D, "_REGISTRY", {})


class FakeRobot:
    """Minimal handle: drives toward a goal a fixed step per tick."""

    def __init__(self, start=(0.0, 0.0), step=0.5):
        self.x, self.z = start
        self.step = step
        self.moves = 0

    def pose(self):
        return {"x": self.x, "z": self.z, "heading": 0.0}

    def goto(self, gx, gz, tol=0.4):
        self.moves += 1
        # Move a step toward the goal; report arrival when within tol.
        self.x += self.step if self.x < gx else 0.0
        return abs(self.x - gx) <= tol and abs(self.z - gz) <= tol

    def stop(self):
        return None


_NO_SLEEP = lambda _s: None


def test_register_and_list_defs():
    @D.scenario_def("noop", suite="s1", tags=["t"], params={"a": 1})
    def noop(ctx):
        ctx.run.passed()

    defs = D.list_defs()
    assert len(defs) == 1 and defs[0]["name"] == "noop"
    assert defs[0]["suite"] == "s1"
    assert D.suites_defined() == ["s1"]


def test_run_scenario_scores_pass():
    @D.scenario_def("reach", suite="nav", params={"goal": (1.0, 0.0), "tol": 0.4})
    def reach(ctx):
        gx, gz = ctx.params["goal"]
        ok = ctx.until(lambda: ctx.robot.goto(gx, gz, tol=ctx.params["tol"]),
                       sleep=_NO_SLEEP)
        ctx.run.metric("moves", ctx.robot.moves)
        ctx.run.passed() if ok else ctx.run.failed("timeout")

    rec = D.run_scenario("reach", robot=FakeRobot(step=0.5))
    assert rec["outcome"] == "passed"
    assert rec["suite"] == "nav"
    assert rec["metrics"]["moves"] >= 1


def test_run_scenario_times_out_to_failure():
    @D.scenario_def("unreachable", params={"goal": (99.0, 0.0)}, timeout_s=0.0)
    def unreachable(ctx):
        ok = ctx.until(lambda: ctx.robot.goto(99.0, 0.0), sleep=_NO_SLEEP)
        (ctx.run.passed if ok else ctx.run.failed)("ok" if ok else "never arrived")

    rec = D.run_scenario("unreachable", robot=FakeRobot(step=0.01))
    assert rec["outcome"] == "failed"
    assert "never arrived" in rec["reason"]


def test_unknown_scenario_raises():
    with pytest.raises(KeyError):
        D.run_scenario("ghost", robot=FakeRobot())


def test_run_suite_aggregates_pass_rate():
    @D.scenario_def("easy", suite="mix", params={"goal": (0.5, 0.0)})
    def easy(ctx):
        ok = ctx.until(lambda: ctx.robot.goto(0.5, 0.0), sleep=_NO_SLEEP)
        ctx.run.passed() if ok else ctx.run.failed("missed")

    @D.scenario_def("hard", suite="mix", params={"goal": (99.0, 0.0)}, timeout_s=0.0)
    def hard(ctx):
        ok = ctx.until(lambda: ctx.robot.goto(99.0, 0.0), sleep=_NO_SLEEP)
        ctx.run.passed() if ok else ctx.run.failed("too far")

    summary = D.run_suite("mix", robot=FakeRobot(step=0.5))
    assert summary["runs"] == 2
    assert summary["passed"] == 1
    assert summary["pass_rate"] == 0.5

    # And the per-scenario records are now in the registry the UI reads.
    from roborun import scenario as S
    assert S.suite_summary("mix")["runs"] == 2


def test_run_suite_unknown_is_graceful():
    out = D.run_suite("nope", robot=FakeRobot())
    assert out["runs"] == 0 and "error" in out
