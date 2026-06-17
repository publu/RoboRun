"""Runnable scenarios — the executable half of the eval layer.

`scenario.py` *scores* a stretch of activity. This module makes scenarios
*launchable*: a `@scenario_def` is a named, suite-grouped function the agent (or
CI, or a person) can run on demand. Running one drives a robot handle to a
deadline, scores it via `scenario()`, and persists the sealed record — so the
Antioch "Accelerate" demo ("run the suite, find the failures, fix them") becomes
a real loop, not a screenshot.

    from roborun.scenario_defs import scenario_def

    @scenario_def("reach_goal", suite="navigation", tags=["nav"],
                  params={"goal": (3.0, 0.0), "tol": 0.4})
    def reach_goal(ctx):
        gx, gz = ctx.params["goal"]
        ok = ctx.until(lambda: ctx.robot.goto(gx, gz, tol=ctx.params["tol"]))
        p = ctx.robot.pose() or {}
        ctx.run.metric("final_pose", [p.get("x"), p.get("z")])
        (ctx.run.passed if ok else ctx.run.failed)(
            "reached goal" if ok else "timed out before goal")

The handle is the same one behaviors get, so a scenario written once runs on
every backend and embodiment by the SIM_SPEC contract. Tests inject a handle so
no browser arena is needed; in the deck the live arena is the handle.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable

from roborun.events import emit
from roborun.scenario import get_result, scenario

# ── registry ────────────────────────────────────────────────────────────────


@dataclass
class ScenarioDef:
    name: str
    fn: Callable[["ScenarioContext"], Any]
    suite: str | None = None
    tags: list[str] = field(default_factory=list)
    params: dict[str, Any] = field(default_factory=dict)
    timeout_s: float = 30.0
    doc: str = ""


_REGISTRY: dict[str, ScenarioDef] = {}


def scenario_def(name: str, *, suite: str | None = None,
                 tags: list[str] | None = None,
                 params: dict[str, Any] | None = None,
                 timeout_s: float = 30.0):
    """Register a runnable scenario. Decorate a `fn(ctx)` that scores via
    `ctx.run`."""
    def deco(fn: Callable[["ScenarioContext"], Any]):
        _REGISTRY[name] = ScenarioDef(
            name=name, fn=fn, suite=suite, tags=list(tags or []),
            params=dict(params or {}), timeout_s=timeout_s,
            doc=(fn.__doc__ or "").strip().split("\n")[0])
        return fn
    return deco


def list_defs(suite: str | None = None) -> list[dict[str, Any]]:
    """The catalog of runnable scenarios (not their past results)."""
    out = []
    for d in _REGISTRY.values():
        if suite and d.suite != suite:
            continue
        out.append({"name": d.name, "suite": d.suite, "tags": d.tags,
                    "params": d.params, "timeout_s": d.timeout_s, "doc": d.doc})
    return sorted(out, key=lambda x: (x["suite"] or "", x["name"]))


def suites_defined() -> list[str]:
    return sorted({d.suite for d in _REGISTRY.values() if d.suite})


# ── execution context ───────────────────────────────────────────────────────


class ScenarioContext:
    """Handed to a scenario function. Exposes the robot handle, merged params,
    the scoring run, and a deadline-aware control helper."""

    def __init__(self, robot: Any, run: Any, params: dict[str, Any],
                 deadline: float, tick_hz: float = 10.0) -> None:
        self.robot = robot
        self.run = run
        self.params = params
        self.deadline = deadline
        self._dt = 1.0 / max(1e-3, tick_hz)

    @property
    def expired(self) -> bool:
        return time.time() >= self.deadline

    @property
    def remaining(self) -> float:
        return max(0.0, self.deadline - time.time())

    def until(self, done: Callable[[], bool],
              on_tick: Callable[[], Any] | None = None,
              sleep: Callable[[float], None] = time.sleep) -> bool:
        """Tick until `done()` is truthy or the deadline passes. Returns
        whether `done()` succeeded. `sleep` is injectable for fast tests."""
        while not self.expired:
            if on_tick is not None:
                on_tick()
            if done():
                return True
            sleep(self._dt)
        return bool(done())


# ── runner ──────────────────────────────────────────────────────────────────


def _default_handle():
    from roborun.behaviors import Robot
    return Robot("scenario")


def run_scenario(name: str, robot: Any = None,
                 params: dict[str, Any] | None = None,
                 tick_hz: float = 10.0) -> dict[str, Any]:
    """Run one scenario by name; return its scored record. Raises KeyError if
    the scenario isn't registered."""
    d = _REGISTRY.get(name)
    if d is None:
        raise KeyError(f"no scenario named {name!r}")
    robot = robot if robot is not None else _default_handle()
    merged = {**d.params, **(params or {})}
    with scenario(d.name, tags=d.tags, params=merged, suite=d.suite) as run:
        ctx = ScenarioContext(robot=robot, run=run, params=merged,
                              deadline=time.time() + d.timeout_s, tick_hz=tick_hz)
        d.fn(ctx)
    # The record is persisted on context exit; read it back by id.
    rec = get_result(run.scenario_id)
    return rec or run.to_dict()


def run_suite(suite: str, robot: Any = None,
              tick_hz: float = 10.0) -> dict[str, Any]:
    """Run every scenario in a suite; return per-scenario records + the
    aggregate pass-rate. This is the Antioch "run the suite" action."""
    names = [d.name for d in _REGISTRY.values() if d.suite == suite]
    if not names:
        return {"suite": suite, "runs": 0, "passed": 0, "pass_rate": 0.0,
                "results": [], "error": f"no scenarios in suite {suite!r}"}
    emit("scenario", "suite", f"running suite {suite} ({len(names)} scenarios)",
         {"suite": suite, "scenarios": names})
    results = [run_scenario(n, robot=robot, tick_hz=tick_hz) for n in names]
    passed = sum(1 for r in results if r.get("outcome") == "passed")
    summary = {"suite": suite, "runs": len(results), "passed": passed,
               "pass_rate": round(passed / len(results), 3),
               "results": results}
    emit("scenario", "suite",
         f"suite {suite} done · {int(summary['pass_rate']*100)}% "
         f"({passed}/{len(results)})", {"suite": suite})
    return summary
