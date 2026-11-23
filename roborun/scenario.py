"""Scenarios — scored, taggable, replayable runs (RoboRun's eval layer).

This is the missing paradigm RL_HARNESS_SPEC.md calls out: RoboRun is a control
loop (`@behavior`, `robot.move()`) with no notion of an *episode* — no reset, no
reward, no pass/fail. A scenario wraps a stretch of robot activity into one scored
record:

    from roborun.scenario import scenario

    with scenario("lobby-nav", tags=["nav", "obstacles"],
                  params={"map": "lobby_v2", "num_obstacles": 6}) as run:
        drive_to_goal(robot)                       # any behaviors / handle calls
        run.metric("path_length_m", 18.4)
        run.metric("collisions", 0)
        run.evaluate("safety", min_clearance_m=0.31, near_misses=0)
        run.passed(goal_reached=True)

On exit it writes one `<id>.scenario.json` into the runs root and emits markers
into the timeline (so, if a recording is live, the scored result lands *inside*
the sealed, tamper-evident MCAP). Two things make this superset Antioch's
Scenarios rather than copy it:

  * every scenario row is backed by a sealed run (recorder.py), so the score is
    verifiable, not just asserted; and
  * the same scenario, written once against the portable handle, runs on every
    embodiment by construction (SIM_SPEC's contract) — Antioch's Isaac scenarios
    are single-stack.

No new dependencies: stdlib + the existing event bus. Recording is optional.
"""
from __future__ import annotations

import json
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from roborun.events import emit, runs_root

SCHEMA = "roborun-scenario/1"


def _scenarios_dir() -> Path:
    d = runs_root() / "scenarios"
    d.mkdir(parents=True, exist_ok=True)
    return d


@dataclass
class ScenarioRun:
    """The live handle yielded by `scenario(...)`. Collect metrics as you go;
    declare the outcome before the block ends (or it's inferred)."""

    name: str
    tags: list[str] = field(default_factory=list)
    params: dict[str, Any] = field(default_factory=dict)
    robot: str = "local"
    suite: str | None = None  # groups runs into a pass-rate card (Antioch suites)
    seed: int | None = None  # recorded for deterministic replay + fair A/B
    scenario_id: str = ""
    run_id: str | None = None  # linked sealed MCAP run, if recording
    started: float = field(default_factory=time.time)
    metrics: dict[str, Any] = field(default_factory=dict)
    evaluation: dict[str, dict[str, Any]] = field(default_factory=dict)
    outcome: str | None = None  # "passed" | "failed" | "error"
    reason: str | None = None

    # ── collection API ──────────────────────────────────────────────────────
    def metric(self, key: str, value: Any) -> "ScenarioRun":
        """Record one flat result, e.g. metric('collisions', 0). Antioch's
        Results row."""
        self.metrics[key] = value
        return self

    def metrics_update(self, **kw: Any) -> "ScenarioRun":
        self.metrics.update(kw)
        return self

    def evaluate(self, group: str, **scores: Any) -> "ScenarioRun":
        """Record a nested evaluation group, e.g.
        evaluate('safety', min_clearance_m=0.31, near_misses=0). Antioch's
        evaluation → path_quality / safety tree."""
        self.evaluation.setdefault(group, {}).update(scores)
        return self

    # ── outcome API ─────────────────────────────────────────────────────────
    def passed(self, **results: Any) -> "ScenarioRun":
        self.metrics.update(results)
        self.outcome = "passed"
        return self

    def failed(self, reason: str = "", **results: Any) -> "ScenarioRun":
        self.metrics.update(results)
        self.outcome = "failed"
        self.reason = reason or self.reason
        return self

    def fail_if(self, condition: bool, reason: str) -> bool:
        """Guard helper: `run.fail_if(collisions > 0, 'hit something')`.
        Returns the condition so callers can branch."""
        if condition:
            self.failed(reason)
        return condition

    # ── persistence ─────────────────────────────────────────────────────────
    def to_dict(self) -> dict[str, Any]:
        ended = getattr(self, "_ended", time.time())
        return {
            "schema": SCHEMA,
            "scenario_id": self.scenario_id,
            "name": self.name,
            "tags": self.tags,
            "params": self.params,
            "robot": self.robot,
            "suite": self.suite,
            "seed": self.seed,
            "run_id": self.run_id,
            "outcome": self.outcome,
            "reason": self.reason,
            "metrics": self.metrics,
            "evaluation": self.evaluation,
            "started": _iso(self.started),
            "ended": _iso(ended),
            "duration_s": round(ended - self.started, 3),
        }

    def _persist(self) -> Path:
        path = _scenarios_dir() / f"{self.scenario_id}.scenario.json"
        path.write_text(json.dumps(self.to_dict(), indent=2, default=str))
        return path


def _iso(ts: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(ts))


class _ScenarioCtx:
    def __init__(self, run: ScenarioRun) -> None:
        self.run = run

    def __enter__(self) -> ScenarioRun:
        r = self.run
        emit("scenario", r.name,
             f"scenario start · {r.name}"
             + (f" [{', '.join(r.tags)}]" if r.tags else ""),
             {"scenario_id": r.scenario_id, "params": r.params, "tags": r.tags})
        return r

    def __exit__(self, exc_type, exc, tb) -> bool:
        r = self.run
        r._ended = time.time()  # type: ignore[attr-defined]
        if exc_type is not None:
            # An exception inside the block is an errored scenario, not a crash
            # of the program — record it and re-raise.
            r.outcome = "error"
            r.reason = f"{exc_type.__name__}: {exc}"
            last = traceback.format_exc().strip().splitlines()[-1]
            r.reason = last
        elif r.outcome is None:
            # Infer: an explicit goal_reached metric decides; else assume passed.
            goal = r.metrics.get("goal_reached")
            r.outcome = "failed" if goal is False else "passed"
        path = r._persist()
        emit("scenario", r.name,
             f"scenario {r.outcome} · {r.name}"
             + (f" — {r.reason}" if r.reason else ""),
             {"scenario_id": r.scenario_id, "outcome": r.outcome,
              "metrics": r.metrics, "evaluation": r.evaluation,
              "duration_s": r.to_dict()["duration_s"], "file": str(path)})
        return False  # never suppress exceptions


def scenario(name: str, tags: list[str] | None = None,
             params: dict[str, Any] | None = None,
             robot: str = "local", suite: str | None = None,
             seed: int | None = None) -> _ScenarioCtx:
    """Open a scored scenario. Use as a context manager; see module docstring.

    `suite` groups runs into one pass-rate card (Antioch's suites). If a
    recording is live (recorder.py), the scenario binds to that run_id so the
    score is committed inside the sealed MCAP."""
    base = time.strftime("scn_%Y%m%d_%H%M%S", time.gmtime())
    sid, n = base, 1
    while (_scenarios_dir() / f"{sid}.scenario.json").exists():
        sid = f"{base}_{n}"
        n += 1
    run_id = None
    try:
        from roborun import recorder
        rec = recorder.active_recorder()
        if rec is not None:
            run_id = rec.run_id
            robot = getattr(rec, "robot_id", robot) or robot
    except Exception:
        pass
    run = ScenarioRun(name=name, tags=list(tags or []),
                      params=dict(params or {}), robot=robot, suite=suite,
                      seed=seed, scenario_id=sid, run_id=run_id)
    return _ScenarioCtx(run)


# ── registry (the "Scenarios" list view, headless) ─────────────────────────

def list_results(limit: int = 100, tag: str | None = None,
                 outcome: str | None = None,
                 name: str | None = None,
                 suite: str | None = None) -> list[dict[str, Any]]:
    """Read scored scenario records newest-first, with Antioch-style filters
    (by tag, outcome, scenario name, suite). Pure file scan — no service."""
    rows: list[dict[str, Any]] = []
    for p in sorted(_scenarios_dir().glob("*.scenario.json"), reverse=True):
        try:
            row = json.loads(p.read_text())
        except Exception:
            continue
        if tag and tag not in row.get("tags", []):
            continue
        if outcome and row.get("outcome") != outcome:
            continue
        if name and row.get("name") != name:
            continue
        if suite and (row.get("suite") or "(ungrouped)") != suite:
            continue
        rows.append(row)
        if len(rows) >= limit:
            break
    return rows


def get_result(scenario_id: str) -> dict[str, Any] | None:
    p = _scenarios_dir() / f"{scenario_id}.scenario.json"
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text())
    except Exception:
        return None


def find_by_run(run_id: str) -> dict[str, Any] | None:
    """The scored scenario record whose run_id matches — the run-detail's
    metadata/params/results/evaluation source. Newest-first file scan."""
    if not run_id:
        return None
    for p in sorted(_scenarios_dir().glob("*.scenario.json"), reverse=True):
        try:
            row = json.loads(p.read_text())
        except Exception:
            continue
        if row.get("run_id") == run_id:
            return row
    return None


def list_suites() -> list[dict[str, Any]]:
    """Aggregate scored runs into suite cards — pass rate, run count, latest
    activity — the Antioch "Suites" view. Runs with no `suite` fall under
    "(ungrouped)". `passed` counts toward the rate; `failed` and `error` don't.
    """
    buckets: dict[str, list[dict[str, Any]]] = {}
    for row in list_results(limit=10_000):
        buckets.setdefault(row.get("suite") or "(ungrouped)", []).append(row)
    cards = []
    for suite, rows in buckets.items():
        passed = sum(1 for r in rows if r.get("outcome") == "passed")
        cards.append({
            "suite": suite,
            "runs": len(rows),
            "passed": passed,
            "pass_rate": round(passed / len(rows), 3) if rows else 0.0,
            "latest": rows[0].get("ended"),   # list_results is newest-first
            "scenarios": sorted({r.get("name", "?") for r in rows}),
        })
    cards.sort(key=lambda c: c["latest"] or "", reverse=True)
    return cards


def suite_summary(suite: str) -> dict[str, Any]:
    """One suite's card plus its runs (the drill-in view)."""
    rows = [r for r in list_results(limit=10_000)
            if (r.get("suite") or "(ungrouped)") == suite]
    passed = sum(1 for r in rows if r.get("outcome") == "passed")
    return {
        "suite": suite,
        "runs": len(rows),
        "passed": passed,
        "pass_rate": round(passed / len(rows), 3) if rows else 0.0,
        "results": rows,
    }
