"""Scenarios: scored, taggable, filterable runs (RoboRun's eval layer).

Mirrors the Antioch Scenarios data model — params + tags + flat metrics +
nested evaluation + a pass/fail outcome — but backed by the event bus and a
plain JSON registry. No network, no recording required.
"""
from __future__ import annotations

import pytest

from roborun import scenario as S


@pytest.fixture(autouse=True)
def _isolated_state(tmp_path, monkeypatch):
    # Send runs_root() at a throwaway dir so the registry is empty per test.
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))


def test_passing_scenario_persists_and_lists():
    with S.scenario("lobby-nav", tags=["nav", "obstacles"],
                    params={"map": "lobby_v2", "num_obstacles": 6}) as run:
        run.metric("path_length_m", 18.4)
        run.metric("collisions", 0)
        run.evaluate("path_quality", smoothness=0.91, efficiency=0.87)
        run.evaluate("safety", min_clearance_m=0.31, near_misses=0)
        run.passed(goal_reached=True)

    rows = S.list_results()
    assert len(rows) == 1
    row = rows[0]
    assert row["outcome"] == "passed"
    assert row["name"] == "lobby-nav"
    assert row["params"]["num_obstacles"] == 6
    assert row["metrics"]["goal_reached"] is True
    assert row["evaluation"]["safety"]["min_clearance_m"] == 0.31
    assert row["duration_s"] >= 0
    assert row["schema"] == S.SCHEMA


def test_explicit_failure_records_reason():
    with S.scenario("obstacle-6", tags=["nav"]) as run:
        run.metric("collisions", 2)
        run.failed("collided with planter", min_clearance_m=0.0)
    row = S.list_results()[0]
    assert row["outcome"] == "failed"
    assert "collided" in row["reason"]


def test_outcome_inferred_from_goal_reached():
    with S.scenario("infer-fail") as run:
        run.metric("goal_reached", False)
    assert S.list_results()[0]["outcome"] == "failed"

    with S.scenario("infer-pass") as run:
        run.metric("goal_reached", True)
    assert S.list_results(name="infer-pass")[0]["outcome"] == "passed"


def test_exception_inside_block_is_errored_not_swallowed():
    with pytest.raises(RuntimeError):
        with S.scenario("boom") as run:
            run.metric("started", True)
            raise RuntimeError("planner crashed")
    row = S.list_results(name="boom")[0]
    assert row["outcome"] == "error"
    assert "planner crashed" in row["reason"]


def test_fail_if_guard():
    with S.scenario("guarded") as run:
        collisions = 3
        run.metric("collisions", collisions)
        hit = run.fail_if(collisions > 0, "had collisions")
        assert hit is True
    assert S.list_results()[0]["outcome"] == "failed"


def test_registry_filters_by_tag_and_outcome():
    with S.scenario("a", tags=["nav"]) as run:
        run.passed()
    with S.scenario("b", tags=["grasp"]) as run:
        run.failed("dropped it")

    assert [r["name"] for r in S.list_results(tag="nav")] == ["a"]
    assert [r["name"] for r in S.list_results(outcome="failed")] == ["b"]
    assert len(S.list_results()) == 2


def test_get_result_by_id():
    with S.scenario("lookup") as run:
        run.passed()
    sid = S.list_results()[0]["scenario_id"]
    assert S.get_result(sid)["name"] == "lookup"
    assert S.get_result("scn_does_not_exist") is None


def test_suite_pass_rate_aggregation():
    # 3 in lobby-detection: 2 pass, 1 fail → 0.667
    with S.scenario("seated_0", suite="lobby-detection") as run:
        run.passed()
    with S.scenario("seated_1", suite="lobby-detection") as run:
        run.passed()
    with S.scenario("seated_2", suite="lobby-detection") as run:
        run.failed("missed person")
    # a different suite
    with S.scenario("forklift_0", suite="warehouse-forklift") as run:
        run.passed()

    cards = {c["suite"]: c for c in S.list_suites()}
    assert cards["lobby-detection"]["runs"] == 3
    assert cards["lobby-detection"]["passed"] == 2
    assert cards["lobby-detection"]["pass_rate"] == 0.667
    assert cards["warehouse-forklift"]["pass_rate"] == 1.0

    drill = S.suite_summary("lobby-detection")
    assert drill["runs"] == 3 and len(drill["results"]) == 3


def test_ungrouped_bucket_for_suiteless_runs():
    with S.scenario("loose") as run:
        run.passed()
    cards = {c["suite"]: c for c in S.list_suites()}
    assert "(ungrouped)" in cards
    assert cards["(ungrouped)"]["runs"] == 1
