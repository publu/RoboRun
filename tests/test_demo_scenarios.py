"""Built-in runnable scenarios make the board live out of the box."""
from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))


def test_demos_register():
    import roborun.demo_scenarios  # noqa
    from roborun.scenario_defs import list_defs
    names = {d["name"] for d in list_defs()}
    assert {"smoke_pass", "threshold_gain"} <= names  # mjx_reach only if [mjx]


def test_smoke_passes_and_threshold_branches():
    import roborun.demo_scenarios  # noqa
    from roborun.scenario_defs import run_scenario
    assert run_scenario("smoke_pass")["outcome"] == "passed"
    assert run_scenario("threshold_gain", params={"gain": 0.9})["outcome"] == "passed"
    assert run_scenario("threshold_gain", params={"gain": 0.3})["outcome"] == "failed"


def test_demo_suite_aggregates():
    import roborun.demo_scenarios  # noqa
    from roborun.scenario_defs import run_suite
    s = run_suite("demo")
    assert s["runs"] >= 2 and 0 <= s["pass_rate"] <= 1
