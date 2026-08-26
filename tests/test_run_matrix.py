"""A/B + sweep runner and regression gate over the scenario layer."""
from __future__ import annotations

import pytest

from roborun import scenario_defs as D


@pytest.fixture(autouse=True)
def _isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    monkeypatch.setattr(D, "_REGISTRY", {})


def _register_threshold_scenario():
    # Passes when params["gain"] >= 1.0; seed shifts the bar slightly so seeds matter.
    @D.scenario_def("reach", suite="nav", params={"gain": 0.5})
    def reach(ctx):
        bar = 1.0 + (ctx.seed or 0) * 0.0  # seed recorded; deterministic here
        ctx.run.metric("gain", ctx.params["gain"])
        ctx.run.metric("seed", ctx.seed)
        if ctx.params["gain"] >= bar:
            ctx.run.passed()
        else:
            ctx.run.failed("gain too low")


def test_seed_is_recorded():
    _register_threshold_scenario()
    rec = D.run_scenario("reach", params={"gain": 1.0}, seed=42)
    assert rec["seed"] == 42
    assert rec["metrics"]["seed"] == 42


def test_run_matrix_compares_variants():
    _register_threshold_scenario()
    out = D.run_matrix("reach",
                       variants={"weak": {"gain": 0.5}, "strong": {"gain": 1.2}},
                       seeds=[0, 1, 2])
    assert len(out["cells"]) == 6
    assert out["by_variant"]["weak"]["pass_rate"] == 0.0
    assert out["by_variant"]["strong"]["pass_rate"] == 1.0
    assert out["winner"] == "strong"


def test_run_matrix_default_variant():
    _register_threshold_scenario()
    out = D.run_matrix("reach", seeds=[0])
    assert out["by_variant"]["default"]["runs"] == 1


def test_regression_gate_pass_and_fail():
    @D.scenario_def("a", suite="grasp")
    def a(ctx):
        ctx.run.passed()

    @D.scenario_def("b", suite="grasp")
    def b(ctx):
        ctx.run.failed("dropped")

    # suite pass-rate = 0.5
    assert D.regression_gate("grasp", baseline_pass_rate=0.4)["ok"] is True
    assert D.regression_gate("grasp", baseline_pass_rate=0.6)["ok"] is False
