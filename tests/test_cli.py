"""Local-runner CLI: roborun search / scenarios (no browser needed)."""
from __future__ import annotations

import time

import pytest


@pytest.fixture(autouse=True)
def _isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))


def test_search_cli_finds_and_prints(capsys):
    from roborun.spatial_memory import SpatialMemoryStore
    SpatialMemoryStore().store(
        detections=[{"label": "forklift", "score": 1, "bbox": [0, 0, 1, 1]}],
        x=5.0, y=3.0, ts=time.time(), robot_id="go2", source="production")
    from roborun.cli import search_cli
    assert search_cli(["forklift", "--by", "label"]) == 0
    out = capsys.readouterr().out
    assert "forklift" in out and "go2" in out


def test_search_cli_no_hits(capsys):
    from roborun.cli import search_cli
    assert search_cli(["unicorn"]) == 0
    assert "no hits" in capsys.readouterr().out


def test_scenarios_cli_lists_and_runs(capsys):
    from roborun.cli import scenarios_cli
    assert scenarios_cli([]) == 0
    assert "smoke_pass" in capsys.readouterr().out
    assert scenarios_cli(["run", "smoke_pass"]) == 0
    assert "PASSED" in capsys.readouterr().out
    assert scenarios_cli(["suite", "demo"]) == 0
    assert "%" in capsys.readouterr().out


def test_demo_cli_populates(capsys):
    from roborun.cli import demo_cli
    assert demo_cli([]) == 0
    assert "Demo seeded" in capsys.readouterr().out
    from roborun.spatial_memory import SpatialMemoryStore
    from roborun.scenario import list_suites
    assert SpatialMemoryStore().stats()["total"] > 0
    assert any(s["suite"] == "demo" for s in list_suites())


def test_status_cli_offline(capsys):
    from roborun.cli import status_cli
    assert status_cli([]) == 0
    out = capsys.readouterr().out
    assert "server:" in out and "data:" in out


def test_ask_cli_clean_error_when_server_down(capsys, monkeypatch):
    monkeypatch.setenv("ROBORUN_PORT", "59999")  # nothing listening
    from roborun.cli import ask_cli
    assert ask_cli(["go", "forward"]) == 1
    assert "can't reach RoboRun" in capsys.readouterr().out


def test_ask_cli_needs_a_message(capsys):
    from roborun.cli import ask_cli
    assert ask_cli([]) == 2
