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
