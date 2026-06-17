"""Project → Environment scoping (platform specs 07/08)."""
import json
import os

import pytest

from roborun import projects, environments, recorder, events


@pytest.fixture
def state(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    monkeypatch.delenv("ROBORUN_PROJECT", raising=False)
    monkeypatch.delenv("ROBORUN_ENV", raising=False)
    return tmp_path


def test_legacy_layout_when_no_project(state):
    assert projects.active() is None
    assert recorder.runs_root() == state / "runs"
    assert events.runs_root() == state / "runs"


def test_active_project_scopes_runs_root(state):
    projects.create("My Pilot")
    environments.create("my-pilot", "DC-3", backend="gazebo")
    projects.set_active("my-pilot", "dc-3")
    rr = recorder.runs_root()
    assert rr == state / "projects" / "my-pilot" / "dc-3" / "runs"
    assert events.runs_root() == rr
    projects.clear_active()
    assert recorder.runs_root() == state / "runs"


def test_env_var_overrides_disk(state, monkeypatch):
    projects.set_active("on-disk", "default")
    monkeypatch.setenv("ROBORUN_PROJECT", "Pinned")
    monkeypatch.setenv("ROBORUN_ENV", "Lab A")
    a = projects.active()
    assert a == {"project": "pinned", "environment": "lab-a"}


def test_project_and_env_crud(state):
    meta = projects.create("Warehouse Pilot", mode="test")
    assert meta["id"] == "warehouse-pilot" and meta["mode_default"] == "test"
    environments.create("warehouse-pilot", "Floor 1", backend="rapier", mode="scratch")
    environments.create("warehouse-pilot", "Floor 2", backend="rapier")
    envs = {e["id"] for e in environments.list_envs("warehouse-pilot")}
    assert envs == {"floor-1", "floor-2"}
    p = projects.get("warehouse-pilot")
    assert set(p["environments"]) == {"floor-1", "floor-2"}


def test_camera_registration(state):
    environments.create("p", "e", backend="rapier")
    environments.register_camera("p", "e", "cam-0",
                                 placement={"x": 1, "y": 0, "z": 2,
                                            "roll": 0, "pitch": 0, "yaw": 0},
                                 kind="fixed")
    e = environments.get("p", "e")
    assert e["cameras"][0]["source_id"] == "cam-0"
    assert e["cameras"][0]["placement"]["x"] == 1
    # re-register same id replaces, not duplicates
    environments.register_camera("p", "e", "cam-0", kind="robot")
    e = environments.get("p", "e")
    assert len(e["cameras"]) == 1 and e["cameras"][0]["kind"] == "robot"


def test_run_manifest_carries_context(state, tmp_path):
    from roborun import run_manifest
    projects.create("proj")
    environments.create("proj", "env", backend="isaac")
    projects.set_active("proj", "env")
    mcap = tmp_path / "runs" / "robo" / "run-1.mcap"
    mcap.parent.mkdir(parents=True)
    mcap.write_bytes(b"x")
    run_manifest.write_start(mcap, "run-1", "robo", backend="isaac")
    m = run_manifest.read(mcap)
    assert m["project"] == "proj" and m["environment"] == "env"
    assert m["backend"] == "isaac" and m["run_id"] == "run-1"
    run_manifest.finalize(mcap, seal={"merkle_root": "abc",
                                      "anchor": {"status": "unanchored"}})
    m = run_manifest.read(mcap)
    assert m["seal"]["merkle_root"] == "abc" and m["ended"] is not None
