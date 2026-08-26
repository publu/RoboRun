"""Backend registry + driver conformance (platform spec 03)."""
from roborun import backends, gz, isaac


def test_registry_has_all_six_with_status():
    bl = {b["id"]: b for b in backends.list_backends()}
    assert set(bl) == {"rapier", "mujoco", "mjx", "gazebo", "isaac", "real"}
    for b in bl.values():
        assert b["status"] in {"ready", "available", "optional"}
    # isaac now has a real driver → no longer 'planned'
    assert bl["isaac"]["status"] in {"ready", "available"}
    assert bl["isaac"]["caps"]["photoreal"] is True


def test_simbackend_satisfies_handle_contract():
    """SimBackend is the reference Backend handle (spec 03 P1)."""
    from roborun.sim_backend import SimBackend
    for m in ("pose", "lidar", "see", "move", "state"):
        assert callable(getattr(SimBackend, m, None)), f"SimBackend missing {m}"


def test_gz_and_isaac_degrade_offline():
    """No live world → discovery returns None and run_level dry-runs the plan."""
    assert gz.detect_world(transport=None) is None
    assert isaac.detect_world(transport=None) is None
    level = {"robot": "dog", "spawn": {"x": 1, "z": 2, "heading": 0.5},
             "props": [{"kind": "box", "x": 3, "z": 4}], "walls": [[0, 0, 1, 1]]}
    # pure mappings are deterministic + complete
    gz_plan = gz.level_to_spawns(level, seed=7)
    isaac_plan = isaac.level_to_prims(level, seed=7)
    assert any(p.get("name") == "robot" for p in gz_plan)
    assert any(p.get("prim_path") == "/World/robot" for p in isaac_plan)
    assert "go2" in isaac_plan[0]["usd"]
    # runners degrade to a plan, no exceptions
    assert gz.GzRunner().run_level(level)["status"] == "no-world"
    assert isaac.IsaacRunner().run_level(level)["status"] == "no-stage"
