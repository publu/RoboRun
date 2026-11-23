"""Multi-floor warehouse world for fleet sim (platform spec 04 P1)."""
from roborun import worlds


def test_warehouse_structure():
    w = worlds.warehouse(floors=3, rooms_per_floor=6, size=48, seed=1)
    assert w["name"] == "warehouse" and len(w["floors"]) == 3
    for fl in w["floors"]:
        assert len(fl["rooms"]) == 6 and fl["walls"] and fl["items"]
    # elevators connect adjacent floors (n-1 of them), at a shared shaft
    assert len(w["elevators"]) == 2
    assert {e["from"] for e in w["elevators"]} == {0, 1}
    assert all(e["to"] == e["from"] + 1 for e in w["elevators"])
    assert worlds.item_count(w) == 3 * 6 * 2


def test_warehouse_deterministic():
    a = worlds.warehouse(seed=42)
    b = worlds.warehouse(seed=42)
    assert a == b
    assert worlds.warehouse(seed=1) != worlds.warehouse(seed=2)


def test_single_floor_has_no_elevators():
    w = worlds.warehouse(floors=1)
    assert w["elevators"] == []
