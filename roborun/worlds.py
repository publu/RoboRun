"""Large multi-space worlds for fleet sim (platform spec 04 P1).

A warehouse is more than one bounded arena: multiple floors, each with rooms and
aisles, connected by **elevators** that translate a robot between floor frames.
Pure data — the frontend fleet sim (web/fleet-sim.js) renders + drives it, and a
gz/isaac backend could spawn it. Deterministic given `seed`.
"""
from __future__ import annotations

import random
from typing import Any


def warehouse(floors: int = 2, rooms_per_floor: int = 6, size: float = 48.0,
              seed: int = 0) -> dict[str, Any]:
    """A multi-floor warehouse: a grid of rooms per floor + perimeter/aisle walls,
    elevators stacking the floors, and tagged item spawns to detect.

    Returns: {name, size, floors:[{level, rooms:[{id,rect}], walls, items:[{label,x,y}]}],
              elevators:[{id, x, y, from, to}], spawns:[{x,y,floor,heading}]}"""
    rng = random.Random(seed)
    cols = 3
    rows = max(1, (rooms_per_floor + cols - 1) // cols)
    cw, ch = size / cols, size / rows
    labels = ["pallet", "forklift", "shelf", "crate", "person", "agv", "barrel"]

    world: dict[str, Any] = {"name": "warehouse", "size": size,
                             "floors": [], "elevators": [], "spawns": []}
    for f in range(floors):
        rooms, items = [], []
        for i in range(rooms_per_floor):
            cx, cy = i % cols, i // cols
            rect = [cx * cw, cy * ch, (cx + 1) * cw, (cy + 1) * ch]
            rooms.append({"id": f"f{f}-r{i}", "rect": rect})
            # a couple of detectable items per room, at deterministic spots
            for _ in range(2):
                items.append({"label": rng.choice(labels),
                              "x": round(rect[0] + rng.uniform(.2, .8) * cw, 2),
                              "y": round(rect[1] + rng.uniform(.2, .8) * ch, 2)})
        walls = [[0, 0, size, 0], [size, 0, size, size],
                 [size, size, 0, size], [0, size, 0, 0]]
        # interior aisle walls between room columns (with gaps = doorways)
        for c in range(1, cols):
            walls.append([c * cw, 0, c * cw, size * 0.42])
            walls.append([c * cw, size * 0.58, c * cw, size])
        world["floors"].append({"level": f, "rooms": rooms, "walls": walls, "items": items})
        world["spawns"].append({"x": cw * 0.5, "y": ch * 0.5, "floor": f, "heading": 0.0})

    # elevators stack adjacent floors at a shared shaft position
    shaft_x, shaft_y = round(size * 0.5, 2), round(size * 0.5, 2)
    for f in range(floors - 1):
        world["elevators"].append({"id": f"elev-{f}", "x": shaft_x, "y": shaft_y,
                                   "from": f, "to": f + 1})
    return world


def item_count(world: dict) -> int:
    return sum(len(fl.get("items", [])) for fl in world.get("floors", []))
