#!/usr/bin/env python3
"""Checks each catalog arm's simulated demo cup: with the arm alone at home (its solo profile, the layout of arm_profiles), the
overhead camera must see the whole cup, the arm must stand clear of it in that image, and no part of the arm may be in the cup's
space. An arm that fails needs a tucked home (data/arm_models/<id>/arm.json `home`, fingers level and forward like the YAM's).

    python tools/arm_models/sim_cup_check.py --runtime ~/roborun/robot/runtime [--only so101,koch_v1_1]

Needs the runtime (kin.py) and MuJoCo rendering (MUJOCO_GL). Exits 1 when an arm fails.
"""
import argparse
import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'src'))
VISIBLE = .97
MARGIN_M = .02
APART_PX = 12           # the arm this far from the cup in the overhead image: an arm touching the cup reads as part of it


def check(world, renderer):
    """(share of the cup the overhead camera sees with the left arm at home, how far that arm is from the cup's space in metres,
    how far it is from the cup in the overhead image in pixels)."""
    import mujoco
    m, d = world.model, world.data
    cup = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '').startswith(world.objects[0]['id'])]
    arm = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) or '').startswith('left')]

    def segments():
        renderer.update_scene(d, camera='top'); renderer.enable_segmentation_rendering()
        seg = renderer.render()[..., 0]; renderer.disable_segmentation_rendering()
        return np.isin(seg, cup), np.isin(seg, arm)
    mask, arm_px = segments()
    alpha = m.geom_rgba[arm, 3].copy(); m.geom_rgba[arm, 3] = 0
    whole = int(segments()[0].sum()); m.geom_rgba[arm, 3] = alpha
    seen = int(mask.sum())
    cy, cx = np.nonzero(mask); ay, ax = np.nonzero(arm_px)
    px = float(np.min(np.hypot(ax[:, None] - cx[None, ::7], ay[:, None] - cy[None, ::7]))) if len(ax) and len(cx) else np.inf
    o = world.objects[0]; c = np.asarray(o['position_mm'][:2], float) / 1000; r = o['size_mm'][0] / 2000; h = o['size_mm'][2] / 1000
    gap = np.inf
    for g in arm:
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        k = int(m.geom_dataid[g]); v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
        p = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        low = p[p[:, 2] <= h + MARGIN_M]
        if len(low):
            gap = min(gap, float(np.min(np.hypot(low[:, 0] - c[0], low[:, 1] - c[1])) - r))
    return (seen / whole if whole else 0.), gap, px


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--runtime', required=True); ap.add_argument('--only')
    a = ap.parse_args()
    os.environ.setdefault('MUJOCO_GL', 'egl')
    root = Path(a.runtime).expanduser()
    for p in (str(root), str(root / 'workspaces' / 'yam' / 'policies')):
        sys.path.insert(0, p)
    from roborun_sdk import arm_profiles
    from roborun_sdk.mujoco_sim import World
    only = set(a.only.split(',')) if a.only else None
    failed = []
    for model in arm_profiles.models():
        if only and model not in only:
            continue
        profile = arm_profiles.name_for(model)
        arm_profiles.ensure(root, profile)                 # the arm's model files and its profile in the runtime
        with tempfile.NamedTemporaryFile('w', suffix='.json', dir=root / 'profiles', delete=False) as f:
            json.dump(arm_profiles.rig(profile), f)
        try:
            world = World(root, f.name, 'cup'); renderer = world.renderer()
            seen, gap, px = check(world, renderer); renderer.close()
        finally:
            os.unlink(f.name)
        ok = seen >= VISIBLE and gap >= MARGIN_M and px >= APART_PX
        failed += [] if ok else [model]
        print(f'{model:18s} {"ok  " if ok else "FAIL"} sees {seen:.2f} of the cup, arm {gap * 1000:.0f} mm from its space, {px:.0f} px from it in the image',
              flush=True)
    if failed:
        print('failed:', ', '.join(failed))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
