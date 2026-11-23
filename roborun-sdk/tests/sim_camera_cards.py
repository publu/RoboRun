"""The simulation's cameras, drawn from their cards: run by test_camera_cards.py in a fresh process per scenario (one GL context
per process, as `roborun sim` has).

    python sim_camera_cards.py SCENARIO KIN_PY WORK_DIR        prints PASSED, or CANNOT RENDER when there is no OpenGL
"""
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np


def world_for(kin, work, **kw):
    from roborun_sdk import arm_profiles, mujoco_sim
    root = Path(work) / 'runtime'
    if not root.exists():
        root.mkdir(parents=True)
        shutil.copy2(kin, root / 'kin.py')
        assert arm_profiles.ensure(root, 'so101_dual')
    sys.path.insert(0, str(root))
    return root, mujoco_sim


def pose_of(world, cam):
    mj = world.mujoco
    cid = mj.mj_name2id(world.model, mj.mjtObj.mjOBJ_CAMERA, cam)
    T = np.eye(4); T[:3, :3] = world.data.cam_xmat[cid].reshape(3, 3) @ np.diag([1., -1., -1.]); T[:3, 3] = world.data.cam_xpos[cid]
    return T


def renderer(world):
    try:
        return world.renderer()
    except Exception as exc:                              # no EGL and no OSMesa on this computer
        print(f'CANNOT RENDER: {type(exc).__name__}: {exc}')
        sys.exit(0)


def wrist_pose(kin, work):
    """A known T_tool_camera: the camera floats 0.35 m above a green block, looking straight down. The block's centre lands
    where the card's pose and intrinsics project it (OpenCV axes: x right, y down, z forward), and nowhere mirrored; when the
    arm turns, the camera turns with the gripper and the image moves, while the card's mount stays."""
    from roborun_sdk import camera_card as cc
    root, mujoco_sim = world_for(kin, work)
    rig_path = root / 'profiles/so101_dual/rig.json'
    spec = json.loads(rig_path.read_text())
    block = [-.30, .45]
    scene = Path(work) / 'scene.json'
    scene.write_text(json.dumps([dict(id='marker', shape='box', position_mm=[block[0] * 1000, block[1] * 1000, 30],
                                      size_mm=[60, 60, 60], color=[0., 1., 0.])]))
    first = mujoco_sim.World(root, rig_path, str(scene))
    site = first._site('left')
    want = np.array([[1, 0, 0, block[0] + .04], [0, -1, 0, block[1] - .03], [0, 0, -1, .06 + .35], [0, 0, 0, 1.]])
    T_tool = np.linalg.inv(site) @ want
    spec['cameras']['left']['T_tool_camera'] = T_tool.round(12).tolist()
    rig_path.write_text(json.dumps(spec))
    world = mujoco_sim.World(root, rig_path, str(scene), cameras=dict(left=dict(source='usb:1', width=320, height=240, card={})))
    card = cc.validate(world.card('left'))
    assert card['pose']['source'] == 'sim' and card['pose']['preview'] is False and card['optics']['lens']['source'] == 'sim'
    assert card['pose']['parent'] == 'left.tool' and card['optics']['rectified'] is True
    assert np.allclose(card['pose']['T'], T_tool, atol=1e-8), card['pose']['T']
    assert np.allclose(pose_of(world, 'wrist_left'), want, atol=1e-6), pose_of(world, 'wrist_left')
    f = world.render(renderer(world), 'left')
    k = card['optics']['pinhole']
    assert f['intrinsics'] == {**{key: k[key] for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy')},
                               'distortion_model': 'none', 'coeffs': [0, 0, 0, 0, 0]}, (f['intrinsics'], k)
    assert f['rgb'].shape == (240, 320, 3) and f['depth_available'] is True and f['camera_id'] == 'left'
    p = np.linalg.inv(want) @ np.array([block[0], block[1], .06, 1.])
    u, v = k['fx'] * p[0] / p[2] + k['ppx'], k['fy'] * p[1] / p[2] + k['ppy']
    assert u < k['ppx'] - 10 and v < k['ppy'] - 10, (u, v)      # the block is left of and above the image centre

    def green(x, y):
        px = f['rgb'][int(round(y)), int(round(x))].astype(int)
        return px[1] > 1.5 * max(px[0], px[2]) + 20
    assert green(u, v), f['rgb'][int(round(v)), int(round(u))]
    assert not green(2 * k['ppx'] - u, 2 * k['ppy'] - v) and not green(u, 2 * k['ppy'] - v) and not green(2 * k['ppx'] - u, v)
    z = f['depth_z16'][int(round(v)), int(round(u))] * f['depth_scale_m']
    assert abs(z - .35) < .01, z
    before = f['rgb'].astype(int)
    with world.lock:
        world.q['left'] = world.q['left'] + np.r_[.3, np.zeros(len(world.q['left']) - 1)]
        world._apply()
    moved = pose_of(world, 'wrist_left')
    assert np.allclose(moved, world._site('left') @ T_tool, atol=1e-6) and not np.allclose(moved, want, atol=1e-3)
    after = world.render(renderer(world), 'left')['rgb'].astype(int)
    assert np.abs(after - before).mean() > 5, np.abs(after - before).mean()     # the image moved with the gripper
    assert world.card('left')['pose']['T'] == card['pose']['T']                  # the mount did not


def rgb_only(kin, work):
    from roborun_sdk import camera_card as cc
    root, mujoco_sim = world_for(kin, work)
    world = mujoco_sim.World(root, root / 'profiles/so101_dual/rig.json',
                             cameras=dict(right=dict(source='usb:1', card=dict(depth=False)), aux='usb:2'))
    assert set(world.cameras) == {'top', 'left', 'right'}               # aux has no alias: it is not simulated
    f = world.render(renderer(world), 'right')
    assert f['depth_available'] is False and f['depth_scale_m'] is None and f['depth_aligned_to'] is None
    assert not f['depth_z16'].any() and f['depth_z16'].shape == f['rgb'].shape[:2]
    status = mujoco_sim.Cameras(world, Path(work) / 'c.sock').status()
    assert status['roles_without_depth'] == ['right'] and status['available_roles'] == ['top', 'left', 'right']
    cards = {c['alias']: c for c in status['cards']}
    assert cards['right']['optics']['depth'] == dict(available=False, scale_m=None, aligned_to=None, declared=False)
    assert cards['top']['pose']['source'] == 'sim'
    assert np.allclose(cards['top']['pose']['T'], world.rig_spec['cameras']['top']['T_table_camera'], atol=1e-6)
    assert cards['left']['optics']['depth']['available'] is True
    doc, _ = cc.describe(dict(right=dict(source='usb:1', card=dict(depth=False))), profile='so101_dual', status=status)
    assert [c['source']['kind'] for c in doc['cameras']] == ['sim'] * 3


def as_before(kin, work):
    root, mujoco_sim = world_for(kin, work)
    world = mujoco_sim.World(root, root / 'profiles/so101_dual/rig.json')
    r = renderer(world)
    for role in ('top', 'left', 'right'):
        f = world.render(r, role)
        assert f['rgb'].shape == (480, 640, 3) and f['depth_available'] is True and f['depth_z16'].any()
        assert f['depth_scale_m'] == (0.001 if role == 'top' else 0.0001)
        fovy = 64.9 if role == 'top' else mujoco_sim.WRIST_FOVY
        assert f['intrinsics']['ppx'] == 320 and abs(f['intrinsics']['fx'] - 240 / math.tan(math.radians(fovy) / 2)) < 1e-6
    assert np.allclose(world.mounts['left'], world.mount) and np.allclose(world.mounts['right'], world.mount)



def unchanged_setup(kin, work):
    """A rig that lists no camera for one arm (as yam_metal lists only top and left) still draws that arm's wrist camera,
    640x480 RGB-D at the CAD mount, as before cards; robot.yaml entries without a card (a size written for the real camera,
    an entry the card checks would refuse) leave the simulation's cameras and pixels as they were."""
    root, mujoco_sim = world_for(kin, work)
    rig_path = root / 'profiles/so101_dual/rig.json'
    spec = json.loads(rig_path.read_text())
    del spec['cameras']['right']
    rig_path = root / 'profiles/so101_dual/rig_no_right.json'
    rig_path.write_text(json.dumps(spec))
    plain = mujoco_sim.World(root, rig_path)
    r = renderer(plain)
    assert set(plain.cameras) == {'top', 'left', 'right'}
    f = plain.render(r, 'right')
    assert f['rgb'].shape == (480, 640, 3) and f['depth_available'] is True and f['depth_z16'].any()
    assert abs(f['intrinsics']['fx'] - 240 / math.tan(math.radians(mujoco_sim.WRIST_FOVY) / 2)) < 1e-6
    assert np.allclose(pose_of(plain, 'wrist_right'), plain._site('right') @ plain.mount, atol=1e-6)
    status = mujoco_sim.Cameras(plain, Path(work) / 'c.sock').status()
    assert status['available_roles'] == ['top', 'left', 'right'] and status['roles_without_depth'] == []
    shots = {role: plain.render(r, role) for role in ('top', 'left', 'right')}
    real = dict(top=dict(source='usb:1', width=1280, height=720), left=dict(source='usb:2', rotate=90, width=640, height=480),
                Back='usb:3')
    own = mujoco_sim.World(root, rig_path, cameras=real)
    assert own.render_size == plain.render_size == (640, 480)
    for role, before in shots.items():
        after = own.render(r, role)
        assert after['intrinsics'] == before['intrinsics'], (role, after['intrinsics'], before['intrinsics'])
        assert np.array_equal(after['rgb'], before['rgb']), role

if __name__ == '__main__':
    scenario, kin, work = sys.argv[1:4]
    dict(wrist_pose=wrist_pose, rgb_only=rgb_only, as_before=as_before, unchanged_setup=unchanged_setup)[scenario](kin, work)
    print('PASSED')
