"""The arm catalog's profiles: each arm's model loads with the planner's names, matches its driver table, and robot.yaml picks its profile."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT / 'roborun-station/src', ROOT / 'roborun-sdk/src'):
    if str(p) not in sys.path: sys.path.insert(0, str(p))

from roborun_sdk import arm_profiles as ap                     # noqa: E402
from roborun_sdk.arms import catalog                            # noqa: E402
from roborun_sdk.arms import settings                           # noqa: E402

MODELS = ap.models()
SIDES = [(m, side) for m in MODELS for side in ap.info(m)['sides']]


def load(model, side):
    row = ap.info(model)['sides'][side]
    return row, mujoco.MjModel.from_xml_path(str(ap.MODELS / model / row['model']))


def test_the_catalog_has_a_model_for_every_arm_with_a_public_one():
    assert len(MODELS) >= 29
    rows = catalog()
    for arm, row in rows.items():
        name = row.get('profile')
        if name in ('yam', None): continue
        assert ap.parse(name) == (name, None), arm


@pytest.mark.parametrize('model,side', SIDES)
def test_model_has_the_planners_names(model, side):
    row, m = load(model, side)
    n = row['joints']; names = [m.joint(i).name for i in range(m.njnt)]
    assert names[:n] == [f'joint{i + 1}' for i in range(n)]
    assert [int(m.jnt_qposadr[i]) for i in range(n)] == list(range(n))
    for body in ('base', 'link1', 'gripper', 'tip_left'):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body) >= 0, body
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'grasp_site') >= 0
    if row.get('jaw'):
        assert row['jaw']['joint'] in names
    else:
        assert m.jnt_range[n].tolist()[0] == 0 and m.jnt_range[n + 1].tolist()[0] == 0       # 0 closed .. stroke open, both fingers


@pytest.mark.parametrize('model,side', SIDES)
def test_home_is_inside_the_models_limits(model, side):
    row, m = load(model, side); n = row['joints']; q = np.array(row['home'])
    assert len(q) == n and (q >= m.jnt_range[:n, 0] - 1e-6).all() and (q <= m.jnt_range[:n, 1] + 1e-6).all()


@pytest.mark.parametrize('arm', [a for a, r in catalog().items() if r.get('profile') not in (None, 'yam') and (r.get('options') or {}).get('joints')])
def test_a_table_driven_arm_has_as_many_joints_as_its_model(arm):
    row = catalog()[arm]; model = row['profile']
    assert len(row['options']['joints']) == ap.info(model)['sides']['left']['joints'] == len(row['home']) == len(row['limits'])


def test_licenses_and_sources_are_recorded():
    for model in MODELS:
        s = ap.info(model)['source']
        assert s['repo'].startswith('https://') and s['commit'] and s['license'] and len(s['sha256']) == 64, model


def display_tree(spec, q):
    """ui/robot-kinematics.js makeArm: world pose of every body of a display model at the joint vector q."""
    def mat(pos, wxyz):
        R = np.empty(9); mujoco.mju_quat2Mat(R, np.asarray(wxyz, float)); T = np.eye(4); T[:3, :3] = R.reshape(3, 3); T[:3, 3] = pos; return T
    out = []
    for i, b in enumerate(spec['bodies']):
        T = mat(b['pos'], b['quat'])
        for j in b['joints']:
            v = q[j['index']] if j['index'] < len(q) else 0.; axis = np.asarray(j['axis'], float)
            if j['type'] == 'slide':
                T[:3, 3] += T[:3, :3] @ (axis * v)
            else:
                qq = np.empty(4); mujoco.mju_axisAngle2Quat(qq, axis / np.linalg.norm(axis), v); R = mat([0, 0, 0], qq); p = np.asarray(j['pos'], float)
                T[:3, 3] += T[:3, :3] @ (p - R[:3, :3] @ p); T[:3, :3] = T[:3, :3] @ R[:3, :3]
        out.append(out[b['parent']] @ T if i else T)
    return out


@pytest.mark.parametrize('profile', ['so101', 'franka_panda', 'omx', 'so101+franka_panda', 'unitree_r1_a7_dual'])
def test_the_display_model_moves_like_the_planners_model(profile, tmp_path):
    (tmp_path / 'kin.py').write_text(''); assert ap.ensure(tmp_path, profile)
    rig = json.loads((tmp_path / 'profiles' / profile / 'rig.json').read_text()); spec = json.loads((tmp_path / 'profiles' / profile / 'robot-model.json').read_text())
    assert (tmp_path / 'profiles' / profile / 'config.json').exists()
    rng = np.random.default_rng(0)
    for side, entry in rig['arms'].items():
        part = (spec.get('arm_models') or {}).get(side, spec); m = mujoco.MjModel.from_xml_path(str(tmp_path / 'models' / entry['model'])); d = mujoco.MjData(m); n = entry['joints']
        assert part['arm_joints'] == n and spec['rig']['arms'][side]['T_table_base'] == entry['T_table_base']
        q = rng.uniform(m.jnt_range[:n, 0], m.jnt_range[:n, 1]); jaw = .6; travel = part['jaw_travel_m']
        d.qpos[:n] = q; j = entry.get('jaw')
        if j:
            d.qpos[n] = jaw * j['open_rad']
            if m.njnt > n + 1: d.qpos[n + 1] = jaw * j['open_rad'] * m.eq_data[0][1]
        else: d.qpos[n:n + 2] = jaw * travel
        mujoco.mj_kinematics(m, d)
        poses = display_tree(part, [*q, jaw * travel, jaw * travel])
        for b in range(1, m.nbody):
            assert np.allclose(poses[b][:3, 3], d.xpos[b], atol=1e-4), (side, b)


@pytest.mark.parametrize('arms,wanted', [(dict(left='so101'), 'so101'), (dict(left='so101', right='so101'), 'so101_dual'), (dict(left='xlerobot', right='koch_v1_1'), 'so101+koch_v1_1'),
                                          (dict(left='franka_fr3'), 'franka_fr3'), (dict(left='yam'), 'yam'), (dict(left='yam', right='yam'), 'yam_dual')])
def test_robot_yaml_picks_the_profile(arms, wanted):
    assert settings._profile_for({s: dict(arm=a) for s, a in arms.items()}) == wanted


def test_switching_to_a_catalog_profile_installs_it(tmp_path, monkeypatch):
    from roborun_sdk import robot
    monkeypatch.setattr(robot, 'HOME', tmp_path / 'home')
    root = tmp_path / 'runtime'; (root / 'profiles' / 'yam').mkdir(parents=True); (root / 'kin.py').write_text('')
    (root / 'profiles' / 'yam' / 'config.json').write_text(json.dumps(dict(body='yam', rig='profiles/yam/rig.json', station='http://127.0.0.1:8091')))
    (root / 'config.json').write_text(json.dumps(dict(body='yam', rig='profiles/yam/rig.json')))
    robot.set_profile(root, 'so101_dual')
    assert json.loads((root / 'config.json').read_text())['rig'] == 'profiles/so101_dual/rig.json' and robot.current_profile(root) == 'so101_dual'
    assert (root / 'models/custom/so101/so101.xml').is_file() and (root / 'profiles/so101_dual/robot-model.json').is_file()


def test_a_mixed_pair_builds_a_rig_of_each_arms_own_joints():
    rig = ap.rig('so101+franka_panda'); assert rig['arms']['left']['joints'] == 5 and rig['arms']['right']['joints'] == 7
    assert 'parked' not in rig['arms']['right'] and rig['arms']['left']['model'] != rig['arms']['right']['model']


def test_arms_without_a_public_model_ask_for_a_profile():
    rows = catalog()
    no_model = [a for a, r in rows.items() if not r.get('profile')]
    assert set(no_model) == {'bracketbot', 'robo_t1', 'galaxea_r1_lite'}
    with pytest.raises(SystemExit, match='no model for robo_t1.*set "profile:"'):
        settings._profile_for(dict(left=dict(arm='robo_t1')))
