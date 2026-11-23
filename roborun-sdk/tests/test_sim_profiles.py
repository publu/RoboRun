"""Simulating the bundled profiles: YAM + MakerMods Metal runs both arms on the simulation's one station; the simulated table
reaches past every object."""
import json

import numpy as np
import pytest

from roborun_sdk import robot

PROFILES = robot.PROFILES


def _install(tmp_path, monkeypatch):
    monkeypatch.setattr(robot, 'HOME', tmp_path / 'home')
    root = tmp_path / 'runtime'; root.mkdir(); (root / 'kin.py').write_text('')
    for pid, cfg in dict(yam=dict(body='yam', rig='profiles/yam/rig.json', station='http://127.0.0.1:8091', settings=dict(execute_allowed=True)),
                         yam_metal=dict(body='yam_metal', rig='profiles/yam_metal/rig.json', station='http://127.0.0.1:8091', runtime_mode='guarded_manual',
                                        settings=dict(execute_allowed=False), mixed_motion_right_enabled=True,
                                        limbs=dict(right=dict(model='metal', endpoint='http://127.0.0.1:8093', home=dict(positions_rad=[1.37, 0, 0, .15, 0, .1]))))).items():
        (root / 'profiles' / pid).mkdir(parents=True); (root / 'profiles' / pid / 'config.json').write_text(json.dumps(cfg))
    (root / 'config.json').write_text((root / 'profiles' / 'yam' / 'config.json').read_text())
    return root


def test_simulating_yam_metal_runs_both_arms_on_the_station_and_the_real_config_comes_back(tmp_path, monkeypatch):
    root = _install(tmp_path, monkeypatch)
    robot.set_profile(root, 'yam_metal')
    real = json.loads((root / 'config.json').read_text())
    assert robot.sim_config(root, True)
    c = json.loads((root / 'config.json').read_text())
    assert c['body'] == 'yam' and c['rig'] == 'profiles/yam_metal/rig.json' and c['sim_of'] == 'yam_metal'
    assert 'limbs' not in c and 'runtime_mode' not in c and c['settings']['execute_allowed'] is True
    assert robot.current_profile(root) == 'yam_metal'                       # the profile keeps its name
    assert robot.sim_config(root, True) is False                            # once
    assert robot.sim_config(root, False)
    assert json.loads((root / 'config.json').read_text()) == real and robot.sim_config(root, False) is False


def test_a_station_profile_is_not_rewritten_for_the_simulation(tmp_path, monkeypatch):
    root = _install(tmp_path, monkeypatch)
    robot.set_profile(root, 'yam')
    before = (root / 'config.json').read_text()
    assert robot.sim_config(root, True) is False and robot.sim_config(root, False) is False
    assert (root / 'config.json').read_text() == before


def test_the_maker_arm_pair_has_an_overhead_camera_over_its_open_cup():
    rig = json.loads((PROFILES / 'makerarm.json').read_text())
    cup = rig['sim_object']; T = np.asarray(rig['cameras']['top']['T_table_camera'], float)
    assert cup['shape'] == 'cup' and cup['fill'] > 0 and cup['position_mm'][2] == round(cup['size_mm'][2] / 2)
    assert np.allclose(T[:3, 2], [0, 0, -1]) and np.isclose(np.linalg.det(T[:3, :3]), 1)        # straight down, a rotation
    assert np.allclose(T[:3, 1], [-1, 0, 0])                                                      # image up is where the arms face (+x)
    xy = np.asarray(cup['position_mm'][:2]) / 1000
    assert .25 <= np.hypot(*(xy - T[:2, 3])) <= .7                                                # off the camera's axis, in view
