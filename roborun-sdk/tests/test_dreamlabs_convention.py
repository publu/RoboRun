"""The YAM joint convention DreamLabs' YAM config uses (policy/data/dreamlabs_conventions.json, written by
tools/dreamlabs_convention.py from DreamLabs' published `dreamlabs` wheel) against Roborun's own YAM model and its vendored
driver config: joint order, signs, zero offsets, limits, gripper polarity. The 0.5 rad seam check cannot catch a consistent
flip, so the base Dream-1 model runs only on an arm model whose entry here is `verified` and matches."""
import json
import re
from pathlib import Path

import yaml

from roborun_sdk.policy import dreamlabs as dl

HERE = Path(__file__).resolve().parent
SDK = HERE.parent / 'src' / 'roborun_sdk'
I2RT = SDK / 'third_party' / 'i2rt'
CONV = SDK / 'policy' / 'data' / 'dreamlabs_conventions.json'


def test_the_file_cites_its_source_wheel():
    data = json.loads(CONV.read_text())
    src = data['source']
    assert src['package'] == 'dreamlabs' and src['license'] == 'Apache-2.0'
    assert re.fullmatch(r'[0-9a-f]{64}', src['sha256']) and src['wheel'].startswith('dreamlabs-')
    assert data['models']['yam']['verified'] is True


def test_yam_matches_roborun_joint_by_joint():
    yam = dl.conventions()['yam']
    arm = json.loads((SDK / 'data' / 'arm_models' / 'yam' / 'arm.json').read_text())['sides']['left']
    assert arm['source_joints'] == yam['joints'] == [f'joint{i}' for i in range(1, 7)]       # order
    assert arm['flipped'] == [] and arm['zero_offsets'] == {}                               # Roborun maps nothing
    ours = yaml.safe_load((I2RT / 'robots' / 'config' / 'yam_v1.yml').read_text())
    assert [list(m) for m in ours['motor_list']] == yam['motors']                            # the same motors, ids 1..6
    assert list(ours['directions']) == yam['directions'] == [1] * 6                          # signs
    opener = (I2RT / 'robots' / 'get_robot.py').read_text()
    assert 'motor_offsets = [0.0] * len(motor_list)' in opener and yam['zero_offsets_rad'] == [0.0] * 6   # zeros
    m = yam['mapping']
    assert m['joint_order'] == list(range(6)) and m['signs'] == [1] * 6 and m['offsets_rad'] == [0.0] * 6
    for (lo, hi), (dlo, dhi) in zip(arm['limits'], yam['limits_rad']):                       # limits: Roborun's sit inside
        assert lo >= dlo - 1e-3 and hi <= dhi + 1e-3, (lo, hi, dlo, dhi)


def test_the_gripper_is_an_open_fraction_on_both_sides():
    yam = dl.conventions()['yam']
    g = yam['gripper']
    assert g['units'] == 'open fraction' and (g['closed'], g['open']) == (0.0, 1.0) and g['driver_converts_nothing']
    assert (I2RT / 'robots' / 'config' / 'linear_4310.yml').exists() and g['type'] == 'linear_4310'
    from roborun_station.core import ArmSpec
    from roborun_station.policy import fraction_from_jaw, jaw_from_fraction
    spec = ArmSpec.__dataclass_fields__
    closed, opened = spec['jaw_closed'].default, spec['jaw_open'].default                     # a YAM's station jaw units
    assert (closed, opened) == (0.0, 1.0)
    assert fraction_from_jaw(0.25, (closed, opened)) == 0.25 and jaw_from_fraction(0.75, (closed, opened)) == 0.75


def test_only_verified_models_are_offered():
    assert {k for k, v in dl.conventions().items() if v.get('verified')} == {'yam'}
