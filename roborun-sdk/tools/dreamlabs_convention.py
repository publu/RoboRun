"""Write roborun_sdk/policy/data/dreamlabs_conventions.json from DreamLabs' own `dreamlabs` wheel (Apache-2.0).

The DreamLabs provider runs the base Dream-1 model only on an arm whose joint convention (joint order, signs, zero offsets,
limits, gripper polarity) is known to be the one DreamLabs' YAM robot config uses. This tool reads that config from the
published wheel and writes what it found, with the wheel's name and sha256; `tests/test_dreamlabs_convention.py` compares
the file with Roborun's own YAM model and vendored driver config.

The wheel is read as a zip; nothing in it is imported, installed or run, and it is never a dependency:

    pip download dreamlabs==1.0.42 --no-deps -d /tmp/dl
    python tools/dreamlabs_convention.py /tmp/dl/dreamlabs-1.0.42-py3-none-any.whl
"""
import argparse
import hashlib
import json
import re
import sys
import zipfile
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / 'src' / 'roborun_sdk' / 'policy' / 'data' / 'dreamlabs_conventions.json'
ARM_CONFIG = 'i2rt/robots/config/yam.yml'
GRIPPER_CONFIG = 'i2rt/robots/config/linear_4310.yml'
ARM_MODEL = 'i2rt/robot_models/arm/yam/yam.xml'
OPENER = 'i2rt/robots/get_robot.py'
DRIVER = 'robots/yam.py'
GRIPPER_WORDS = 'gripper is already 0 closed to 1 open'
JOINT = re.compile(r'<joint name="(joint\d)"[^>]*\brange="([-0-9.eE]+) ([-0-9.eE]+)"')


def read(z, name):
    try:
        return z.read(name).decode()
    except KeyError:
        raise SystemExit(f'{name} is not in the wheel: this tool knows the layout of dreamlabs 1.0.x')


def convention(wheel):
    wheel = Path(wheel)
    digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
    with zipfile.ZipFile(wheel) as z:
        arm = yaml.safe_load(read(z, ARM_CONFIG))
        grip = yaml.safe_load(read(z, GRIPPER_CONFIG))
        model = read(z, ARM_MODEL)
        opener = read(z, OPENER)
        driver = read(z, DRIVER)
        version = next((m.group(1) for m in map(re.compile(r'dreamlabs-([^/-]+)\.dist-info/METADATA$').match,
                                                  z.namelist()) if m), None)
    joints = [(name, float(lo), float(hi)) for name, lo, hi in JOINT.findall(model)]
    if [j[0] for j in joints] != [f'joint{i}' for i in range(1, 7)]:
        raise SystemExit(f'{ARM_MODEL}: expected joint1..joint6 in order, found {[j[0] for j in joints]}')
    motors = [[int(m[0]), str(m[1])] for m in arm['motor_list']]
    directions = [int(d) for d in arm['directions']]
    zero = 'motor_offsets = [0.0] * len(motor_list)' in opener
    gripper_fraction = ' '.join(GRIPPER_WORDS.split()) in ' '.join(driver.split())
    verified = zero and gripper_fraction and len(motors) == 6 and len(directions) == 6
    return {
        'schema': 'roborun-policy-convention/1',
        'about': ("The joint convention of DreamLabs' YAM robot config, read from DreamLabs' published `dreamlabs` wheel "
                  '(Apache-2.0) by tools/dreamlabs_convention.py. tests/test_dreamlabs_convention.py compares it with '
                  "Roborun's YAM model and driver config; `verified` models may run the base Dream-1 model."),
        'source': {'package': 'dreamlabs', 'version': version, 'wheel': wheel.name, 'sha256': digest,
                   'license': 'Apache-2.0', 'files': [ARM_CONFIG, ARM_MODEL, GRIPPER_CONFIG, OPENER, DRIVER]},
        'models': {
            'yam': {
                'verified': bool(verified),
                'joints': [j[0] for j in joints],
                'motors': motors,
                'directions': directions,
                'zero_offsets_rad': [0.0] * 6 if zero else None,
                'limits_rad': [[lo, hi] for _, lo, hi in joints],
                'gripper': {'type': 'linear_4310', 'motor_type': grip.get('motor_type'), 'units': 'open fraction',
                            'closed': 0.0, 'open': 1.0, 'driver_converts_nothing': gripper_fraction},
                # how a Roborun row maps to DreamLabs' state and back: identity when everything above matches Roborun's
                'mapping': {'joint_order': list(range(6)), 'signs': [1] * 6, 'offsets_rad': [0.0] * 6,
                            'gripper': 'same open fraction'},
            },
        },
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('wheel', help='the downloaded dreamlabs-1.0.x wheel')
    ap.add_argument('--out', default=str(OUT))
    a = ap.parse_args(argv)
    data = convention(a.wheel)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(data, indent=1) + '\n')
    print(f"wrote {a.out}: yam verified={data['models']['yam']['verified']} from {data['source']['wheel']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
