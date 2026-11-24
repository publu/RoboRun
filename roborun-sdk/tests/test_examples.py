"""examples/: each example loads through the same path Roborun uses for it."""
import importlib.util
import json
import urllib.request
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[1] / 'examples'


def load(name):
    spec = importlib.util.spec_from_file_location(f'example_{name}', EXAMPLES / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_minimal_arm_runs_on_the_station():
    pytest.importorskip('roborun_station')
    from roborun_sdk.arms import open_arm
    from roborun_sdk.embodiment import run_station
    arm = open_arm(f'{EXAMPLES / "minimal_arm.py"}:MinimalArm', 'left', 'serial:///dev/ttyUSB0')
    assert arm.bus == 'serial:///dev/ttyUSB0'
    station, srv = run_station(arm, port=0)
    try:
        with urllib.request.urlopen(srv.url + '/supervisor/pose', timeout=3) as r:
            assert json.load(r)
    finally:
        srv.close()


def test_keep_out_box_is_a_rules_file():
    from roborun_sdk.arms.base import _load_rule
    check = _load_rule(str(EXAMPLES / 'keep_out_box.py'))
    q = [0.0] * 6
    assert check('left', q, 1.0, dict(tool_position_m=[0.4, 0.0, 0.1])) is False
    assert check('left', q, 1.0, dict(tool_position_m=[0.2, 0.3, 0.1])) is None
    assert check('left', q, 1.0, dict(tool_position_m=None)) is None


def test_folder_camera_is_a_camera_source(tmp_path):
    cv2 = pytest.importorskip('cv2')
    from roborun_sdk.cameras import CustomSource
    image = cv2.imread(str(Path(__file__).parent / 'data' / 'astronaut_face.jpg'))
    cv2.imwrite(str(tmp_path / 'a.jpg'), image)
    source = CustomSource(f'{EXAMPLES / "folder_camera.py"}:FolderCamera', 'top', settings=dict(fps=50),
                          extra=dict(folder=str(tmp_path)))
    model = source.open()
    assert model['serial'] == f'folder:{tmp_path.name}'
    item = source.read()
    assert item['rgb'].shape == image.shape and item['rgb'].dtype.name == 'uint8'


def test_robot_client_imports_with_the_standard_library_only():
    client = load('robot_client')
    assert client.Robot().api == 'http://127.0.0.1:8120'


def test_my_policy_sways_the_wrist_on_the_station(tmp_path, monkeypatch):
    pytest.importorskip('roborun_station')
    import time
    from roborun_sdk import policy
    from roborun_sdk.embodiment import KinematicArm, run_station
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path / 'home'))
    station, srv = run_station(KinematicArm(home=[0.0, 0.5, 0.8, 0.0, 0.3, 0.0]), port=0)
    try:
        seen = []
        run = policy.start(f'{EXAMPLES / "my_policy.py"}:Sway', options={'period': 1.0}, prompt='sway the wrist',
                           arms='left', seconds=1.5, station_url=srv.url, runtime=tmp_path / 'rt', dataset=False,
                           camera_socket=str(tmp_path / 'none.sock'), config=dict(robot='t', profile='custom'))
        while run.wait(0.03) is None:
            seen.append(station.left.q_cmd[5])
        result = run.wait()
        assert result.status == 'done', result
        assert max(seen) > 0.05 and min(seen) < -0.05                   # the wrist swung both ways around 0
        assert max(abs(a - b) for a, b in zip(station.left.q_cmd[:5], [0.0, 0.5, 0.8, 0.0, 0.3])) < 1e-6
    finally:
        srv.close()


def test_dreamlabs_policy_imports_with_the_standard_library_only():
    example = load('dreamlabs_policy')                    # running it needs the extra, a DreamLabs key and arms
    assert example.MODEL == 'dreamlabs/dream-1-beta' and callable(example.main)
