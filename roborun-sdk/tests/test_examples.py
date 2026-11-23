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
