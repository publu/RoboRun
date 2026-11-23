"""Any arm can run on Roborun: the user's own driver file goes through the same `--left` flag and the real station."""
import json
import urllib.request
from types import SimpleNamespace

import pytest

from roborun_sdk.arms import open_arm, parse_spec
from roborun_sdk.arms.hardware import config_from_args
from roborun_sdk.embodiment import ArmAdapter, run_station

DRIVER = """
from roborun_sdk.embodiment import ArmAdapter
class MyArm(ArmAdapter):
    home = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    def __init__(self, bus=None):
        self.bus = bus; self.q = list(self.home); self.jaw = 0.5; self.commands = []
    def read(self): return dict(positions_rad=list(self.q), gripper=self.jaw)
    def command(self, q, jaw): self.commands.append(list(q)); self.q = list(q); self.jaw = jaw
class NotAnArm: pass
"""


def args(**kw):
    return SimpleNamespace(**dict(dict(left=None, right=None, camera=None), **kw))


@pytest.fixture
def driver(tmp_path):
    path = tmp_path / 'my_arm.py'; path.write_text(DRIVER)
    return path


def test_the_flag_keeps_the_address_and_an_absolute_driver_path(driver, monkeypatch):
    monkeypatch.chdir(driver.parent)
    hw = config_from_args(args(left='my_arm.py:MyArm@serial:///dev/ttyUSB0'))
    assert hw['left'] == dict(arm=f'{driver}:MyArm', bus='serial:///dev/ttyUSB0')
    assert parse_spec('my_arm.py:MyArm') == ('my_arm.py:MyArm', None)


def test_a_plain_adapter_gets_only_what_it_accepts_and_runs_on_the_station(driver):
    arm = open_arm(f'{driver}:MyArm', 'left', 'serial:///dev/ttyUSB0')
    assert isinstance(arm, ArmAdapter) and arm.bus == 'serial:///dev/ttyUSB0'
    station, srv = run_station(arm, port=0)
    try:
        with urllib.request.urlopen(srv.url + '/supervisor/pose', timeout=3) as r:
            assert json.load(r)
        assert station.left.q_cmd == pytest.approx(arm.home)     # told nothing yet: it stays where it is
    finally:
        srv.close()


def test_something_that_is_not_an_arm_is_refused(driver):
    with pytest.raises(SystemExit, match='not an ArmAdapter'):
        open_arm(f'{driver}:NotAnArm', 'left')


def test_an_unknown_name_points_at_the_way_in():
    with pytest.raises(SystemExit, match='my_arm.py:MyArm'):
        config_from_args(args(left='no_such_arm'))


def test_arm_options_reach_the_config(driver, tmp_path):
    f = tmp_path / 'o.json'; f.write_text(json.dumps(dict(limits=[[-1, 1]] * 6)))
    hw = config_from_args(args(left=f'{driver}:MyArm', arm_options=[f'left={f}']))
    assert hw['left']['options'] == dict(limits=[[-1, 1]] * 6)
    with pytest.raises(SystemExit, match='left=FILE'):
        config_from_args(args(left=f'{driver}:MyArm', arm_options=[f'right={f}']))


def test_reset_leaves_a_healthy_arm_and_what_it_holds_alone(driver):
    import time
    arm = open_arm(f'{driver}:MyArm', 'left'); arm.q = [.3] * 6; arm.jaw = .2; released = []
    arm.release = lambda: released.append(1)
    station, srv = run_station(arm, port=0)
    try:
        station.reset(); end = time.monotonic() + 8
        while station.state != 'idle' and time.monotonic() < end: time.sleep(.02)
        assert station.state == 'idle' and not released
        assert arm.q == pytest.approx([.3] * 6) and arm.jaw == pytest.approx(.2)
    finally:
        srv.close()
