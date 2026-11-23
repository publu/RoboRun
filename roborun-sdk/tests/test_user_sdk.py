"""The user-facing SDK: the arm adapter station, the history upload, the runner's keyed live views."""
import json
import sqlite3
import time

import pytest


# ---------------------------------------------------------------- your arm under the Manifest station
def test_adapter_station_reads_commands_and_goes_stale_on_a_dead_bus():
    from roborun_sdk.embodiment import KinematicArm, _station_classes
    arm = KinematicArm()
    station = _station_classes()(arm, None, lease_free_hold=True)
    for _ in range(5):
        station.tick(); time.sleep(0.01)
    assert station.status()['arms']['left'] is True
    row = station.left.row(station.clock())
    assert row['fresh'] and len(row['positions_rad']) == 6
    station.left.command(q=[0.1] * 6, jaw=0.5)                 # the station's command reaches the adapter
    station.tick()
    assert arm.target[0] == pytest.approx([0.1] * 6) and arm.target[1] == pytest.approx(0.5)
    arm.read = lambda: (_ for _ in ()).throw(OSError('bus dropped'))
    time.sleep(0.12); station.tick()
    assert not station.left.row(station.clock())['fresh']       # no new sample: the station sees it stale, refuses motion


# ---------------------------------------------------------------- history upload
def test_history_upload_is_on_by_default_from_the_runtime_log(tmp_path):
    from types import SimpleNamespace
    from roborun_sdk import robot
    a = SimpleNamespace(manifest=str(tmp_path / 'roborun' / 'robot' / 'runtime'), data_root=None, video_motion=None, no_data_sync=False)
    d = robot._data_config(a)
    assert d['enabled'] and not d['opt_out'] and 'telemetry' not in d
    assert d['sources'] == {'video-motion': str(tmp_path / 'roborun/robot/runtime/log/video-motion'),
                            'tasks': str(tmp_path / 'roborun/robot/runtime/log/tasks'),
                            'logs': str(tmp_path / 'roborun/robot/runtime/log/telemetry')}     # the station's hourly telemetry
    off = robot._data_config(SimpleNamespace(**dict(vars(a), no_data_sync=True)))
    assert not off['enabled'] and off['opt_out']


