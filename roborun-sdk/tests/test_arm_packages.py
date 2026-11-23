"""An arm or camera's packages arrive on first use; nobody has to name them at install time."""
import subprocess
from importlib import metadata

import pytest

from roborun_sdk.arms import packages


DECLARED = ['numpy>=1.26', 'trossen-arm>=1.11; extra == "trossen"', 'already-here>=1; extra == "trossen"',
            'too-old>=2; extra == "trossen"', 'only-mac>=1; extra == "trossen" and sys_platform == "no-such-platform"',
            'other>=1; extra == "ur"']


@pytest.fixture
def installed(monkeypatch):
    versions = {'already-here': '1.5', 'too-old': '1.0'}
    def version(name):
        if name in versions: return versions[name]
        raise metadata.PackageNotFoundError(name)
    monkeypatch.setattr(packages.metadata, 'requires', lambda _: DECLARED)
    monkeypatch.setattr(packages.metadata, 'version', version)


def test_only_what_this_extra_still_needs_on_this_computer(installed):
    assert packages.missing('trossen') == ['trossen-arm>=1.11', 'too-old>=2']


def test_ensure_installs_into_this_python_and_reports_a_failure(installed, monkeypatch):
    calls = []
    monkeypatch.setattr(subprocess, 'run', lambda argv, **kw: calls.append(argv) or subprocess.CompletedProcess(argv, 0, '', ''))
    packages.ensure('trossen', 'Trossen')
    assert calls[0][1:5] == ['-m', 'pip', 'install', '-q'] and calls[0][5:] == ['trossen-arm>=1.11', 'too-old>=2']
    monkeypatch.setattr(subprocess, 'run', lambda argv, **kw: subprocess.CompletedProcess(argv, 1, '', 'no matching build'))
    with pytest.raises(SystemExit, match='no matching build'):
        packages.ensure('trossen', 'Trossen')


def test_nothing_runs_when_nothing_is_missing(installed, monkeypatch):
    monkeypatch.setattr(subprocess, 'run', lambda *a, **k: pytest.fail('nothing to install'))
    packages.ensure('no-such-extra'); packages.ensure(None)


def test_a_source_checkout_is_left_alone(monkeypatch):
    def gone(_): raise metadata.PackageNotFoundError('roborun-sdk')
    monkeypatch.setattr(packages.metadata, 'requires', gone)
    assert packages.missing('trossen') == []


def test_prepare_covers_the_arms_and_cameras_but_not_own_drivers(monkeypatch):
    seen = []
    monkeypatch.setattr(packages, 'ensure', lambda extra, label=None: seen.append(extra))
    packages.prepare(dict(left=dict(arm='ur5e'), right=dict(arm='my_arm.py:MyArm'), cameras=dict(top='usb:0')))
    assert seen == ['ur', 'cameras']
