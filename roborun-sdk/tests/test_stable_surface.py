"""The stable surface (docs/BUILDING.md "Versions and upgrades") is only added to within a series: tools/stable_surface.py
compares it with the last release's snapshot and fails on a removal or change that CHANGELOG.md does not name. Also: the
removed Robot.clip() is a stub with a clear error, and the changelog ships with the package (roborun docs changelog)."""
import importlib.util
import json
import re
from pathlib import Path

import pytest

from roborun_sdk import cli
from roborun_sdk.client import Robot, RobotError, RobotUnavailable
from roborun_sdk.errors import RoborunError

SDK = Path(__file__).resolve().parents[1]
SRC = SDK / 'src' / 'roborun_sdk'


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


tool = load(SDK / 'tools' / 'stable_surface.py', 'stable_surface_under_test')
SNAPSHOT = json.loads(tool.SNAPSHOT.read_text())


def tree_with(tmp_path, edits):
    """A copy of the stable files with `edits` {file: (old, new)} applied: a release that changed something."""
    root = tmp_path / 'roborun_sdk'
    for rel in [*tool.STABLE, 'cameras.py']:
        (root / rel).parent.mkdir(parents=True, exist_ok=True)
        text = (SRC / rel).read_text()
        for old, new in edits.get(rel, ()):
            assert old in text, (rel, old)
            text = text.replace(old, new)
        (root / rel).write_text(text)
    return tmp_path


def test_the_source_keeps_every_stable_name_of_the_last_release_or_the_changelog_says_why(capsys):
    assert tool.main([]) == 0, capsys.readouterr().out
    assert re.fullmatch(r'0\.1\.\d{14}', SNAPSHOT['release'])
    assert 'roborun_sdk.client:Robot.run' in SNAPSHOT['surface'] and 'roborun_sdk.arms:open_arm' in SNAPSHOT['surface']


def test_the_surface_covers_what_building_md_calls_stable():
    now = tool.surface(SDK)
    for key in ('roborun_sdk.client:Robot.stop', 'roborun_sdk.client:RobotUnavailable', 'roborun_sdk.client:Task.outcome',
                'roborun_sdk.client:StopStatus.state', 'roborun_sdk.errors:RoborunError', 'roborun_sdk.arms:open_arm',
                'roborun_sdk.arms.base:StreamedArm.write_hardware', 'roborun_sdk.arms.base:StreamedArm.__init__'):
        assert key in now, key
    assert now['roborun_sdk.arms.base:rules check()'] == '4 positional'                 # check(side, positions_rad, gripper, state)
    assert now['roborun_sdk.cameras:camera source class'] == ('__init__ may take fps, height, role, width; '
                                                              'called: close, open, read, set_exposure_us')
    assert not any(k.startswith('roborun_sdk.arms.base:StreamedArm.') and not (k.endswith('_hardware') or k.endswith('__init__'))
                   for k in now)                                  # its other methods are Roborun's, not the driver's surface
    building = (SDK / 'docs' / 'BUILDING.md').read_text()
    stable = building.split('What stays compatible within a series', 1)[1].split('An incompatible change', 1)[0]
    for name in tool.PUBLIC_NAMES:
        assert f'`{name}' in stable or f'{name}`' in stable, name


@pytest.mark.parametrize('rel, edits, key, kind', [
    ('client.py', [('    def recording(self, task_id: str)', '    def _recording(self, task_id: str)')], 'Robot.recording', 'removed'),
    ('client.py', [('    recording_url: Optional[str]\n', '')], 'Task.recording_url', 'removed'),
    ('client.py', [('def wait(self, task_id: str, timeout: Optional[float] = None,', 'def wait(self, task_id: str, timeout: float,')],
     'Robot.wait', 'changed'),
    ('arms/__init__.py', [('def open_arm(arm: str, side: str, bus=None,', 'def open_arm(arm: str, side: str, *, bus=None,')],
     'open_arm', 'changed'),
    ('arms/base.py', [('    def write_hardware(self, positions_rad, gripper):', '    def write_hardware(self, targets):')],
     'StreamedArm.write_hardware', 'changed'),
    ('arms/base.py', [('self._rule(self.side, ', 'self._rule(')], 'rules check()', 'changed'),
    ('cameras.py', [("getattr(self.camera, 'set_exposure_us', None)", "getattr(self.camera, 'exposure', None)"),
                    ('self.camera.set_exposure_us(', 'self.camera.exposure(')], 'camera source class', 'changed'),
])
def test_a_stable_name_removed_or_changed_fails_until_the_changelog_names_it(tmp_path, rel, edits, key, kind):
    release = tool.surface(tree_with(tmp_path / 'a', {}))
    snapshot = dict(release='0.1.20261121012822', surface=release)
    broken = tool.surface(tree_with(tmp_path / 'b', {rel: edits}))
    assert [(tool.short(k), why) for k, why, *_ in tool.check(broken, snapshot, '')] == [(key, kind)]
    entry = f"**{kind.capitalize()}**\n\n- `{key}` {kind}: why, and what to use instead\n"
    assert tool.check(broken, snapshot, f'# Changelog\n\n## Unreleased\n\n{entry}') == []
    assert tool.check(broken, snapshot, f'## 0.1.20261122000000 (build x)\n\n{entry}') == []          # a later release
    stale = f'## Unreleased\n\n- other\n\n## 0.1.20261121012822 (build x)\n\n{entry}\n## 0.1.20261119203708\n\n{entry}'
    assert len(tool.check(broken, snapshot, stale)) == 1         # named only for that release or before: not for this change


def test_only_an_exact_name_under_removed_or_changed_excuses_a_break(tmp_path):
    """A mention under **Added**, in prose, or of a longer name (`Robot.stop_task()` for `Robot.stop`) is not an entry."""
    release = tool.surface(tree_with(tmp_path / 'a', {}))
    snapshot = dict(release='0.1.20261121012822', surface=release)
    stopless = tool.surface(tree_with(tmp_path / 'b', {'client.py': [('    def stop(self) -> StopResult:',
                                                                        '    def _stop(self) -> StopResult:')]}))
    for text in ('## Unreleased\n\n**Removed**\n\n- `Robot.stop_task()`: gone\n',
                 '## Unreleased\n\n**Added**\n\n- `Robot.stop`: a note\n',
                 '## Unreleased\n\n- `Robot.stop` removed\n',                     # no heading
                 '## Unreleased\n\n**Removed**\n\n- Robot.stop, without backticks\n',
                 '## Unreleased\n\n**Removed**\n\n- `roborun_sdk.client.Robot.stopped`\n',
                 '## Unreleased\n\n**Removed**\n\n- x\n\n**Added**\n\n- `Robot.stop`\n'):  # the heading ends at the next one
        assert [tool.short(k) for k, *_ in tool.check(stopless, snapshot, text)] == ['Robot.stop'], text
    for text in ('## Unreleased\n\n**Removed**\n\n- `Robot.stop`: why\n',
                 '## Unreleased\n\n**Changed**\n\n- `Robot.stop()` is now `Robot.halt()`\n',
                 '## Unreleased\n\n**Removed** (a break within the 0.1 series)\n\n- `Robot.stop()`: why\n'):
        assert tool.check(stopless, snapshot, text) == [], text


def test_the_real_changelog_does_not_excuse_names_it_only_mentions(tmp_path):
    """CHANGELOG.md's Unreleased section mentions `Robot.recording(task_id)`, `RobotError` and `StreamedArm` under
    **Added**: changing them must still fail the check."""
    snapshot = dict(SNAPSHOT, surface=tool.surface(tree_with(tmp_path / 'a', {})))
    broken = tool.surface(tree_with(tmp_path / 'b', {
        'client.py': [('    def recording(self, task_id: str)', '    def recording(self, task_id: str, *, frames: bool)'),
                      ('class RobotError(RoborunError):', 'class RobotError(Exception):')],
        'arms/base.py': [('class StreamedArm(ArmAdapter):', 'class StreamedArm(object):')]}))
    text = (SDK / 'CHANGELOG.md').read_text()
    assert all(f'`{n}' in tool.changelog_since(text, SNAPSHOT['release']) for n in ('Robot.recording(', 'RobotError', 'StreamedArm'))
    assert sorted((tool.short(k), why) for k, why, *_ in tool.check(broken, snapshot, text)) == [
        ('Robot.recording', 'changed'), ('RobotError', 'changed'), ('StreamedArm', 'changed')]


def test_a_new_name_needs_no_entry_and_the_two_release_compare_reports_it(tmp_path, capsys):
    old = tree_with(tmp_path / 'a', {'client.py': [('    def clip(self, task_id: str) -> NoReturn:', '    def _gone(self, task_id: str) -> NoReturn:')]})
    new = tree_with(tmp_path / 'b', {})
    snapshot = dict(release='0.1.20261121012822', surface=tool.surface(old))
    assert tool.check(tool.surface(new), snapshot, '') == []
    assert tool.main(['--old', str(old), '--new', str(new)]) == 0
    assert 'added    roborun_sdk.client:Robot.clip' in capsys.readouterr().out
    assert tool.main(['--old', str(new), '--new', str(old)]) == 1
    assert 'removed  roborun_sdk.client:Robot.clip' in capsys.readouterr().out


def test_update_refuses_a_break_and_needs_the_release(tmp_path, monkeypatch, capsys):
    snap = tmp_path / 'stable_surface.json'
    snap.write_text(tool.SNAPSHOT.read_text())
    monkeypatch.setattr(tool, 'SNAPSHOT', snap)
    monkeypatch.setattr(tool, 'CHANGELOG', tmp_path / 'CHANGELOG.md')
    broken = tree_with(tmp_path / 'b', {'arms/__init__.py': [('def open_arm(', 'def open_any_arm(')]})
    assert tool.main(['--update', '--release', '0.1.20261122000000', '--new', str(broken)]) == 1
    assert '`open_arm`' in capsys.readouterr().out and snap.read_text() == tool.SNAPSHOT.read_text()
    (tmp_path / 'CHANGELOG.md').write_text('## Unreleased\n\n**Removed**\n\n- `open_arm`: renamed open_any_arm\n')
    with pytest.raises(SystemExit):
        tool.main(['--update', '--new', str(broken)])
    assert tool.main(['--update', '--release', '0.1.20261122000000', '--new', str(broken)]) == 0
    written = json.loads(snap.read_text())
    assert written['release'] == '0.1.20261122000000' and 'roborun_sdk.arms:open_arm' not in written['surface']


def test_clip_says_it_was_removed_and_asks_the_robot_for_nothing(monkeypatch):
    robot = Robot('http://127.0.0.1:9')
    monkeypatch.setattr(robot, '_open', lambda *a, **k: pytest.fail('clip() sent a request'))
    with pytest.raises(RobotError) as caught:
        robot.clip('t-1')
    err = caught.value
    assert (err.status, err.code, err.retryable) == (410, 'removed', False) and not isinstance(err, RobotUnavailable)
    assert isinstance(err, RoborunError) and "Robot.recording('t-1')" in err.message and 'removed' in str(err)
    assert 'recording(task_id)' in Robot.clip.__doc__
    assert not re.search(r'generated|video', Robot.clip.__doc__ + err.message, re.I)    # says nothing of what the service keeps
    source = (SRC / 'client.py').read_text()
    assert not re.search(r'/clip\b|clip_url', source)              # it names no clip route or field


def test_the_changelog_ships_names_every_release_and_prints(tmp_path, capsys):
    text = (SDK / 'CHANGELOG.md').read_text()
    assert not re.search(r'/clip\b|clip_url|\.mp4', text)
    assert not re.search(r'generated[ _-]?(video|clip)', text, re.I)     # what the service keeps is never described
    releases = re.findall(r'^## (0\.1\.\d{14}) \(build (\d{8}-\d{6})-[0-9a-f]{10}\)', text, re.M)
    assert releases[0][0] == SNAPSHOT['release'] and len(releases) >= 17     # grows by one each release
    assert releases[-1][0] == '0.1.20261119203708'                           # the series' first release
    for version, build in releases:                              # a version and its build id name the same time
        assert version.split('.')[2] == build.replace('-', '')
    assert [v for v, _ in releases] == sorted((v for v, _ in releases), reverse=True)
    removal = text.split('## 0.1.20261119224249', 1)[1].split('\n## ', 1)[0]
    assert '**Removed**' in removal and '`Robot.clip()`' in removal and 'Robot.recording(task_id)' in removal
    assert cli.main(['docs', 'changelog']) == 0 and capsys.readouterr().out.startswith('# Changelog: roborun-sdk')



def test_unreleased_names_what_is_merged_but_not_released():
    """Each feature merged since the last release has its line under Unreleased before it ships."""
    text = (SDK / 'CHANGELOG.md').read_text()
    unreleased = text.split('## Unreleased', 1)[1].split('\n## ', 1)[0]
    added, changed = unreleased.split('**Changed**', 1)
    assert '**Added**' in added and '**Removed**' not in unreleased.split('**Added**', 1)[0]
    assert '`roborun-sdk[robot]`' in changed                                    # the light install
    assert '`roborun up --body file.py:Class`' in added and '`<ROBORUN_HOME>/bodies/<id>.yaml`' in added
    assert '`body_kind`' in added and 'docs/BUILDING.md#your-own-robot-body' in added   # custom bodies
    assert "`robot.yaml`'s `body:`" in changed
    anchors = re.findall(r'docs/BUILDING\.md#([a-z0-9-]+)', unreleased)
    building = (SDK / 'docs' / 'BUILDING.md').read_text()
    heads = {re.sub(r'[^a-z0-9 -]', '', h.lower()).replace(' ', '-') for h in re.findall(r'^#+ (.+)$', building, re.M)}
    assert anchors and set(anchors) <= heads, set(anchors) - heads
