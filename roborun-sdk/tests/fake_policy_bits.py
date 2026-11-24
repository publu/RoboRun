"""Pieces for the policy-run tests: a camera class for the broker, scripted providers (pure Python, nothing remote) and the
tests' own Roborun home."""
import threading
import time

from roborun_sdk.policy import Chunk, Needs, Provider, ProviderError, plain_needs


def own_home(tmp_path, monkeypatch):
    """The test's own Roborun home (ROBORUN_HOME and robot.HOME, so the simulation's runtime copy is the test's too), with no
    robot.json until a test installs one (robot.CONFIG), its own $HOME and its own account home: never this computer's. On a
    CI runner $HOME is the account's own folder and may hold a default install, whose robot.json and runtime (config.json,
    recorder) a policy run would otherwise read as the robot's (the same rule as test_install_lifecycle's home)."""
    from roborun_sdk import robot
    h = tmp_path / 'home'
    user = tmp_path / 'user'
    user.mkdir(exist_ok=True)
    monkeypatch.setenv('HOME', str(user))
    monkeypatch.delenv('MANIFEST_DREAMAPI', raising=False)        # robot._manifest_root() reads it before any home
    monkeypatch.setenv('ROBORUN_HOME', str(h))
    monkeypatch.setattr(robot, 'HOME', h)
    monkeypatch.setattr(robot, 'CONFIG', h / 'robot.json')
    monkeypatch.setattr(robot, 'account_home', lambda: tmp_path / 'account' / '.config' / 'roborun')
    return h


class FakeCam:
    """A 64x48 camera for cameras.CustomSource: a moving gradient, one new frame per read every 1/30 s. Its serial reads as a
    real camera's, so its runs join the dataset (SimCam's do not)."""
    SERIAL = 'testcam-{role}'

    def __init__(self, role='top'):
        self.role, self.last, self.n = role, 0.0, 0

    def open(self):
        return dict(serial=self.SERIAL.format(role=self.role))

    def read(self):
        import numpy as np
        now = time.monotonic()
        if now - self.last < 1 / 30:
            return None
        self.last, self.n = now, self.n + 1
        img = np.zeros((48, 64, 3), np.uint8)
        img[:, :, 0] = (np.arange(64)[None, :] * 4 + self.n) % 255
        return img


class SimCam(FakeCam):
    """The simulator's camera, as far as a recording can tell: its serial is mujoco-<role>."""
    SERIAL = 'mujoco-{role}'


class Sway(Provider):
    """Sways one joint (the wrist) by `amplitude` rad around where the arm started, at `period` s; records what it got."""
    name = 'sway'

    def __init__(self, arms=('left',), amplitude=0.1, period=2.0, joint=-1, cameras=None, act_s=0.0, hz=30.0,
                 extra_side=None, remote=False):
        self.arms, self.amplitude, self.period, self.joint = tuple(arms), amplitude, period, joint
        self.cameras, self.act_s, self.hz, self.extra_side, self.remote = dict(cameras or {}), act_s, hz, extra_side, remote
        self.calls, self.observed, self.acted, self.started, self.ended, self.closed = [], [], [], 0, 0, 0
        self.t0 = None
        self.lock = threading.Lock()

    def needs(self, spec):
        self.spec = spec
        self.gripper = {a.side: a.gripper for a in spec.arms}
        self.calls.append('needs')
        return Needs(arms=self.arms, cameras=self.cameras, frame_hz=5.0, frame_size=None, remote=self.remote,
                     sends=('the prompt', 'joint angles') if self.remote else (), version='sway 1')

    def start(self, spec):
        self.calls.append('start')
        self.started += 1

    def observe(self, obs):
        self.observed.append(obs)

    def act(self, obs):
        import math
        self.acted.append((obs, time.monotonic()))
        if self.act_s:
            time.sleep(self.act_s)
        rows = {}
        for side in self.arms + ((self.extra_side,) if self.extra_side else ()):
            st = obs.arms[side]
            if self.t0 is None:
                self.t0 = (obs.t, dict((s, list(obs.arms[s].commanded_rad)) for s in obs.arms))
            base = list(self.t0[1][side])
            out = []
            for i in range(48):
                t = obs.t - self.t0[0] + i / self.hz
                q = list(base)
                q[self.joint] += self.amplitude * math.sin(2 * math.pi * t / self.period)
                out.append(q + ([1.0] if self.gripper[side] else []))
            rows[side] = out
        return Chunk(rows, hz=self.hz, meta=dict(provider_ms=1.0))

    def end_episode(self):
        self.calls.append('end_episode')
        self.ended += 1

    def close(self):
        self.calls.append('close')
        self.closed += 1


class Raises(Sway):
    def __init__(self, error=None, after=0, **kw):
        super().__init__(**kw)
        self.error, self.after, self.n = error, after, 0

    def act(self, obs):
        self.n += 1
        if self.n > self.after:
            raise self.error
        return super().act(obs)


class Warming(Sway):
    """Warming for `warm` calls (with retry_after_s), and the clock jumps `jump_s` on the first call (a long cold start)."""
    def __init__(self, warm=0, jump_s=0.0, clock=None, **kw):
        super().__init__(**kw)
        self.warm, self.jump_s, self.clock, self.n = warm, jump_s, clock, 0

    def act(self, obs):
        self.n += 1
        if self.n == 1 and self.jump_s and self.clock is not None:
            self.clock.offset += self.jump_s
        if self.n <= self.warm:
            raise ProviderError('pool warming', warming=True, retry_after_s=0.05)
        return super().act(obs)


class FileProvider(Sway):
    """Loaded by name from this file in a test (file.py:Class)."""
    name = 'file-sway'


def local_needs(arms):
    return plain_needs(arms)
