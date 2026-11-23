"""Each humanoid move, recorded as one episode of this robot's LeRobot v2.1 dataset (roborun_sdk.dataset.append_episode, at
roborun_sdk.robot.dataset_root(robot)): the head camera, the head's angles, the commanded twist and head target, and the base's
measured odometry and velocity, sampled together at FPS on one clock. Only the robot's own data and the user's prompt: never
anything generated. Writing happens after the move, in the background; a robot without the dataset extra moves exactly the
same and the move record says why nothing was recorded.
"""
from __future__ import annotations

import threading
import time

FPS = 10.0
FEATURES = {
    'observation.state': dict(names=['head_pitch', 'head_yaw'], units='rad', dtype='float32'),
    'action': dict(names=['vx', 'vy', 'vyaw', 'head_pitch', 'head_yaw'], units=['m/s', 'm/s', 'rad/s', 'rad', 'rad'], dtype='float32'),
    'observation.base_odometry': dict(names=['x', 'y', 'yaw'], units=['m', 'm', 'rad'], dtype='float32'),
    'observation.base_velocity': dict(names=['vx', 'vy', 'vyaw'], units=['m/s', 'm/s', 'rad/s'], dtype='float32'),
}
CAMERA = 'observation.images.head'


NOT_INSTALLED = 'the recording tools are not installed (roborun up sets them up)'


def _writer():
    """(append_episode, None) when episodes can be written now, else (None, the plain reason)."""
    try:
        from roborun_sdk import dataset
    except ImportError:
        return None, NOT_INSTALLED
    why = dataset.unavailable() if hasattr(dataset, 'unavailable') else None
    return (None, why) if why else (dataset.append_episode_background, None)    # stages cheaply; a nice-19 child encodes


class MoveRecorder:
    """start() before the move, stop(...) after it. Sampling never commands anything; a failed read repeats the last one."""

    def __init__(self, robot, profile_id, base, head, *, fps=FPS, clock=time.monotonic, writer=None, root=None):
        self.robot, self.profile_id, self.base, self.head, self.fps, self.clock = robot, profile_id, base, head, float(fps), clock
        self.writer, self.why = (writer, None) if writer is not None else _writer()
        self.root = root
        self.samples = []
        self._closed = threading.Event()
        self._thread = None
        self._target = (0., 0.)

    def start(self):
        if self.writer is None:
            return self
        self._thread = threading.Thread(target=self._loop, name='body-record', daemon=True)
        self._thread.start()
        return self

    def head_target(self, pitch, yaw):
        self._target = (float(pitch), float(yaw))

    def _sample(self):
        s = self.base.state()
        h = self.head.state()
        cmd = s.get('commanded') or {}
        try:
            jpeg = self.head.frame()['jpeg']
        except Exception:
            jpeg = self.samples[-1]['jpeg'] if self.samples else None
        tp, ty = self._target
        return dict(state=[float(h['pitch']), float(h['yaw'])],
                    action=[float(cmd.get('vx', 0.)), float(cmd.get('vy', 0.)), float(cmd.get('vyaw', 0.)), tp, ty],
                    odom=[float(s['x']), float(s['y']), float(s['yaw'])],
                    vel=[float(s.get('vx', 0.)), float(s.get('vy', 0.)), float(s.get('vyaw', 0.))], jpeg=jpeg)

    def _loop(self):
        period, t0, k = 1. / self.fps, self.clock(), 0
        while not self._closed.is_set():
            try:
                self.samples.append(self._sample())
            except Exception:
                if self.samples:
                    self.samples.append(dict(self.samples[-1]))
            k += 1
            self._closed.wait(max(0., t0 + k * period - self.clock()))

    def stop(self, *, prompt, extras=None):
        """Ends sampling and writes the episode (in the background by default). Returns {recorded, why?} for the move record."""
        if self.writer is None:
            return dict(recorded=False, why=self.why or NOT_INSTALLED)
        self._closed.set()
        if self._thread is not None:
            self._thread.join(timeout=2.)
        frames = [s for s in self.samples if s.get('jpeg')]
        if len(frames) < 2:
            return dict(recorded=False, why='too short to record')
        rows = {'observation.state': [s['state'] for s in frames], 'action': [s['action'] for s in frames],
                'observation.base_odometry': [s['odom'] for s in frames], 'observation.base_velocity': [s['vel'] for s in frames]}
        cameras = {CAMERA: [(i, s['jpeg']) for i, s in enumerate(frames)]}
        kw = dict(robot_type=self.profile_id, robot=self.robot, profile=self.profile_id, fps=self.fps, features=FEATURES,
                  rows=rows, cameras=cameras, prompt=prompt, extras=dict(extras or {}))
        if self.root is not None:
            kw['root'] = self.root
        else:
            from roborun_sdk.robot import dataset_root
            kw['root'] = dataset_root(self.robot)
        root = kw.pop('root')

        def write():
            try:
                self.writer(root, **kw)
            except Exception as exc:                          # the move is unaffected; the next one tries again
                print(f'[body-record] episode not written: {exc}'[:300], flush=True)
        write()                                           # cheap: the writer stages it and encodes in a nice-19 child
        return dict(recorded=True, frames=len(frames), fps=self.fps)
