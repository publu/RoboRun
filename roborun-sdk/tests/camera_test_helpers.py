"""Helpers shared by the camera tests: a scripted capture source, a private socket folder, a running camera service and a
frame read through the runtime's camera client."""
import os
import shutil
import tempfile
import time
from pathlib import Path

import numpy as np
import pytest

from roborun_sdk import camera_capture_helper as helper
from roborun_sdk import cameras

W, H = 64, 48
K = dict(width=W, height=H, fx=50., fy=51., ppx=32., ppy=24., distortion_model='distortion.brown_conrady', coeffs=[0.] * 5)


def until(check, timeout=8., what='condition'):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            value = check()
            if value:
                return value
        except (RuntimeError, ValueError, OSError):
            pass
        time.sleep(.01)
    raise AssertionError(f'timed out waiting for {what}')


class Scripted(cameras.Source):
    """A capture source under test control; each pixel value carries the sequence number."""
    def __init__(self, name='fake', depth=True, period=.005, intrinsics=K):
        self.source, self.depth, self.period, self.k = f'fake:{name}', depth, period, intrinsics
        self.seq = 0; self.broken = False; self.paused = False; self.opens = 0

    def open(self):
        if self.broken:
            raise cameras.CameraError('absent', 'unplugged')
        self.opens += 1
        return dict(serial=self.source, intrinsics=self.k, depth_scale_m=.001 if self.depth else None, depth_available=self.depth,
                    depth_aligned_to='color' if self.depth else None, profiles=dict(color=dict(width=W, height=H)))

    def read(self):
        if self.broken:
            raise cameras.CameraError('stalled', 'the cable came out')
        time.sleep(self.period)
        if self.paused:
            return None
        self.seq += 1; now = time.monotonic()
        out = dict(rgb=np.full((H, W, 3), self.seq % 251, np.uint8), acquired_at_s=now - .002, depth_acquired_at_s=now - .003,
                   sdk_received_monotonic_s=now - .001, timestamp_domain='global_time', sensor_frame=self.seq)
        if self.depth:
            out['depth_z16'] = np.full((H, W), 1000 + self.seq % 1000, np.uint16)
        return out


@pytest.fixture
def run_dir():
    d = tempfile.mkdtemp(prefix='cam')
    os.chmod(d, 0o700)
    yield Path(d)
    helper.call(str(Path(d) / 'camera-capture.sock'), dict(op='shutdown'))
    shutil.rmtree(d, ignore_errors=True)


@pytest.fixture
def serve(run_dir):
    running = []
    def start(sources, **kw):
        kw.setdefault('backoff', (.02, .05)); kw.setdefault('stall_s', .5)
        broker = cameras.Broker(sources, **kw)
        path = str(run_dir / 'camera.sock')
        server = cameras.listen(broker, path); broker.start()
        running.append((broker, server))
        return broker, path
    yield start
    for broker, server in running:
        server.shutdown(); server.server_close(); broker.close()


def frame(path, role='top', **kw):
    """One frame through the robot runtime's own camera client; a test that reads frames skips where that client is absent."""
    cs = pytest.importorskip('camera_service')
    return cs.request('frame', role, address=path, **kw)
