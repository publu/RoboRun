"""Loaded by every Python of a guided-setup test (PYTHONPATH): the simulated USB world. Only what the operating system would
report changes here: which serial ports and cameras exist. The CLI, its drivers and the bus traffic are the real ones."""
import json
import os
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = os.environ.get('ROBORUN_FAKE_HW')


def _read(name, default):
    for _ in range(50):
        try:
            return json.loads((Path(ROOT) / name).read_text())
        except FileNotFoundError:
            return default
        except ValueError:
            time.sleep(.01)
    return default


_frames = {}


def _count_frame(ident):
    """Frames read per camera, for a test that waits until the setup has looked long enough (hw.py `frames`)."""
    _frames[ident] = _frames.get(ident, 0) + 1
    tmp = Path(ROOT) / f'frames.{os.getpid()}.tmp'
    tmp.write_text(json.dumps(_frames)); tmp.replace(Path(ROOT) / 'frames.json')


def _install():
    from serial.tools import list_ports
    list_ports.comports = lambda *_a, **_k: [SimpleNamespace(**row) for row in _read('ports.json', [])]
    from roborun_sdk.arms import canbus
    canbus._usb_all = lambda: []
    canbus._tty_holders = lambda device: []                # the simulated adapter itself keeps its pty open
    from roborun_sdk import cameras
    import numpy as np

    def listing(platform=None):
        return [(f"usb:{name}", f"{name}, RGB only") for name in _read('cameras.json', {})], []

    def _open(self):
        if self.ident not in _read('cameras.json', {}):
            raise cameras.CameraError('absent', f'USB camera {self.ident} is not connected')
        self.seq = 0
        return cameras._rgb_only_model(self.ident, 64, 48)

    def _frame(self):
        level = _read('cameras.json', {}).get(self.ident)
        if level is None:
            return None
        self.seq += 1
        _count_frame(self.ident)
        now = time.monotonic()
        image = np.full((48, 64, 3), int(level), np.uint8)
        image[:8, :8] = (self.seq * 7) % 255
        return dict(rgb=image, acquired_at_s=now, sdk_received_monotonic_s=now, timestamp_domain='host_arrival', sensor_frame=self.seq)

    cameras.list_usb = listing
    cameras.list_realsense = lambda _socket, platform=None: ([], [])
    cameras.UsbSource.open = _open
    cameras.UsbSource.read = _frame
    cameras.UsbSource.close = lambda self: None


if ROOT:
    try:
        _install()
    except Exception as exc:                     # a test then fails on the missing hardware, with this as the reason
        print(f'guided test hardware not installed: {exc!r}', flush=True)
