"""connector-hid: USB pedal or button as a Stop source (K1).

Reads Linux input events straight from the device node (no evdev library): ``struct input_event`` is
(timeval sec, usec, type u16, code u16, value s32); any EV_KEY press (value 1) of an accepted key calls the callback
with ``"pedal"``. Runs inside roborun-rt (a crash of roborun-link cannot take Stop with it).

Not handled here: a udev rule, a stable /dev/input/by-id path, exclusive grab (EVIOCGRAB) and hot-plug. ``HidPedal.from_fd`` lets tests
feed events through a pipe.
"""
from __future__ import annotations

import os
import struct
import threading
import time

EVENT = struct.Struct("llHHi")
EV_KEY = 1


class HidPedal:
    def __init__(self, path: str | None = None, *, fd: int | None = None, keys: set[int] | None = None,
                 debounce_s: float = 0.2):
        if (path is None) == (fd is None):
            raise ValueError("give a device path or an fd")
        self.path = path
        self._fd = fd
        self.keys = keys
        self.debounce_s = debounce_s
        self._callbacks = []
        self._thread = None
        self._stop = threading.Event()
        self._last = 0.0
        self.presses = 0

    @classmethod
    def from_fd(cls, fd: int, **kw) -> "HidPedal":
        return cls(fd=fd, **kw)

    def on_press(self, callback) -> None:
        self._callbacks.append(callback)

    def start(self) -> None:
        if self._fd is None:
            self._fd = os.open(self.path, os.O_RDONLY)
        self._thread = threading.Thread(target=self._loop, name="manifest-hid-pedal", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        fd, self._fd = self._fd, None
        if fd is not None and self.path is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = os.read(self._fd, EVENT.size * 32)
            except (OSError, TypeError):
                return
            if not chunk:
                return
            buf += chunk
            while len(buf) >= EVENT.size:
                _, _, etype, code, value = EVENT.unpack_from(buf)
                buf = buf[EVENT.size:]
                if etype == EV_KEY and value == 1 and (self.keys is None or code in self.keys):
                    now = time.monotonic()
                    if now - self._last >= self.debounce_s:
                        self._last = now
                        self.presses += 1
                        for cb in list(self._callbacks):
                            try:
                                cb("pedal")
                            except Exception:
                                pass


def event_bytes(code: int = 30, value: int = 1, etype: int = EV_KEY) -> bytes:
    """One packed input_event (tests and bench rigs)."""
    now = time.time()
    return EVENT.pack(int(now), int((now % 1) * 1e6), etype, code, value)
