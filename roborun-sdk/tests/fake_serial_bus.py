"""A pseudo-terminal the driver opens like a USB serial adapter; a family's fake servos answer on the other end."""
import os
import select
import threading
import time
import tty


class FakeSerialBus:
    tick_s = .002

    def __init__(self):
        self.master, slave = os.openpty()
        tty.setraw(self.master); tty.setraw(slave)
        self.device = os.ttyname(slave)
        self.slave = slave
        self.address = f'serial://{self.device}'
        self.rx = bytearray()
        self.packets = []
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def close(self):
        self._stop.set(); self.thread.join(2)
        os.close(self.master); os.close(self.slave)

    def reply(self, data):
        os.write(self.master, bytes(data))

    def consume(self): raise NotImplementedError
    def physics(self, dt): raise NotImplementedError

    def _run(self):
        last = time.monotonic()
        while not self._stop.is_set():
            ready, _, _ = select.select([self.master], [], [], self.tick_s)
            if ready:
                try:
                    self.rx += os.read(self.master, 4096)
                except OSError:
                    return
                self.consume()
            now = time.monotonic()
            if now - last >= self.tick_s:
                self.physics(now - last); last = now
