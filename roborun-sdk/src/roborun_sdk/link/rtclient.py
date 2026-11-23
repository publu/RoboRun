"""roborun-link's handle on roborun-rt: over the rt Unix socket (``RtClient``), or in-process (``InProcessRt``,
tests and the single-process CLI). Same methods either way."""
from __future__ import annotations

import socket
import threading

from ..rt import server as rtserver
from ..rt.datatypes import Refused


class RtClient:
    def __init__(self, path: str, *, timeout_s: float = 2.0):
        self.path = path
        self.timeout_s = timeout_s
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()

    def _call(self, op: str, **kw):
        with self._lock:
            if self._sock is None:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.settimeout(self.timeout_s)
                s.connect(self.path)
                self._sock = s
            try:
                rtserver.send_frame(self._sock, {"op": op, **kw})
                reply = rtserver.recv_frame(self._sock)
            except (OSError, ConnectionError, ValueError):
                self._sock.close()
                self._sock = None
                raise
        if not reply.get("ok"):
            err = reply.get("error") or {}
            raise Refused(err.get("reason", "rt_error"), err.get("message", ""))
        return reply.get("result")

    def close(self):
        with self._lock:
            if self._sock is not None:
                self._sock.close()
                self._sock = None

    def info(self): return self._call("info")
    def hb(self): return self._call("hb")
    def submit(self, msg, received_t=None): return self._call("submit", msg=msg, received_t=received_t)
    def stop(self, msg, source="api"): return self._call("stop", msg=msg, source=source)
    def control(self, msg): return self._call("control", msg=msg)
    def decide(self, msg): return self._call("decide", msg=msg)
    def session(self, **kw): return self._call("session", **kw)
    def state(self): return self._call("state")
    def events(self, after=0): return self._call("events", after=after)
    def observation(self, obs, t): return self._call("observation", obs=obs, t=t)
    def samples(self): return self._call("samples")
    def clear_stop(self, operator, console_token):
        """Local operator only: the console token is printed by `roborun rt` on its console, never given to the link."""
        return self._call("clear_stop", operator=operator, console_token=console_token)


class InProcessRt:
    """Same surface over a Runtime object in this process (no socket)."""

    def __init__(self, runtime):
        self.rt = runtime

    def info(self): return rtserver.info(self.rt)
    def hb(self): return self.rt.link_heartbeat()
    def submit(self, msg, received_t=None): return self.rt.submit(msg, received_t=received_t)
    def stop(self, msg, source="api"): return rtserver.stop_message(self.rt, msg, source_prefix=source)
    def control(self, msg): return self.rt.control(msg)
    def decide(self, msg): return self.rt.decide(msg)
    def session(self, **kw):
        return self.rt.session_started(kw.pop("session"), **kw)
    def state(self): return self.rt.state()
    def events(self, after=0): return self.rt.drain_events(after)
    def observation(self, obs, t): return self.rt.register_observation(obs, t)
    def samples(self): return rtserver.samples(self.rt)
    def close(self): pass
