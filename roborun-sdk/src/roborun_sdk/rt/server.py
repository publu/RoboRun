"""roborun-rt's local socket for roborun-link (a Unix socket).

Framing: 4-byte big-endian length + msgpack map, both ways; max 4 MiB. Requests ``{"op": ..., ...}``, replies
``{"ok": true, "result": ...}`` or ``{"ok": false, "error": {"reason", "message"}}``. The socket file is created
0600 (owner only). roborun-link may crash or reconnect at will: nothing here resumes motion, and a link that stops
relaying heartbeats makes the runtime's watchdog hold.

The link is not trusted for operator, time or replay facts: observation and receipt times are clamped to now by the
runtime, signed controls are ordered by seq there, and ``clear_stop`` needs the console token that ``roborun rt``
prints on its own console (the link never has it).
"""
from __future__ import annotations

import hashlib
import hmac
import os
import socket
import struct
import threading
from pathlib import Path

import msgpack

from . import signing
from .datatypes import OperatorConfirm, Refused

MAX_FRAME = 4 * 1024 * 1024


def send_frame(sock: socket.socket, obj) -> None:
    raw = msgpack.packb(obj, use_bin_type=True, default=_default)
    if len(raw) > MAX_FRAME:
        raise ValueError("frame too large")
    sock.sendall(struct.pack("!I", len(raw)) + raw)


def _default(obj):
    item = getattr(obj, "item", None)
    if callable(item):
        return item()
    tolist = getattr(obj, "tolist", None)
    if callable(tolist):
        return tolist()
    raise TypeError(f"cannot encode {type(obj).__name__}")


def _exact(sock: socket.socket, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        block = sock.recv(n - len(out))
        if not block:
            raise ConnectionError("peer closed")
        out.extend(block)
    return bytes(out)


def recv_frame(sock: socket.socket):
    size = struct.unpack("!I", _exact(sock, 4))[0]
    if size > MAX_FRAME:
        raise ValueError("frame too large")
    return msgpack.unpackb(_exact(sock, size), raw=False, strict_map_key=False)


def kernel_sha256() -> str:
    """Digest of the rt sources (sorted). Any kernel change is a major SDK version."""
    h = hashlib.sha256()
    root = Path(__file__).resolve().parent
    for path in sorted(root.rglob("*.py")):
        h.update(str(path.relative_to(root)).encode() + b"\0" + path.read_bytes())
    return h.hexdigest()


class RtServer:
    def __init__(self, runtime, path: str | Path, *, console_token: str | None = None):
        self.runtime = runtime
        self.path = str(path)
        self.console_token = console_token      # local operator credential for clear_stop (None: no socket clear)
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        self._threads: list = []

    def start(self) -> "RtServer":
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        old = os.umask(0o177)
        try:
            sock.bind(self.path)
        finally:
            os.umask(old)
        sock.listen(4)
        sock.settimeout(0.2)
        self._sock = sock
        t = threading.Thread(target=self._accept, name="roborun-rt-socket", daemon=True)
        t.start()
        self._threads.append(t)
        return self

    def close(self) -> None:
        self._stop.set()
        if self._sock is not None:
            self._sock.close()
        for t in self._threads:
            t.join(1.0)
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass

    def _accept(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            t = threading.Thread(target=self._serve, args=(conn,), name="roborun-rt-conn", daemon=True)
            t.start()

    def _serve(self, conn: socket.socket) -> None:
        with conn:
            while not self._stop.is_set():
                try:
                    req = recv_frame(conn)
                except (ConnectionError, OSError, ValueError):
                    return
                try:
                    result = self.dispatch(req)
                    reply = {"ok": True, "result": result}
                except Refused as r:
                    reply = {"ok": False, "error": r.as_dict()}
                except Exception as exc:  # never kill the server on a bad request
                    reply = {"ok": False, "error": {"reason": "rt_error", "message": f"{type(exc).__name__}: {exc}"[:300]}}
                try:
                    send_frame(conn, reply)
                except OSError:
                    return

    def dispatch(self, req: dict):
        rt = self.runtime
        if not isinstance(req, dict):
            raise Refused("malformed", "requests are maps")
        op = req.get("op")
        if op == "info":
            return info(rt)
        if op == "hb":
            rt.link_heartbeat()
            return None
        if op == "submit":
            return rt.submit(req.get("msg"), received_t=req.get("received_t"))
        if op == "stop":
            return stop_message(rt, req.get("msg") or {}, source_prefix=req.get("source", "api"))
        if op == "control":
            return rt.control(req["msg"])
        if op == "decide":
            return rt.decide(req["msg"])
        if op == "session":
            return rt.session_started(req["session"], transport=req.get("transport"), link_lost_ms=req.get("link_lost_ms"),
                                      admitted=req.get("admitted"), armed_server=req.get("armed_server"))
        if op == "state":
            return rt.state()
        if op == "events":
            return rt.drain_events(int(req.get("after", 0)))
        if op == "observation":
            rt.register_observation(req.get("obs"), req.get("t"))       # validated and clamped to now by rt
            return None
        if op == "samples":
            return samples(rt)
        if op == "offer_pack":
            return rt.offer_pack(req["raw"])
        if op == "clear_stop":
            token = req.get("console_token")
            if (not self.console_token or not isinstance(token, str)
                    or not hmac.compare_digest(token.encode(), self.console_token.encode())):
                raise Refused("operator_confirmation_required", "clearing a Stop over the rt socket needs the console "
                              "token `roborun rt` printed on its own console (roborun-link never has it)")
            operator = req.get("operator")
            if not isinstance(operator, str) or not operator.strip():
                raise Refused("operator_confirmation_required", "name the operator who clears the Stop")
            return rt.clear_stop(OperatorConfirm(operator.strip()[:64], "clear stop", rt.clock.now(), "console"))
        raise Refused("unsupported_op", f"op {op!r}")


def info(rt) -> dict:
    return {"arms": {name: {**d.public(), "health": rt.health.public()[name]["state"],
                            "interlocks": rt.interlocks.public()[name]} for name, d in rt.descriptors.items()},
            "pack": rt.pack.digest, "clock": rt.clock.describe(), "kernel_sha256": kernel_sha256(),
            "armed": rt.armed, "latched": {"stopped": rt.stop_latch.latched, "link_lost": rt.link.latched_lost,
                                           "faults": sorted(rt.health.faults())},
            "calibration": rt.calibration.get("id"), "signing_backend": signing.BACKEND}


def samples(rt) -> dict:
    now = rt.clock.now()
    out = {}
    for name, s in rt.samples.items():
        if s is not None:
            out[name] = {"sample": s.sample_id, "t": s.t, "age_s": now - s.t + s.clock_uncertainty_s,
                         "q": list(s.q), "jaw": s.jaw, "q_cmd": None if s.q_cmd is None else list(s.q_cmd),
                         "jaw_cmd": s.jaw_cmd}
    return {"now": now, "arms": out}


def _text(value, default, n: int = 64):
    return value[:n] if isinstance(value, str) and value else default


def stop_message(rt, msg, *, source_prefix: str = "api") -> dict:
    """A relayed Stop (no preconditions). It latches and holds first, whatever the message looks like;
    its signature is checked afterwards, while the holds are in flight, only to fill ``verified`` (an unverified Stop
    is obeyed and logged). Nothing in the message can raise before the latch."""
    m = msg if isinstance(msg, dict) else {}

    def verify() -> bool:
        if not m.get("sig"):
            return False
        return signing.verify(m, "stop", rt.trusted_keys)[0] is True
    source = f"{_text(source_prefix, 'api')}:{_text(m.get('source'), 'relay')}"
    return rt.stop(source, stop_id=_text(m.get("id"), None), user_role=_text(m.get("user_role"), None), verify=verify)
