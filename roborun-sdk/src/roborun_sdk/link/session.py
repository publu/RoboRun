"""Session client for manifest.wire.v1.

One device session per rig: ``hello`` -> ``admit``; then 10 Hz heartbeats both ways, the 272-byte ``state`` at 30 Hz
while executing / 2 Hz idle (latest wins), events/acks/gate reports (never dropped: rt keeps them until sent),
segments/controls/stops/decides relayed to roborun-rt, capture directives answered with observations.

Reconnect never resumes motion: when the link drops, rt's watchdog holds and latches the loss; a new session
reports the latched state and rest pose in ``hello``; the admit only clears the latch, and the scene-change mark
makes rt refuse every segment planned before it.
"""
from __future__ import annotations

import math
import time
from collections import deque

from .. import SDK_NAME, WIRE_VERSION
from ..rt import signing
from ..rt.clock import Clock
from ..rt.datatypes import Refused
from . import codec, wire_compat
from .transport import TransportClosed


class SessionRefused(ConnectionError):
    pass


def build_hello(info: dict, *, rig: str, device: str, seq: int, cameras: dict | None = None,
                stop_sources=("lan", "cli", "api")) -> dict:
    return wire_compat.hello(info, rig=rig, device=device, seq=seq, cameras=cameras, stop_sources=stop_sources)


class SessionClient:
    def __init__(self, rt, *, rig: str, device: str, clock: Clock | None = None, capture=None,
                 admit_keys: dict | None = None, allow_unsigned_admit: bool = False, heartbeat_hz: float = 10.0,
                 state_hz: tuple = (30.0, 2.0), cameras: dict | None = None, stop_sources=("lan", "cli", "api")):
        self.rt = rt
        self.rig, self.device = rig, device
        self.clock = clock or Clock()
        self.capture = capture
        self.admit_keys = admit_keys or {}
        self.allow_unsigned_admit = allow_unsigned_admit
        self.heartbeat_s = 1.0 / heartbeat_hz
        self.state_s = (1.0 / state_hz[0], 1.0 / state_hz[1])
        self.cameras = cameras or {}
        self.stop_sources = stop_sources
        self.transport = None
        self.session: str | None = None
        self.admit: dict | None = None
        self.seq_up = 0
        self.state_seq = 0
        self.api_seq_seen = 0
        self.rt_event_seq = 0
        self.last_hb = -math.inf
        self.last_state = -math.inf
        self.hb_gaps: deque = deque(maxlen=2048)
        self.last_api_msg: float | None = None
        self.log: deque = deque(maxlen=256)

    # ------------------------------------------------------------------ send
    def _next_seq(self) -> int:
        self.seq_up += 1
        return self.seq_up

    def _send_raw(self, msg: dict) -> None:
        self.transport.send(codec.encode(msg))

    def _send(self, local: dict) -> None:
        """Translate one local message (ack, event, gate report, observation) to the wire and send it."""
        seq = self._next_seq()
        kind = local.get("type")
        if kind == "ack":
            msg = wire_compat.ack(local, seq)
        elif kind == "observation":
            msg = wire_compat.observation(local, seq)
        elif kind in ("event", "gate_report"):
            msg = wire_compat.event(local, seq)
            if msg is None:
                self.seq_up -= 1                     # local bookkeeping only: no wire message, no seq consumed
                return
        else:
            msg = dict(local, seq=seq)
        self._send_raw(msg)

    # ------------------------------------------------------------------ connect
    def connect(self, transport, *, admit_timeout_s: float = 2.0) -> dict:
        self.transport = transport
        info = self.rt.info()
        self._send_raw(build_hello(info, rig=self.rig, device=self.device, seq=self._next_seq(), cameras=self.cameras,
                                   stop_sources=self.stop_sources))
        deadline = time.monotonic() + admit_timeout_s
        while True:
            raw = transport.recv(max(0.0, deadline - time.monotonic()))
            if raw is None:
                if time.monotonic() >= deadline:
                    raise SessionRefused("no admit within the timeout")
                continue
            msg = codec.decode(raw)
            if msg.get("type") == "stop":
                self._handle(msg, self.clock.now())       # a Stop is obeyed even before admission
                continue
            if msg.get("type") != "admit":
                self.log.append(("pre_admit_dropped", msg.get("type")))
                continue
            break
        ok, why = signing.verify(msg, "admit", self.admit_keys)
        if not ok and not (self.allow_unsigned_admit and why == "unsigned"):
            raise SessionRefused(f"admit signature: {why}")
        if msg.get("wire") != WIRE_VERSION or msg.get("result") not in ("accepted", "telemetry_only"):
            raise SessionRefused(f"admit refused: {msg.get('result')} {msg.get('reason', '')}")
        self.admit = msg
        self.session = str(msg.get("session"))
        admitted = {arm: bool((v or {}).get("admitted")) for arm, v in (msg.get("arms") or {}).items()}
        # The server half of arming: segments only come for `mode: run` tasks of an accepted session;
        # "telemetry_only" (unsupported wire, quota) leaves it off. Local verbs never depend on it.
        self.rt.session(session=self.session, transport=msg.get("transport", "wss"),
                        link_lost_ms=msg.get("link_lost_ms"), admitted=admitted or None,
                        armed_server=msg.get("result") == "accepted")
        hb_ms = msg.get("heartbeat_ms")
        if isinstance(hb_ms, (int, float)) and 20 <= hb_ms <= 1000:
            self.heartbeat_s = hb_ms / 1000.0
        rates = msg.get("state_hz") or {}
        self.state_s = (1.0 / float(rates.get("executing", 1 / self.state_s[0])),
                        1.0 / float(rates.get("idle", 1 / self.state_s[1])))
        self.last_api_msg = self.clock.now()
        return msg

    # ------------------------------------------------------------------ pump
    def pump(self, timeout_s: float = 0.01) -> None:
        raw = self.transport.recv(timeout_s)
        now = self.clock.now()
        if raw is not None:
            if self.last_api_msg is not None:
                self.hb_gaps.append(now - self.last_api_msg)
            self.last_api_msg = now
            self.rt.hb()                                  # any API message proves the link
            self._handle(codec.decode(raw), now)
        if now - self.last_hb >= self.heartbeat_s:
            self.last_hb = now
            self._send_raw({"type": "hb", "seq": self._next_seq(), "t": max(0.0, round(now, 6)),
                            "echo_seq": self.api_seq_seen})
        state = self.rt.state()
        executing = (state.get("kernel") or {}).get("mode") == "EXECUTING"
        if now - self.last_state >= self.state_s[0 if executing else 1]:
            self.last_state = now
            self.state_seq += 1
            p999 = 0
            if self.hb_gaps:
                gaps = sorted(self.hb_gaps)
                p999 = int(gaps[min(len(gaps) - 1, int(len(gaps) * 0.999))] * 1000)
            self.transport.send(codec.pack_state(state, seq=self.state_seq, api_seq_seen=self.api_seq_seen,
                                                 hb_gap_p999_ms=p999))
        for ev in self.rt.events(self.rt_event_seq):
            self.rt_event_seq = ev["seq"]
            out = dict(ev)
            out["rt_seq"] = out.pop("seq")
            self._send(out)                          # translated to the wire vocabulary (rt_seq kept in detail)

    def _handle(self, msg: dict, now: float) -> None:
        kind = msg.get("type")
        if isinstance(msg.get("seq"), int):
            self.api_seq_seen = max(self.api_seq_seen, msg["seq"])
        if kind == "hb":
            return
        if kind == "segment":
            ack = self.rt.submit(msg, received_t=now)
            self._send(ack)
            return
        if kind == "stop":
            self.rt.stop(msg, source="api")               # the 'stopped' event follows through rt's events
            return
        if kind == "control":
            try:
                result = self.rt.control(msg)
                self._send({"type": "event", "kind": "control_done", "sdk_t": now, "op": msg.get("op"),
                            "result": result})
            except Refused as r:
                self._send({"type": "event", "kind": "control_refused", "sdk_t": now, "op": msg.get("op"),
                            "control_seq": msg.get("seq"), **r.as_dict()})
            return
        if kind == "decide":
            try:
                self.rt.decide(msg)
            except Refused as r:
                self._send({"type": "event", "kind": "decide_refused", "sdk_t": now, "decide_seq": msg.get("seq"),
                            **r.as_dict()})
            return
        if kind == "capture":
            if self.capture is None:
                self._send({"type": "event", "kind": "capture_refused", "sdk_t": now, "capture": msg.get("capture"),
                            "reason": "no_camera_configured"})
                return
            try:
                obs = self.capture(msg)
                self.rt.observation(obs["obs"], obs["t_exposure"])
                self._send(obs)
            except Exception as exc:
                self._send({"type": "event", "kind": "capture_refused", "sdk_t": now, "capture": msg.get("capture"),
                            "reason": type(exc).__name__, "message": str(exc)[:300]})
            return
        if kind == "goaway":
            self.transport.close()
            raise TransportClosed("goaway")
        self.log.append(("ignored", kind))

    # ------------------------------------------------------------------ loop with reconnect
    def run(self, connect_fn, stop_event, *, backoff_s=(0.5, 5.0)) -> None:
        delay = backoff_s[0]
        while not stop_event.is_set():
            try:
                self.connect(connect_fn())
                delay = backoff_s[0]
                while not stop_event.is_set():
                    self.pump()
            except (TransportClosed, ConnectionError, OSError, SessionRefused, ValueError) as exc:
                self.log.append(("disconnected", f"{type(exc).__name__}: {exc}"[:200]))
                if self.transport is not None:
                    try:
                        self.transport.close()
                    except Exception:
                        pass
                stop_event.wait(delay)
                delay = min(backoff_s[1], delay * 2)
