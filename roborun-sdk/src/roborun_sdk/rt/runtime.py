"""roborun-rt runtime: one tick loop over the connectors, the kernel and the player.

Tick (30 Hz by default, the arms' control rate):
  1. watchdog: a tick > 20 ms late raises ``tick_late``; > 50 ms late ends the running segment and holds the rig
     (``tick_overrun``)
  2. samples + K6 health (fault latch disarms, refuses and holds the rig) + history for the rest test
  3. K2 link watchdog (LOST: hold, revoke grants, latch, scene-change mark; never a retreat)
  4. playback: freshness (K4 fresh-pose hold, 0.5 s budget), K5 built-ins and triggers, proposal, K3 limits,
     K4 swept proximity, connector write, write-then-recheck of the Stop latch, end gate
Stop (K1) never waits for the tick: it latches, holds every owner in parallel, then cancels under the lock.

Who gets held. Explicit Stop sources (pedal, LAN UDP/HTTP, CLI/SIGUSR1, the relayed API Stop) and the ``hold`` control
reach every owner, whatever it is doing. Automatic kernel holds (a K6 fault, a tick overrun, link loss, a segment's
own end) are sent only while rt commands an arm (a running segment or an open grant): an arm without a grant is
already holding as far as rt is concerned (K2), and rt never preempts another authority's motion on its own (the
robot runtime's leases and the native 95 C thermal retreat, which only an explicit Stop may interrupt).
"""
from __future__ import annotations

import gc
import math
import threading
from collections import OrderedDict, deque

import numpy as np

from . import acceptance, signing
from .clock import Clock, sample_age, stamp_ok
from .datatypes import ConnectorError, Event, Grant, NotWired, OperatorConfirm, Refused
from .kernel import limits as k3
from .kernel import reflexes as k5
from .kernel.health import HealthMonitor
from .kernel.interlocks import Interlocks
from .kernel.link import LinkWatchdog
from .kernel.proximity import CapsuleProximity
from .kernel.stop import StopLatch
from .pack import Pack, PackStore
from .player import Playback
from .rest import History
from .segment import SIG_KIND, parse_segment

LATE_EVENT_S = 0.020
LATE_HOLD_S = 0.050
REFRESH_TIMEOUT_S = 0.050          # the synchronous start-sample read waits at most this per connector
TRACKING_PAUSE_MAX_S = 1.5
CONTROL_SIG_KIND = "control"


class Runtime:
    def __init__(self, connectors: dict, pack: Pack, *, clock: Clock | None = None, proximity=None,
                 interlocks: Interlocks | None = None, rate_hz: float = 30.0, transport: str = "local",
                 armed_local: bool = False, armed_server: bool = False, trust_unknown_plan_basis: bool = False,
                 allow_unsigned_segments: bool = False, calibration: str | None = None, max_events: int = 8192,
                 root_keys: dict | None = None, refused_kids=()):
        missing = [name for name in connectors if name not in pack.arms]
        if missing:
            raise Refused("unknown_arm", f"pack {pack.id} has no entry for {', '.join(missing)}")
        self.clock = clock or Clock()
        self.connectors = dict(connectors)
        self.descriptors = {name: c.descriptor for name, c in connectors.items()}
        for name, desc in self.descriptors.items():
            if desc.semantics != pack.arms[name].semantics:
                raise Refused("semantics_mismatch", f"{name}: connector {desc.semantics} vs pack {pack.arms[name].semantics}")
        self.packs = PackStore(pack)
        self.root_keys = dict(root_keys or {})          # pinned at enrollment: the only keys that may sign a pack
        self.refused_kids = frozenset(refused_kids)     # never trusted here (live rt refuses the public DEV keys)
        self.proximity = proximity if proximity is not None else CapsuleProximity(pack)
        self.interlocks = interlocks if interlocks is not None else Interlocks(None, connectors)
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz
        self.stop_latch = StopLatch(self.clock)
        link = pack.link
        self.link = LinkWatchdog(transport=transport, late_after_s=float(link["late_after_s"].get(transport, 0.5)),
                                 lost_after_s=float(link["lost_after_s"].get(transport, 1.0)))
        self.health = HealthMonitor({n: pack.arms[n] for n in connectors},
                                    control_hz={n: d.control_hz for n, d in self.descriptors.items()},
                                    max_age_s=float(pack.proximity["max_sample_age_s"]))
        self.histories = {name: History() for name in connectors}
        self.samples: dict = {name: None for name in connectors}
        self.grants: dict = {name: None for name in connectors}
        self.admitted = set(connectors)
        self.playback: Playback | None = None
        self.armed_local = armed_local
        self.armed_server = armed_server
        self.trust_unknown_plan_basis = trust_unknown_plan_basis
        self.allow_unsigned_segments = allow_unsigned_segments
        self.calibration = {"id": calibration, "suspect": False}
        self.last_seq = 0                                # downlink seq: segments and signed controls ("seq per direction")
        self.generations: dict = {}
        self.seen: OrderedDict = OrderedDict()
        self.finished: OrderedDict = OrderedDict()
        self.observations: OrderedDict = OrderedDict()
        self.scene_mark_t = -math.inf
        self.scene_mark_why = None
        self.events: deque = deque(maxlen=max_events)
        self.event_seq = 0
        self.events_overflowed = 0
        self.next_tick_t: float | None = None
        self.tick_count = 0
        self._lock = threading.RLock()
        self._gc_was_enabled: bool | None = None
        self._clamp_kinds: set = set()
        self._fresh_event_sent = False

    # ------------------------------------------------------------------ helpers
    @property
    def pack(self) -> Pack:
        return self.packs.active

    @property
    def trusted_keys(self) -> dict:
        """The active pack's segment/control keys still in date, minus the kids refused on this device."""
        return signing.valid_keys(self.pack.trusted_keys, self.pack.key_not_after, exclude=self.refused_kids)

    @property
    def armed(self) -> bool:
        return self.armed_local and self.armed_server and not self.health.faults()

    def emit(self, event: Event) -> None:
        if len(self.events) == self.events.maxlen:
            self.events_overflowed += 1
        self.event_seq += 1
        self.events.append((self.event_seq, event))

    def drain_events(self, after_seq: int = 0) -> list[dict]:
        return [e.public(seq) for seq, e in list(self.events) if seq > after_seq]

    def scene_change(self, why: str, now: float | None = None) -> None:
        now = self.clock.now() if now is None else now
        self.scene_mark_t = now
        self.scene_mark_why = why
        self.emit(Event("scene_change_mark", now, {"why": why}))

    def register_observation(self, obs_id: str, t: float) -> None:
        """An observation this SDK captured (roborun-link reports it). Its time is clamped to now (a future stamp
        would outlive the next scene-change mark) and the first registration of an id wins (re-registering an old
        id never makes it newer than a Stop)."""
        if not isinstance(obs_id, str) or not obs_id or len(obs_id) > 200:
            raise Refused("malformed", "observation ids are short strings")
        if isinstance(t, bool) or not isinstance(t, (int, float)) or not math.isfinite(t):
            raise Refused("malformed", "observation time must be a finite number on the rig clock")
        with self._lock:
            t = min(float(t), self.clock.now())
            if obs_id in self.observations:
                t = min(t, self.observations[obs_id])
            self.observations[obs_id] = t
            while len(self.observations) > 256:
                self.observations.popitem(last=False)

    def at_rest_all(self) -> bool:
        """No segment running and no grant open (an ended playback stays referenced for its report)."""
        return not self._executing()

    def _executing(self) -> bool:
        """rt commands an arm: a segment is running or a grant is open."""
        pb = self.playback
        return (pb is not None and pb.state != "ended") or any(g is not None for g in self.grants.values())

    def _age(self, s) -> float:
        return sample_age(self.clock.now(), s.t, s.clock_uncertainty_s)

    # ------------------------------------------------------------------ K1 Stop
    def stop(self, source: str, *, stop_id: str | None = None, verified: bool = True, user_role: str | None = None,
             verify=None) -> dict:
        report = self.stop_latch.trigger(source, connectors=self.connectors, stop_id=stop_id, verified=verified,
                                         cancel=self._cancel_all, emit=self.emit, user_role=user_role, verify=verify)
        return report

    def _cancel_all(self, why: str) -> dict:
        with self._lock:
            now = self.clock.now()
            ended = None
            if self.playback is not None and self.playback.state != "ended":
                ended = self.playback.seg.id
                self._finish(why, now, hold=False)
            self.scene_change(why, now)
            return {"segment": ended}

    def clear_stop(self, confirm: OperatorConfirm, *, role: str = "operator") -> dict:
        rec = self.stop_latch.clear(confirm, role=role)
        self.scene_change("stop_cleared")
        self.emit(Event("stop_cleared", self.clock.now(), rec))
        return rec

    def hold_all(self, reason: str) -> dict:
        """Hold every owner without latching Stop. Kernel-initiated callers send it only while rt commands an arm."""
        return self.stop_latch.hold_all(self.connectors, reason)

    # ------------------------------------------------------------------ link
    def link_heartbeat(self, now: float | None = None) -> None:
        self.link.message(self.clock.now() if now is None else now)

    def session_started(self, session: str, *, transport: str | None = None, link_lost_ms: float | None = None,
                        admitted: dict | None = None, armed_server: bool | None = None) -> list[dict]:
        with self._lock:
            now = self.clock.now()
            pack_link = self.pack.link
            transport = transport or self.link.transport
            lost = float(pack_link["lost_after_s"].get(transport, 1.0))
            late = float(pack_link["late_after_s"].get(transport, 0.5))
            if link_lost_ms is not None:
                lost = min(lost, float(link_lost_ms) / 1000.0)       # the API may only tighten it
                late = min(late, lost / 2)
            self.link.configure(transport=transport, late_after_s=min(late, lost), lost_after_s=lost)
            previous = self.link.session
            events = self.link.session_started(session, now)
            for e in events:
                self.emit(e)
            if previous is not None or events:
                self.scene_change("reconnect", now)          # plans made before the reconnect are refused
            if admitted is not None:
                self.admitted = {arm for arm, v in admitted.items() if v and arm in self.connectors}
            if armed_server is not None:
                self.armed_server = bool(armed_server)
            return [e.public(0) for e in events]

    def _on_link_lost(self, now: float) -> None:
        pb = self.playback
        if pb is not None and pb.state != "ended":
            # 'brake' (grants above 0.10 m/s) is not built: grants here never exceed 0.10 m/s on WSS, so hold.
            self._finish("link_lost", now, hold=True)
        for arm in self.grants:
            self.grants[arm] = None
        self.scene_change("link_loss", now)
        self.link.mark_held()

    # ------------------------------------------------------------------ packs
    def _pack_changed(self, now: float) -> None:
        """A pack was activated: K6 thresholds and liveness, the capsule checker and the link thresholds follow it
        (link thresholds only tighten mid-session; a new session reads them from the pack again)."""
        pack = self.pack
        self.health.set_packs({n: pack.arms[n] for n in self.connectors},
                              max_age_s=float(pack.proximity["max_sample_age_s"]))
        if isinstance(self.proximity, CapsuleProximity):
            self.proximity = CapsuleProximity(pack)
        t = self.link.transport
        lost = min(self.link.lost_after_s, float(pack.link["lost_after_s"].get(t, self.link.lost_after_s)))
        late = min(self.link.late_after_s, float(pack.link["late_after_s"].get(t, self.link.late_after_s)), lost)
        self.link.configure(transport=t, late_after_s=late, lost_after_s=lost)

    # ------------------------------------------------------------------ segments
    def _ack(self, ref, result, now, **extra) -> dict:
        ack = {"type": "ack", "ref": ref, "result": result, "sdk_t": round(now, 6)}
        ack.update(extra)
        return ack

    def _fresh_read(self) -> None:
        """One synchronous read per connector that can (``refresh``), OUTSIDE the lock: the tick loop never waits on a
        GET, and acceptance judges the start on a sample taken now, not on whatever the poller cached up to ~67 ms
        ago (a 30 Hz poller on a 30 Hz stamp refused up to 45 % of starts as stale)."""
        for c in list(self.connectors.values()):
            refresh = getattr(c, "refresh", None)
            if callable(refresh):
                try:
                    refresh(REFRESH_TIMEOUT_S)
                except Exception:
                    pass                                   # the cached sample stands; acceptance judges its age

    def submit(self, msg: dict, *, received_t: float | None = None) -> dict:
        if isinstance(msg, dict) and not (isinstance(msg.get("id"), str) and msg["id"] in self.seen):
            self._fresh_read()
        with self._lock:
            now = self.clock.now()
            if isinstance(received_t, bool) or not isinstance(received_t, (int, float)) or not math.isfinite(received_t):
                received_t = now
            received_t = min(float(received_t), now)        # the link's receipt time never extends expiry
            sid = msg.get("id") if isinstance(msg, dict) else None
            if isinstance(sid, str) and sid in self.seen:
                return dict(self.seen[sid], duplicate=True)        # re-acked, never re-run
            self._refresh_samples(now)                    # judge the start on the freshest cached samples
            ok, why = signing.verify(msg, SIG_KIND, self.trusted_keys) if isinstance(msg, dict) else (False, "malformed")
            sig_ok = ok or (self.allow_unsigned_segments and why == "unsigned")
            try:
                seg = parse_segment(msg)
            except Refused as r:
                ack = self._ack(sid, "refused", now, **r.as_dict())
                if isinstance(sid, str):
                    self._remember(sid, ack)
                return ack
            ctx = acceptance.AcceptanceContext(
                now=now, received_t=received_t, pack=self.pack, samples=dict(self.samples), histories=self.histories,
                descriptors={a: d for a, d in self.descriptors.items() if a in self.admitted},
                proximity=self.proximity, interlocks=self.interlocks, health=self.health,
                stopped=self.stop_latch.latched, link_accepting=self.link.accepting,
                link_latched_lost=self.link.latched_lost, armed=self.armed, last_seq=self.last_seq,
                generations=self.generations, calibration=self.calibration, scene_mark_t=self.scene_mark_t,
                observations=self.observations, trust_unknown_plan_basis=self.trust_unknown_plan_basis,
                grants={a: g for a, g in self.grants.items() if g is not None}, finished=self.finished,
                signature_ok=sig_ok, transport=self.link.transport)
            if seg.seq > self.last_seq and msg.get("wire") == 1 and sig_ok:
                self.last_seq = seg.seq
            try:
                facts = acceptance.check(seg, ctx)
            except Refused as r:
                if r.reason == "stale_seq":
                    return self._ack(seg.id, "refused", now, **r.as_dict())    # an old replay is not remembered
                ack = self._ack(seg.id, "refused", now, **r.as_dict())
                self._remember(seg.id, ack)
                return ack
            self.generations[seg.task] = max(self.generations.get(seg.task, -1), seg.generation)
            arm = seg.arm
            ap = self.pack.arm(arm)
            jaw = facts.pop("jaw")
            cap = k3.ee_cap(ap, seg.limits, None, self.link.transport, loaded=jaw["held"])
            grant = Grant(arm, seg.id, seg.generation, self.pack.digest, now, cap)
            try:
                self.connectors[arm].acquire(grant)
            except NotWired as exc:
                ack = self._ack(seg.id, "refused", now, reason="connector_not_wired", message=str(exc)[:300],
                                acceptance="passed", **facts)
                self._remember(seg.id, ack)
                return ack
            except ConnectorError as exc:
                ack = self._ack(seg.id, "refused", now, reason="connector_refused", message=str(exc)[:300],
                                acceptance="passed", **facts)
                self._remember(seg.id, ack)
                return ack
            self.grants[arm] = grant
            s = self.samples[arm]
            tol = float(seg.complete["tol_rad"])
            arrival = np.full(ap.n, tol) if ap.arrival_rad is None else np.maximum(tol, ap.arrival_rad)
            self.playback = Playback(seg, start_q7=tuple(s.q) + (jaw["reference"],), n_joints=ap.n, now=now,
                                     rate_hz=self.rate_hz, start_jaw=s.jaw, jaw_ref=jaw["reference"],
                                     jaw_allow=jaw["allowance"], arrival_tol=arrival,
                                     start_signals=k5.signals(s, None, arm_pack=ap))
            self._clamp_kinds = set()
            self._fresh_event_sent = False
            if self._gc_was_enabled is None:
                self._gc_was_enabled = gc.isenabled()
                gc.disable()                                   # no automatic collection while a segment runs
            ack = self._ack(seg.id, "accepted", now, tau0=0.0, **facts)
            self._remember(seg.id, ack)
            self.emit(Event("segment_started", now, {"segment": seg.id, "arm": arm, "label": seg.label,
                                                      "format": seg.format, "rows": int(len(seg.rows))}))
            return ack

    def _remember(self, sid: str, ack: dict) -> None:
        self.seen[sid] = ack
        while len(self.seen) > 512:
            self.seen.popitem(last=False)

    # ------------------------------------------------------------------ tick
    def tick(self) -> None:
        now = self.clock.now()
        if self.next_tick_t is not None:
            late = now - self.next_tick_t
            if late > LATE_HOLD_S:
                with self._lock:
                    executing = self._executing()
                    if self.playback is not None and self.playback.state != "ended":
                        self._finish("tick_overrun", now, hold=False)
                self.emit(Event("tick_overrun", now, {"late_ms": round(late * 1000, 1), "rig_hold": executing}))
                if executing:                                 # nothing rt commands: no automatic hold is sent
                    self.hold_all("tick_overrun")
            elif late > LATE_EVENT_S:
                self.emit(Event("tick_late", now, {"late_ms": round(late * 1000, 1)}))
        with self._lock:
            self.tick_count += 1
            self._sample_all(now)
            for e in self.link.poll(now):
                self.emit(e)
                if e.kind == "link_lost":
                    self._on_link_lost(now)
            if self.at_rest_all():
                rec = self.packs.maybe_activate_staged(at_rest=True, now=now)
                if rec:
                    self._pack_changed(now)
                    self.emit(Event("pack_activated", now, rec))
            if self.playback is not None and self.playback.state != "ended":
                self._play(now)

    def _refresh_samples(self, now: float) -> None:
        """Newer cached samples only (no health bookkeeping): connectors' sample() never blocks."""
        for name, c in self.connectors.items():
            try:
                s = c.sample()
            except Exception:
                continue
            if s is None or stamp_ok(now, s.t, s.clock_uncertainty_s) is not None:
                continue
            prev = self.samples.get(name)
            if prev is None or s.t > prev.t:
                self.samples[name] = s
                self.histories[name].add(s)

    def _sample_all(self, now: float) -> None:
        fault_now = []
        for name, c in self.connectors.items():
            try:
                s = c.sample()
            except Exception:
                s = None
            if s is not None and stamp_ok(now, s.t, s.clock_uncertainty_s) is not None:
                s = None                                # K7: an unusable stamp is no sample
            try:
                h = c.health()
            except Exception as exc:
                from .datatypes import Health
                h = Health("offline", f"health_unavailable: {type(exc).__name__}", now)
            for e in self.health.observe(name, s, h, now):
                self.emit(e)
                if e.kind == "arm_offline":
                    fault_now.append(name)
            if s is not None:
                prev = self.samples.get(name)
                if prev is None or s.t >= prev.t:
                    self.samples[name] = s
                self.histories[name].add(s)
        if fault_now:
            # K6: latch, refuse segments, disarm, report; hold the rig while rt commands an arm
            executing = self._executing()
            self.armed_local = False
            if self.playback is not None and self.playback.state != "ended":
                self._finish("fault", now, hold=False)
            self.scene_change("fault", now)
            if executing:
                self.hold_all("fault:" + ",".join(fault_now))

    def _play(self, now: float) -> None:
        pb = self.playback
        seg = pb.seg
        arm = seg.arm
        c = self.connectors[arm]
        ap = self.pack.arm(arm)
        if self.stop_latch.latched:
            self._finish("stopped", now, hold=False)
            return
        if not self.armed:
            self._finish("disarmed", now, hold=True)
            return
        if pb.state == "gate" and pb.settled:
            self._gate(now)
            return
        if pb.state == "paused":
            if pb.pause_until is not None and now < pb.pause_until:
                return
            then = pb.pause_trigger.then if pb.pause_trigger is not None else "end"
            if then == "continue":
                pb.state, pb.pause_until, pb.pause_trigger = "running", None, None
            else:
                self._finish(f"trigger:{pb.pause_trigger.id if pb.pause_trigger else 'pause'}", now, hold=True)
                return
        s = self.samples.get(arm)
        max_age = float(self.pack.proximity["max_sample_age_s"])
        stale = [n for n, smp in self.samples.items() if smp is None or self._age(smp) > max_age]
        if stale:
            self._fresh_hold(now, pb, {"stale": stale})
            return
        h = self.health.arms[arm]
        if h.thermal == "stop_work":
            self._finish("thermal", now, hold=True)
            return
        kind, detail = k5.builtin(ap, s)
        if kind == "force_cap":
            self.emit(Event("reflex", now, {"segment": seg.id, "kind": kind, **detail}))
            self._finish("force_cap", now, hold=True)
            return
        if kind == "tracking_pause":
            pb.tracking_pause_since = pb.tracking_pause_since or now
            if now - pb.tracking_pause_since > TRACKING_PAUSE_MAX_S:
                self._finish("tracking_pause_timeout", now, hold=True)
            return                                               # tau frozen; the owner keeps the last target
        pb.tracking_pause_since = None
        if seg.triggers:
            lp = pb.last_proximity
            sigs = k5.signals(s, pb.last_target, arm_pack=ap, min_gap_m=None if lp is None else lp.min_gap_m)
            trig = k5.first_match(seg.triggers, sigs, self.histories[arm])
            if trig is not None:
                values = {k: (round(v, 5) if isinstance(v, float) else v) for k, v in sigs.items() if v is not None}
                self.emit(Event("trigger", now, {"segment": seg.id, "tau": round(pb.tau, 4), "id": trig.id,
                                                 "values": values, "action": trig.do if trig.do else trig.then}))
                if isinstance(trig.do, dict) and "pause_s" in trig.do:
                    pb.state, pb.pause_until, pb.pause_trigger = "paused", now + float(trig.do["pause_s"]), trig
                    return
                if trig.do == "hold" or trig.then in ("end", "hold"):
                    self._finish(f"trigger:{trig.id}", now, hold=True)
                    return
        settling = pb.state == "gate"           # rows done; the command has not reached the last row yet
        rest = self.pack.rest                   # the sensor-noise band: rest window, joint range, jaw range
        if settling and now - (pb.gate_opened if pb.gate_opened is not None else now) > float(seg.complete["timeout_s"]):
            n = ap.n
            if (pb.last_target is not None
                    and float(np.max(np.abs(pb.last_target[:n] - seg.rows[-1][:n]))) <= float(rest["range_rad"]) + 1e-12):
                self._settled(now, pb)                 # the joints are there; a slow jaw is not chased past the budget
                return
            self._finish("not_arrived", now, hold=True)
            return
        if settling:
            target, tau = pb.end_target(), pb.tau      # keep commanding the last row through K3/K4
        else:
            target, tau = pb.proposal(measured_q=s.q)
        # the first step is taken from the measured joints and from the jaw reference (the owner's present command)
        last = pb.last_target if pb.last_target is not None else np.r_[np.asarray(s.q, float), pb.jaw_ref]
        grant = self.grants.get(arm)
        chk = k3.check_target(ap, target, last7=last, measured=s, dt=pb.dt, seg_limits=seg.limits,
                              grant_cap=None if grant is None else grant.speed_cap_m_s, transport=self.link.transport,
                              loaded=self.interlocks.held(arm) is not None, jaw_ref=pb.jaw_ref, jaw_allow=pb.jaw_allow)
        if chk.refused:
            self.emit(Event("limit_refused", now, {"segment": seg.id, "reason": chk.refused, **chk.detail}))
            self._finish("limit:" + chk.refused, now, hold=True)
            return
        if chk.clamps:
            new = set(chk.clamps) - self._clamp_kinds
            if new:
                self._clamp_kinds |= new
                self.emit(Event("clamp", now, {"segment": seg.id, "tau": round(tau, 4), "kinds": sorted(new),
                                               "planner_bug": True, **chk.detail}))
        if settling and pb.last_target is not None and float(np.max(np.abs(chk.target - pb.last_target))) <= 1e-9:
            self._settled(now, pb)                     # a bound holds the command where it is: nothing left to write
            return
        if settling and pb.settle_idle_s(chk.target, now, joint_eps=float(rest["range_rad"]),
                                         jaw_eps=float(rest["jaw_range"]),
                                         moving=chk.scale < 1.0 - 1e-9) >= float(rest["window_s"]) - 1e-9:
            # a bound holds the command where it is up to sensor noise (K3's jaw lead bound follows a jittering jaw
            # reading on a blocked close): no closer by more than the rest band for a whole rest window -> settled
            self._settled(now, pb)
            return
        prox = self.proximity.check(arm, s.q, chk.target[:ap.n], self.samples, now)
        if not prox.ok:
            if prox.reason == "stale":
                self._fresh_hold(now, pb, prox.public())
                return
            self.emit(Event("proximity_hold", now, {"segment": seg.id, "tau": round(tau, 4), **prox.public()}))
            self.scene_change("proximity_hold", now)
            self._finish("proximity", now, hold=True)
            return
        if self.stop_latch.latched:
            return
        try:
            c.write(chk.target, now)
        except ConnectorError as exc:
            self.emit(Event("connector_error", now, {"arm": arm, "error": str(exc)[:200]}))
            self._finish("connector_error", now, hold=True)
            return
        if self.stop_latch.latched:
            self.stop_latch.hold_all({arm: c}, "stop_after_write")   # write-then-recheck: a racing Stop wins
            return
        pb.commit(chk.target, tau, clamped=bool(chk.clamps), scale=1.0 if settling else chk.scale)
        pb.last_proximity = prox
        if settling:
            if pb.command_at_end():
                self._settled(now, pb)
        elif pb.rows_done():
            pb.state, pb.gate_opened = "gate", now
            if pb.command_at_end():
                self._settled(now, pb)

    def _settled(self, now: float, pb: Playback) -> None:
        """The command holds the last row (or K3 lets it get no closer): the end gate's stillness window starts."""
        pb.settled, pb.gate_since = True, now

    def _fresh_hold(self, now: float, pb: Playback, detail: dict) -> None:
        pb.fresh_hold_used_s += self.dt
        if not self._fresh_event_sent:
            self._fresh_event_sent = True
            self.emit(Event("fresh_pose_hold", now, {"segment": pb.seg.id, "tau": round(pb.tau, 4), **detail}))
        if pb.fresh_hold_used_s > float(self.pack.proximity["fresh_hold_budget_s"]) + 1e-9:
            self._finish("fresh_pose_budget_exhausted", now, hold=True)

    def _verify_require(self, pb: Playback, now: float) -> tuple[bool, dict]:
        """A verify gate's requirement on locally computed signals (segment._check_require admitted its shape)."""
        seg = pb.seg
        req = seg.end_gate["require"]
        s = self.samples.get(seg.arm)
        if s is None:
            return False, {"sig": req["sig"], "value": None, "why": "no_sample"}
        lp = pb.last_proximity
        sigs = k5.signals(s, pb.last_target, arm_pack=self.pack.arm(seg.arm),
                          min_gap_m=None if lp is None else lp.min_gap_m)
        value = sigs.get(req["sig"])
        if req.get("stat", "value") == "delta_since_start":
            start = pb.start_signals.get(req["sig"])
            value = None if value is None or start is None else value - start
        ok = value is not None and math.isfinite(value) and k5.OPS[req["op"]](value, float(req["v"]))
        return ok, {"sig": req["sig"], "stat": req.get("stat", "value"), "op": req["op"], "v": req["v"],
                    "value": None if value is None else round(float(value), 5), "passed": ok}

    def _gate(self, now: float) -> None:
        """End gate (the command already holds the last row: ``_play``'s settle phase wrote it through K3/K4 within
        complete.timeout_s of the rows ending, else ``not_arrived``): hold it until the arm has been still
        (complete.tol_rad over complete.still_s) for a full window after the command settled. Then:
        - verify: its requirement is evaluated; failed -> ``verify_failed`` (held)
        - every gate: ``complete`` only on arrival (each joint within max(complete.tol_rad, the pack's arrival_rad)
          of the last row); still but short of it until complete.timeout_s -> ``not_arrived`` (held)
        Never still within complete.timeout_s -> ``unsettled`` (held)."""
        pb = self.playback
        seg = pb.seg
        still_s = float(seg.complete["still_s"])
        rows = self.histories[seg.arm].window(still_s)
        still = False
        if rows and rows[-1][0] >= pb.gate_since + still_s - 1e-9:
            q = np.stack([r[1] for r in rows])
            still = float(np.max(np.ptp(q, axis=0))) <= float(seg.complete["tol_rad"]) + 1e-12
        if still:
            n = len(rows[-1][1])
            err = np.abs(rows[-1][1] - seg.rows[-1][:n])
            tol = pb.arrival_tol if pb.arrival_tol is not None else np.full(n, float(seg.complete["tol_rad"]))
            arrived = bool(np.all(err <= tol[:n] + 1e-12))
            pb.gate_evidence = {"arrived": arrived, "arrival_tol_rad": [round(float(v), 5) for v in tol[:n]]}
            if seg.end_gate.get("kind") == "verify":
                ok, req = self._verify_require(pb, now)
                pb.gate_evidence["require"] = req
                if not ok:
                    self._finish("verify_failed", now, hold=True)
                    return
            if arrived:
                self._finish("complete", now, hold=False)
                return
        if now - pb.gate_since > float(seg.complete["timeout_s"]):
            self._finish("not_arrived" if still else "unsettled", now, hold=True)

    def _finish(self, outcome: str, now: float, *, hold: bool) -> None:
        pb = self.playback
        if pb is None or pb.state == "ended":
            return
        seg = pb.seg
        arm = seg.arm
        pb.end(outcome)
        c = self.connectors[arm]
        acks = None
        if hold:
            acks = self.stop_latch.hold_all({arm: c}, f"segment_{outcome}")
        try:
            c.release()
        except Exception as exc:
            self.emit(Event("connector_error", now, {"arm": arm, "error": f"release: {exc}"[:200]}))
        self.grants[arm] = None
        s = self.samples.get(arm)
        end_row = seg.rows[-1]
        evidence = {"writes": pb.writes, "clamps": pb.clamps, "tau": round(pb.tau, 4)}
        if s is not None:
            evidence["arrival_err_rad"] = round(float(np.max(np.abs(np.asarray(s.q) - end_row[:len(s.q)]))), 5)
        evidence.update(pb.gate_evidence)
        last_prox = pb.last_proximity
        if last_prox is not None and last_prox.min_gap_m is not None:
            evidence["min_gap_mm"] = round(last_prox.min_gap_m * 1000, 1)
        self.finished[seg.id] = {"outcome": outcome, "t": now,
                                 "end_q": None if s is None else list(s.q), "end_jaw": None if s is None else s.jaw}
        while len(self.finished) > 64:
            self.finished.popitem(last=False)
        report = {"segment": seg.id, "gate": seg.end_gate.get("kind", "hold"), "outcome": outcome,
                  "evidence": evidence,
                  "rest": {arm: {"q": None if s is None else [round(v, 4) for v in s.q], "jaw": None if s is None else s.jaw}},
                  "capture": seg.end_gate.get("capture", []) if outcome == "complete" else [],
                  "observations": []}
        if acks is not None:
            report["hold"] = acks
        self.emit(Event("gate_report", now, report))
        if self._gc_was_enabled is not None:
            if self._gc_was_enabled:
                gc.enable()
            self._gc_was_enabled = None

    # ------------------------------------------------------------------ controls
    def control(self, msg: dict, *, confirm: OperatorConfirm | None = None) -> dict:
        if not isinstance(msg, dict):
            raise Refused("malformed", "control messages are objects")
        op = msg.get("op")
        now = self.clock.now()
        if op == "hold":                             # only ever tightens: obeyed unsigned, any seq
            arms = msg.get("arm")
            if arms not in (None, "all") and arms not in self.connectors:
                raise Refused("unknown_arm", str(arms)[:64])
            with self._lock:
                if self.playback is not None and self.playback.state != "ended":
                    self._finish("hold", now, hold=False)
            targets = self.connectors if arms in (None, "all") else {arms: self.connectors[arms]}
            return {"op": op, "arms": self.stop_latch.hold_all(targets, "control_hold")}
        ok, why = signing.verify(msg, CONTROL_SIG_KIND, self.trusted_keys)
        if op == "clear_stop":
            if msg.get("user_role") not in ("operator", "owner") or confirm is None:
                raise Refused("operator_confirmation_required", "a hosted clear needs the operator role and a local confirmation")
        if not ok:
            raise Refused("bad_signature", f"control {op!r} must be signed ({why})")
        with self._lock:
            seq = msg.get("seq")
            if isinstance(seq, bool) or not isinstance(seq, int) or seq <= self.last_seq:
                raise Refused("stale_seq", f"control seq {seq!r} is not above {self.last_seq}: replays are refused")
            self.last_seq = seq
            if op == "clear_stop":
                return self.clear_stop(confirm, role=msg["user_role"])
            if op == "abort_task":
                if self.playback is not None and self.playback.state != "ended" and self.playback.seg.task == msg.get("task"):
                    self._finish("aborted", now, hold=True)
                return {"op": op}
            if op == "set_interlock":
                arm = msg.get("arm")
                if arm not in self.connectors:
                    raise Refused("unknown_arm", str(arm)[:64])
                out = {}
                if isinstance(msg.get("pending_release"), str):
                    out["pending_release"] = self.interlocks.set_pending_release(
                        arm, evidence=str(msg.get("evidence", "")), source="api")
                if msg.get("held") is True:
                    s = self.samples.get(arm)
                    # the jaw at grasp: the most-closed of measured and commanded (the squeeze), never the width alone
                    jaw = None if s is None or s.jaw is None else acceptance.jaw_reference(self.pack.arm(arm), s, None)
                    out["held"] = self.interlocks.set_held(arm, jaw=jaw, tool_z_m=None,
                                                           evidence=str(msg.get("evidence", "")), source="api")
                elif msg.get("held") is False:
                    out["cleared"] = self.interlocks.clear(arm, reason="api_confirmed")
                if not out:
                    raise Refused("malformed", "set_interlock needs held or pending_release")
                self.emit(Event("interlock", now, {"arm": arm, "evidence": str(msg.get("evidence", ""))[:200],
                                                   "bits": self.interlocks.public()[arm]}))
                return out
            if op == "pack_notice":
                # roborun-link fetches GET /v1/packs/{digest} into its PackCache and calls offer_pack(); here we only
                # say whether it is the active one
                return {"op": op, "pack": msg.get("pack"), "active": msg.get("pack") == self.pack.digest}
            if op == "goaway":
                # an orderly link loss: the running segment holds, grants are revoked, the latch waits for a new session
                for e in self.link.goaway(now):
                    self.emit(e)
                self._on_link_lost(now)
                return {"op": op}
        raise Refused("unsupported_control", f"control {op!r} unsupported")

    def offer_pack(self, raw: dict, *, confirm: OperatorConfirm | None = None) -> dict:
        """A new signed pack (K2 admission): tighter packs activate at rest; anything else is staged
        until a local operator confirms it at rest."""
        from .pack import load_pack
        root = {kid: pub for kid, pub in self.root_keys.items() if kid not in self.refused_kids}
        if not root:
            raise Refused("no_root_keys", "no pack root key is pinned on this device")
        new = load_pack(raw, root)
        missing = [name for name in self.connectors if name not in new.arms]
        if missing:
            raise Refused("unknown_arm", f"pack {new.id} has no entry for {', '.join(missing)}")
        with self._lock:
            now = self.clock.now()
            if confirm is not None and self.packs.staged is not None and self.packs.staged.digest == new.digest:
                rec = self.packs.confirm(new.digest, confirm, at_rest=self.at_rest_all(), now=now)
            else:
                rec = self.packs.offer(new, at_rest=self.at_rest_all(), now=now)
            kind = "pack_activated" if rec.get("result") == "activated" else "pack_staged"
            if rec.get("result") == "activated":
                self._pack_changed(now)
            if rec.get("result") != "current":
                self.emit(Event(kind, now, {k: v for k, v in rec.items() if k != "t"}))
            return rec

    def decide(self, msg: dict) -> dict:
        with self._lock:
            pb = self.playback
            if not isinstance(msg, dict) or pb is None or pb.state != "paused" or msg.get("segment") != pb.seg.id:
                raise Refused("no_pause", "no matching pause window is open")
            action = msg.get("action")
            if action == "continue":
                pb.state, pb.pause_until, pb.pause_trigger = "running", None, None
            else:                                       # end, back_off (not built) -> end and hold
                self._finish(f"decide:{action}", self.clock.now(), hold=True)
            return {"segment": pb.seg.id, "action": action, "state": pb.state}

    # ------------------------------------------------------------------ state
    def state(self) -> dict:
        now = self.clock.now()
        pb = self.playback
        arms = {}
        for name, s in self.samples.items():
            row = {"status": "offline"} if s is None else s.public(now)
            row["health"] = self.health.public()[name]
            row["interlocks"] = self.interlocks.public()[name]
            row["grant"] = self.grants[name] is not None
            arms[name] = row
        guards = {}
        if pb is not None and pb.last_proximity is not None:
            lp = pb.last_proximity
            guards = {"min_gap_mm": None if lp.min_gap_m is None else round(lp.min_gap_m * 1000, 1),
                      "required_gap_mm": None if lp.required_m is None else round(lp.required_m * 1000, 1)}
        running = pb is not None and pb.state != "ended"
        mode = "PAUSED" if running and pb.state == "paused" else "EXECUTING" if running else "IDLE"
        if self.link.state == "HELD":
            mode = "HOLDING"
        if self.health.faults():
            mode = "FAULT"
        if self.stop_latch.latched:
            mode = "STOPPED"
        flags = []
        if running and pb.fresh_hold_used_s > 0:
            flags.append("fresh_pose_hold")
        if running and pb.clamps:
            flags.append("clamp")
        if running and pb.state == "paused":
            flags.append("pause")
        if running and pb.tracking_pause_since is not None:
            flags.append("bias")
        if any(h.thermal != "ok" for h in self.health.arms.values()):
            flags.append("thermal")
        guards["flags"] = flags
        return {"type": "state", "sdk_t": round(now, 6),
                "kernel": {"mode": mode, "link": self.link.public(now), "stopped": self.stop_latch.latched,
                           "faults": self.health.faults(), "armed": self.armed,
                           "armed_halves": {"local": self.armed_local, "server": self.armed_server}},
                "exec": None if pb is None else pb.public(), "arms": arms, "guards": guards,
                "pack": self.pack.digest, "scene_mark": None if not math.isfinite(self.scene_mark_t) else
                {"t": round(self.scene_mark_t, 6), "why": self.scene_mark_why}}

    # ------------------------------------------------------------------ real-time loop
    def run(self, stop_event: threading.Event, *, feed_local_link: bool = False) -> None:
        """Tick until ``stop_event`` is set. Never catches up with bursts: a missed tick is skipped."""
        next_t = self.clock.now()
        while not stop_event.is_set():
            self.next_tick_t = next_t
            if feed_local_link:
                self.link_heartbeat()
            self.tick()
            next_t += self.dt
            delay = next_t - self.clock.now()
            if delay < 0:
                next_t = self.clock.now()
            else:
                self.clock.sleep(delay)
