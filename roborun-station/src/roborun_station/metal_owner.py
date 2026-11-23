"""A fake MakerMods Metal owner (:8093) with the real Metal owner's semantics.

- POST /stop: no token, any state; bumps the control epoch, so it preempts the owner's own Home; holds, never
  disables (the right reference for every connector's hold()).
- POST /home moves at 5 deg/s; POST /recover_hold resyncs a fault hold (needs X-Manifest-Metal-Control: 1 over HTTP).
- motion: GET /lease/observe, POST /lease/action (an observation <= 60 s old), /lease/heartbeat (0.8 s), /lease/stop (lease id);
  answers are pulled from a predict callable / URL each 30 Hz tick, bounded to 5 deg and 0.081 jaw per answer.
- Faults: a USB drop marks transport_lost (fault_hold, no samples); an adapter in DFU or with the wrong identity is
  refused ("can0 is not the confirmed right Metal candleLight adapter"); `crash_on_double_fault=True` makes the
  owner process die on a second fault and stop answering.

Parameterised: `FakeMetalOwner(arm=, port=, channel=, serial=)` is any Metal of any rig -- the right Metal
of yam_metal (the defaults: right, :8093, can0, the simulated adapter serial), or either Metal of a dual-Metal pair -- and every reply, fault
and refusal names that arm, channel and adapter. `physics="live"` behaves like the live Metal (core.metal_spec(live=True)):
5 deg/s, joints 2/3 stalling 0.29-2.1 deg short of their command from sag, joint 5 stopping 0.28 deg short (its arrival
bias), and a jaw that stalls on the cup (`cup_between()` -> True while a cup sits between the pads: the jaw stops on its
wall at METAL_JAW_ON_CUP with the owner's .30 Nm). A live owner's Home arrives at its paired measured pose by commanding
the paired command (positions + the stall), as the yam_metal profile's hardware settings pair them. `physics="ideal"` (default)
is an exact follower, which the SDK drills judge at 1 mrad.
"""
from __future__ import annotations

import json
import math
import threading
import urllib.request
import uuid

from .core import (METAL_JAW_ON_CUP, METAL_SERIAL, Faults, RealClock, SimArm, finite, metal_shortfall_rad,
                   metal_spec)

PERIOD = 1/30
MAX_JOG = math.radians(5)
MOTION_HEARTBEAT_S = 0.8
ANSWER_S = 0.25
JAW_FRACTION_STEP = 0.081
SERIAL = METAL_SERIAL                             # the right Metal's adapter (the default identity)
PORT = 8093


class OwnerError(ValueError):
    pass


class OwnerDead(RuntimeError):
    """The owner process is gone (a crash or kill -9): nothing answers until it is restarted."""


class FakeMetalOwner:
    def __init__(self, *, clock=None, faults: Faults | None = None, predict=None, connected=True,
                 crash_on_double_fault=False, seed=7, arm: str = "right", port: int = PORT, channel: str = "can0",
                 serial: str = SERIAL, physics: str = "ideal", cup_between=None, home=None):
        if arm not in ("left", "right"):
            raise ValueError("a Metal owner drives the left or the right arm")
        if physics not in ("ideal", "live"):
            raise ValueError("physics is 'ideal' or 'live'")
        self.clock = clock or RealClock()
        self.faults = faults or Faults()
        self.arm_name, self.port, self.channel, self.serial, self.physics = arm, int(port), channel, serial, physics
        self.cup_between = cup_between            # callable() -> bool: a cup between the pads (live jaw stalls on it)
        live = physics == "live"
        extra = {} if home is None else {"home": tuple(home)}
        self.arm = SimArm(metal_spec(arm, live=live, serial=serial, channel=channel,
                                     jaw_floor=self._jaw_floor if live else None, **extra),
                          self.clock, self.faults, seed=seed)
        self.predict = predict
        self.connected = connected
        self.fault: str | None = None
        self.transport_lost = False
        self.motion: dict | None = None
        self.epoch = 0
        self.motion_lease: dict | None = None
        self.observations: dict[str, dict] = {}
        self.sessions: dict[str, dict] = {}
        self.crash_on_double_fault = crash_on_double_fault
        self.dead = False
        self.events: list[tuple[float, str, dict]] = []
        self.lock = threading.RLock()

    def note(self, event, **detail):
        self.events.append((self.clock(), event, detail))

    def _jaw_floor(self):
        """Raw jaw position where the pads stop on a cup wall (live physics), or None."""
        if self.cup_between is None or not self.cup_between():
            return None
        lo, hi = self.arm.spec.jaw_closed, self.arm.spec.jaw_open
        return lo + METAL_JAW_ON_CUP*(hi - lo)

    def home_command(self):
        """What Home commands: the paired command whose measured arrival is the Home pose (live: + each joint's stall)."""
        home = list(self.arm.spec.home)
        if self.physics != "live":
            return home
        q = list(self.arm.q)
        return [h + (1 if h >= c else -1)*metal_shortfall_rad(home, i) for i, (h, c) in enumerate(zip(home, q))]

    def _fault_text(self, dfu):
        return (f"{self.channel} is not the confirmed {self.arm_name} Metal candleLight adapter" if dfu
                else "Metal CAN transport lost")

    def _alive(self):
        if self.dead:
            raise OwnerDead("metal owner is not running")

    # ---- time ----------------------------------------------------------------------------------------------------
    def advance(self, seconds: float, *, step: float = PERIOD):
        for _ in range(max(1, int(round(seconds/step)))):
            self.clock.advance(step)
            self.tick()

    def tick(self):
        if self.dead:
            return
        now = self.clock()
        if self.faults.pause_until(now) is not None:
            return
        with self.lock:
            dropped = self.faults.active("usb_drop", now, arm=self.arm_name) is not None
            dfu = self.faults.active("dfu", now, arm=self.arm_name) is not None
            if dropped or dfu:
                if self.fault is not None and self.crash_on_double_fault and self.transport_lost:
                    self.dead = True               # the 02:07 crash in the fault handler
                    self.note("crashed")
                    return
                self.transport_lost = dropped
                self.fault = self._fault_text(dfu)
                if self.motion:
                    self.motion["state"] = "failed"
                    self.motion = None
                if self.motion_lease and self.motion_lease["active"]:
                    self._end_motion("transport_lost")
            self._motion_tick(now)
            self._lease_tick(now)
            self.arm.step(now)

    # ---- status ----------------------------------------------------------------------------------------------------
    def status(self, body=None) -> dict:
        self._alive()
        with self.lock:
            now = self.clock()
            row = None if self.transport_lost else self.arm.row(now)
            state = ("fault_hold" if self.fault else "moving" if self.motion else
                     "holding" if self.connected else "observing" if row else "offline")
            jaw = None
            if row is not None:
                lo, hi = self.arm.spec.jaw_closed, self.arm.spec.jaw_open
                row = dict(row, gripper_motor_rad=row["gripper"], gripper=(row["gripper"]-lo)/(hi-lo),
                           commanded_gripper=(row["commanded_gripper"]-lo)/(hi-lo),
                           commanded_rad=list(self.arm.q_cmd) if self.connected else None)
                jaw = row["gripper"]
            if row is not None and self.physics == "live":
                row["jaw_effort_nm"] = self.arm.jaw_effort
            return {"schema": "manifest-metal-owner/1", "body": "metal", "arm": self.arm_name, "state": state,
                    "control_epoch": self.epoch, "connected": self.connected, "error": self.fault,
                    "control_available": self.connected and not self.fault, "port": self.port,
                    "adapter": {"channel": self.channel, "usb_serial": self.serial}, "transport_lost": self.transport_lost,
                    "physics": self.physics,
                    "measured": {} if row is None else {self.arm_name: row}, "motion": self.motion,
                    "motion_lease": None if self.motion_lease is None else {k: v for k, v in self.motion_lease.items()
                                                                   if not k.startswith("_")},
                    "jaw_fraction": jaw}

    # ---- verbs ---------------------------------------------------------------------------------------------------
    def stop(self, body=None) -> dict:
        """Any state, no token: epoch bump (preempts Home and any motion lease), hold the measured pose."""
        self._alive()
        with self.lock:
            self.epoch += 1
            if self.motion:
                self.motion["state"] = "stopped"
                self.motion = None
            if self.motion_lease and self.motion_lease["active"]:
                self._end_motion("supervisor_stop")
            if self.connected and not self.transport_lost:
                self.arm.hold_here()
            self.note("stop", epoch=self.epoch)
            return self.status()

    def home(self, body=None) -> dict:
        self._alive()
        with self.lock:
            if self.fault:
                raise OwnerError(self.fault)
            if not self.connected or self.motion:
                raise OwnerError("Metal joints must be connected and idle before Home")
            self.motion = {"kind": "home", "state": "moving", "epoch": self.epoch, "id": uuid.uuid4().hex}
            self.arm.command(q=self.home_command())
            self.note("home_start")
            return self.status()

    def recover_hold(self, body=None) -> dict:
        self._alive()
        with self.lock:
            if self.motion or not self.connected:
                raise OwnerError("No idle previously owned motors to recover")
            if self.transport_lost or self.faults.active("usb_drop", self.clock(), arm=self.arm_name):
                raise OwnerError("Metal CAN transport lost")
            self.fault = None
            self.arm.hold_here()
            return self.status()

    def _motion_tick(self, now):
        if self.motion and self.motion["kind"] == "home":
            if self.motion["epoch"] != self.epoch:
                self.motion = None
            elif self.arm.at(self.arm.spec.home, tol=math.radians(.5)):
                self.motion["state"] = "complete"
                self.motion = None
                self.note("home_end")

    # ---- motion ------------------------------------------------------------------------------------------------------
    def register_session(self, session_id=None, *, token=None):
        session_id = session_id or uuid.uuid4().hex
        self.sessions[session_id] = {"id": session_id, "token": token or uuid.uuid4().hex*2}
        return session_id

    def motion_observe(self, body=None) -> dict:
        self._alive()
        with self.lock:
            if self.fault:
                raise OwnerError(self.fault)
            oid = uuid.uuid4().hex
            self.observations[oid] = {"at": self.clock(), "epoch": self.epoch, "q": list(self.arm.q)}
            st = self.status()
            return {"observation_id": oid, "observed_at": self.clock(), "measured": st["measured"],
                    "state": "running" if self.motion_lease and self.motion_lease["active"] else "idle",
                    "healthy": self.connected and not self.fault, "capabilities": {f"mixed_motion_{self.arm_name}_version": 1}}

    def motion_action(self, body: dict) -> dict:
        self._alive()
        with self.lock:
            ref = body.get("reference") or {}
            if ref.get("kind") != "video_motion" or ref.get("session_id") not in self.sessions:
                raise OwnerError("Metal motion accepts one locally prepared video-motion session")
            seconds = body.get("seconds")
            if not finite(seconds) or not 4 <= seconds <= 12:
                raise OwnerError("seconds must be between 4 and 12")
            obs = self.observations.pop(body.get("observation_id"), None)
            if not obs or self.clock() - obs["at"] > 60 or obs["epoch"] != self.epoch:
                raise OwnerError("Observe the current Metal owner before starting motion")
            if not self.connected or self.fault or self.motion:
                raise OwnerError("Metal joints and calibrated jaw must already be connected, holding and healthy")
            now = self.clock()
            self.motion = {"kind": "motion", "state": "moving", "epoch": self.epoch, "id": uuid.uuid4().hex}
            self.motion_lease = {"id": self.motion["id"], "active": True, "reason": None, "started_at": now,
                             "seconds": float(seconds), "_heartbeat": now, "_epoch": self.epoch,
                             "_session": ref["session_id"], "_ticks": 0}
            return {"lease": {k: v for k, v in self.motion_lease.items() if not k.startswith("_")}}

    def motion_heartbeat(self, body: dict) -> dict:
        self._alive()
        with self.lock:
            if not self.motion_lease or body.get("lease_id") != self.motion_lease["id"]:
                raise OwnerError("Unknown Metal motion lease")
            if self.motion_lease["active"]:
                self.motion_lease["_heartbeat"] = self.clock()
            return {"lease": {k: v for k, v in self.motion_lease.items() if not k.startswith("_")}}

    def motion_stop(self, body: dict) -> dict:
        self._alive()
        with self.lock:
            if not self.motion_lease or body.get("lease_id") != self.motion_lease["id"]:
                raise OwnerError("Unknown Metal motion lease")
            if self.motion_lease["active"]:
                self.stop()
            return {"lease": {k: v for k, v in self.motion_lease.items() if not k.startswith("_")}}

    def _end_motion(self, why):
        lease = self.motion_lease
        if lease and lease["active"]:
            lease.update(active=False, reason=why, ended_at=self.clock())
            if self.motion and self.motion.get("kind") == "motion":
                self.motion = None
            self.arm.hold_here()
            self.note("motion_end", reason=why)

    def _lease_tick(self, now):
        lease = self.motion_lease
        if not lease or not lease["active"]:
            return
        if lease["_epoch"] != self.epoch:
            self._end_motion("operator_stop")
        elif now - lease["_heartbeat"] > MOTION_HEARTBEAT_S:
            self._end_motion("client_disconnected")
        elif now - lease["started_at"] >= lease["seconds"]:
            self._end_motion("action_window_finished")
        elif self.predict is not None:
            lo, hi = self.arm.spec.jaw_closed, self.arm.spec.jaw_open
            jaw = (self.arm.jaw_cmd - lo)/(hi - lo)
            session = self.sessions[lease["_session"]]
            chunk = "q_r_chunk" if self.arm_name == "right" else "q_l_chunk"
            body = {"session_id": session["id"], "token": session["token"], "robot_body": "yam_metal",
                    "enabled_arms": [self.arm_name], "observed_at": now,
                    "measured": {self.arm_name: self.status()["measured"].get(self.arm_name)},
                    "commanded": {chunk: list(self.arm.q_cmd) + [jaw]}}
            try:
                answer = self.predict(body) if callable(self.predict) else _post(self.predict, body)
                target = answer["target"]
                if not 0 <= self.clock() - answer["acquired_at"] < ANSWER_S:
                    raise OwnerError("motion prediction expired before Metal acceptance")
                if (len(target) != 7 or not all(finite(v) for v in target) or not 0 <= target[6] <= 1
                        or max(abs(a-b) for a, b in zip(target[:6], self.arm.q_cmd)) > MAX_JOG
                        or abs(target[6]-jaw) > JAW_FRACTION_STEP):
                    raise OwnerError("Metal motion chunk exceeds the bounded joint or jaw increment")
            except Exception as exc:
                lease["stop_reason"] = str(exc)
                self._end_motion("prediction_error")
                return
            self.arm.command(q=target[:6], jaw=lo + target[6]*(hi-lo))

    # ---- HTTP routes ---------------------------------------------------------------------------------------------
    def routes(self):
        get = {"/status": self.status, "/pose": self.status, "/lease/observe": self.motion_observe}
        post = {"/stop": self.stop, "/home": self.home, "/recover_hold": self.recover_hold,
                "/lease/action": self.motion_action, "/lease/heartbeat": self.motion_heartbeat, "/lease/stop": self.motion_stop}
        return get, post


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=0.4) as resp:
        return json.load(resp)
