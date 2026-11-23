"""The YAM station (:8091): the station logic around your arm adapters (supervisor leases on top of the vendor station),
plus simulated owners for tests. Real arms run in a subclass that drives your adapters (roborun_sdk.embodiment).

What it reproduces (each is a behaviour Stop and the connectors must deal with):
- leases: POST /supervisor/action (reference.kind video_motion | mixed_jog), heartbeat within 2.5 s, 4-12 s windows;
- POST /supervisor/stop refuses without the active lease id;
- the vendor POST /stop is refused while HOMING, RESETTING or STOPPING;
- POST /home moves both arms home with the grippers OPEN; POST /reset lets the arms go, reopens, then homes;
- the thermal retreat's fallback blend runs under HOMING (grip retained), where Stop is refused;
- the vendor's automatic Homes open the grippers, but are suppressed under a supervised lease;
- during an motion lease the station pulls answers from a predict callable / URL every other 30 Hz tick and enforces
  0.04 rad and 0.081 jaw per answer and a 0.25 s answer age;
- GET /supervisor/pose: the stamped manifest-native-pose/1 rows.
`lease_free_hold=True` adds the lease-free hold route: POST /supervisor/hold, no
lease id, any state, never refused (an unverified request is obeyed and recorded). It latches (new leases refused for
0.75 s), ends any lease with reason 'supervisor_stop', interrupts the thermal blend and the station Home move, and
cancels the Home that follows a Reset's reopen (the reopen itself cannot be interrupted). It then waits up to wait_s
(0.25 s) for the arms to settle and answers each arm's MEASURED state from its newest sample: 'resetting' (let go), 'fault' (driver dead: USB drop or DFU), 'moving' (something still writes, or a joint
reports > 0.25 rad/s), 'stale' (newest sample > 0.25 s old: frozen or late samples cannot confirm a hold) or 'holding'.
Only 'holding' says holding:true, and positions come from that sample, never from the simulator's ground truth.
Under a SimClock the settle wait advances the clock in driver cycles; under a RealClock it sleeps while the ticker runs.

`right="yam"` makes the station the dual-YAM rig: a REAL second YAM on the right (yam_right_spec, its own
SimArm, never parked) instead of the parked stand-in. Every lease then names its arm (the session's arm for video_motion,
`reference.arm` for mixed_jog and the grip-kept `move_home`), the station pulls answers for that arm only (q_l_chunk /
q_r_chunk, enabled_arms [arm], measured rows of both arms), and Home, Reset, the hold and the pose rows cover both arms. The
default (`right=None` -> `parked_right`) keeps the left-only station, which the SDK drills use.
"""
from __future__ import annotations

import json
import threading
import time
import urllib.request
import uuid

from .core import Faults, RealClock, SimArm, finite, metal_right_spec, yam_left_spec, yam_right_spec

ORDINARY_ENDS = ("action_window_finished", "action_goal_reached")   # the commands ran out; nobody asked for a stop
HOLD_WAIT_S = 0.25                                 # wait this long for the arms to settle
HOLD_WAIT_MAX_S = 2.0
LATCH_S = 0.75                                     # new leases refused this long after a hold
MOVING_RAD_S = 0.25
STALE_S = 0.25
SETTLE_POLL_S = 0.01
HEARTBEAT_TIMEOUT = 2.5
SETUP_POSE_MAX_RAD = 1.2                           # a setup pose (guided setup, overhead camera): the most any joint may travel
SETUP_POSE_SPEED_RAD_S = 0.25                      # its peak joint speed, on a minimum-jerk curve
HAND_GUIDE_MAX_S = 180.0
TICK_HZ = 30.0
REFILL_TICKS = 2                                   # one answer per two 30 Hz ticks (~15 Hz), as the native client
ANSWER_AGE_S = 0.25
JOINT_STEP = 0.04
JAW_STEP = 0.081
HOME_S = 3.0
REOPEN_S = 2.0
THERMAL_STEPS = 30


class StationError(ValueError):
    """A 409 from the station (the message is the station's own wording)."""


class FakeYamStation:
    def __init__(self, *, clock=None, faults: Faults | None = None, lease_free_hold=False, predict=None,
                 parked_right=True, right: str | None = None, left_spec=None, right_spec=None, seed=0):
        self.clock = clock or RealClock()
        self.faults = faults or Faults()
        self.lease_free_hold = lease_free_hold
        self.predict = predict                     # callable(body) -> answer dict, or an http(s) URL string
        self.left = SimArm(left_spec or yam_left_spec(), self.clock, self.faults, seed=seed)
        if right not in (None, "yam", "parked"):
            raise ValueError("right is 'yam' (a real second YAM) or 'parked' (the stand-in)")
        self.dual = right == "yam"
        if self.dual:
            self.right = SimArm(right_spec or yam_right_spec(), self.clock, self.faults, seed=seed+1)  # the second YAM
            parked_right = False
        else:
            self.right = SimArm(metal_right_spec(), self.clock, self.faults, seed=seed+1)   # a parked stand-in
            parked_right = True if right == "parked" else parked_right
        self.parked_right = parked_right
        self.state = "idle"
        self.run = 0
        self.lease: dict | None = None
        self.sessions: dict[str, dict] = {}
        self.observations: dict[str, float] = {}
        self.events: list[tuple[float, str, dict]] = []
        self.lock = threading.RLock()
        self._task = None                          # ("home"|"reset"|"thermal", data)
        self._ticks = 0
        self._last_tick = self.clock()
        self.last_error = None
        self.hold_generation = 0
        self.holds: list[dict] = []                # recent hold records (id, source, verified, states)
        self._last_hold_at = float("-inf")       # the latch: no lease starts within LATCH_S of a hold

    # ---- time ----------------------------------------------------------------------------------------------------
    def advance(self, seconds: float, *, step: float = 1/TICK_HZ):
        """SimClock mode: move time forward in driver-cycle steps, ticking the station."""
        n = max(1, int(round(seconds/step)))
        for _ in range(n):
            self.clock.advance(step)
            self.tick()

    def tick(self):
        now = self.clock()
        if self.faults.pause_until(now) is not None:
            return                                 # a paused process does nothing; time still passes
        with self.lock:
            self._ticks += 1
            self._run_task(now)
            self._follow_setup_pose(now)
            self._watchdog(now)
            if (self.lease and self.lease["active"] and self.lease["kind"] == "video_motion"
                    and self._ticks % REFILL_TICKS == 0 and self.state == "running"):
                self._refill(now)
            for arm in self._arms():
                arm.step(now)
            self._last_tick = now

    def _arms(self):
        return (self.left, self.right)

    def _follow_setup_pose(self, now):
        lease = self.lease
        if not lease or not lease["active"] or lease["kind"] != "setup_pose":
            return
        path = lease["_path"]
        s = min(1.0, (now - path["t0"]) / path["T"])
        u = s * s * s * (10 + s * (-15 + 6 * s))
        self.lease_arm().command(q=[a + u * (b - a) for a, b in zip(path["start"], path["target"])])
        lease["_arrived"] = s >= 1.0

    def arm(self, name: str) -> SimArm:
        """A live arm by name; the parked stand-in is never one."""
        if name == "left":
            return self.left
        if name == "right" and not self.parked_right:
            return self.right
        raise StationError(f"ValueError: the {name} arm is not available on this station")

    def lease_arm(self) -> SimArm:
        return self.arm((self.lease or {}).get("arm") or "left")

    def note(self, event, **detail):
        self.events.append((self.clock(), event, detail))

    def _dead(self, arm, now) -> str | None:
        """Why an arm's driver is dead (its adapter vanished or re-enumerated in DFU), or None."""
        if self.faults.active("usb_drop", now, arm=arm.spec.name) is not None:
            return f"fail to communicate with the motor 4 {arm.spec.adapter}"
        if self.faults.active("dfu", now, arm=arm.spec.name) is not None:
            return f"adapter {arm.spec.adapter} re-enumerated in DFU (bootloader) mode"
        return None

    # ---- reads ---------------------------------------------------------------------------------------------------
    def status(self) -> dict:
        with self.lock:
            dead = [a.spec.name for a in self._arms() if self._dead(a, self.clock())]
            return {"state": self.state, "run": self.run, "arms": {a.spec.name: a.spec.name not in dead for a in self._arms()},
                    "dead_arms": dead, "last_error": self.last_error,
                    "parked_arms": ["right"] if self.parked_right else []}

    def pose(self) -> dict:
        now = self.clock()
        measured, arms = {}, {}
        for arm in self._arms():
            if arm is self.right and self.parked_right:
                arms["right"] = {"body": "yam", "state": "parked"}
                continue
            row = arm.row(now)
            dead = self._dead(arm, now)
            if row is None or dead:
                arms[arm.spec.name] = {"body": arm.spec.body, "state": "unavailable",
                                       "error": dead or "no sample yet"}
                if row is not None:
                    measured[arm.spec.name] = row
                    arms[arm.spec.name]["state"] = "stale"
                continue
            measured[arm.spec.name] = row
            arms[arm.spec.name] = {"body": arm.spec.body, "state": "fresh" if row["fresh"] else "stale"}
        out = {"schema": "manifest-native-pose/1", "version": 4, "server_monotonic_s": now, "measured": measured,
               "arms": arms, "controller_state": self.state, "mixed_motion_left_version": 1}
        if self.lease and self.lease["active"]:
            out["execution_lease"] = {"id": self.lease["id"], "active": True, "kind": self.lease["kind"]}
        return out

    def observe(self, body=None) -> dict:
        with self.lock:
            oid = uuid.uuid4().hex
            self.observations[oid] = self.clock()
            return {"observation_id": oid, "status": self.status(), "measured": self.pose()["measured"],
                    "limits": {a.spec.name: [list(row) for row in a.spec.limits] for a in self._arms()},
                    "lease": self.public_lease()}

    def public_lease(self):
        if not self.lease:
            return None
        return {k: v for k, v in self.lease.items() if not k.startswith("_")}

    # ---- leases --------------------------------------------------------------------------------------------------
    def register_session(self, session_id: str | None = None, *, token: str | None = None, arm="left", seconds=6):
        session_id = session_id or uuid.uuid4().hex
        self.sessions[session_id] = {"id": session_id, "token": token or uuid.uuid4().hex*2, "arm": arm,
                                     "seconds": seconds}
        return session_id

    def action(self, body: dict) -> dict:
        with self.lock:
            since = self.clock() - self._last_hold_at
            if since < LATCH_S:
                raise StationError(f"ValueError: Stop (hold) was pressed {since:.2f} s ago: nothing new starts within "
                                   f"{LATCH_S:g} s of a Stop; send it again")
            if self.lease and self.lease["active"]:
                raise StationError("ValueError: An action is already leased")
            if self.state != "idle":
                raise StationError("ValueError: Station is not idle; the operator keeps control")
            if body.get("observation_id") not in self.observations:
                raise StationError("ValueError: Observe the station before acting")
            status = self.status()
            if status["dead_arms"]:
                why = (status.get("last_error") or {}).get("message")      # the arm's own reason: a fault, a failed rules file
                raise StationError(f"ValueError: The arm cannot move: {why}" if why else
                                   "ValueError: arm driver is dead; reset the arms first")
            seconds = body.get("seconds", 4)
            ref = body.get("reference") or {}
            kind = ref.get("kind")
            longest = HAND_GUIDE_MAX_S if kind == "hand_guide" else 12
            if not finite(seconds) or not 0.5 <= seconds <= longest:
                raise StationError(f"ValueError: seconds must be between 0.5 and {longest:g}")
            if kind == "video_motion":
                if ref.get("session_id") not in self.sessions or not 4 <= seconds <= 12:
                    raise StationError("ValueError: Expired or invalid motion native session")
                arm_name = self.sessions[ref["session_id"]].get("arm") or "left"
            elif kind in ("mixed_jog", "move_home", "setup_pose", "hand_guide"):
                arm_name = ref.get("arm") or "left"
            else:
                raise StationError("ValueError: unsupported reference")
            arm = self.arm(arm_name)
            if self._dead(arm, self.clock()):
                raise StationError("ValueError: arm driver is dead; reset the arms first")
            if kind == "mixed_jog":
                target = ref.get("target_rad")
                joints = len(arm.q)
                if not isinstance(target, list) or len(target) != joints or not all(finite(v) for v in target):
                    raise StationError("ValueError: mixed jog needs " + ("six" if joints == 6 else str(joints)) + " target joints")
                if max(abs(a-b) for a, b in zip(target, arm.q)) > 0.0873:
                    raise StationError("ValueError: Each manual jog is limited to five degrees")
            now = self.clock()
            path = None
            if kind == "setup_pose":
                target = ref.get("target_rad")
                if not isinstance(target, list) or len(target) != len(arm.q) or not all(finite(v) for v in target):
                    raise StationError(f"ValueError: a setup pose needs {len(arm.q)} target joints")
                if any(not lo <= v <= hi and abs(v - now_q) > 1e-3 for v, now_q, (lo, hi) in zip(target, arm.q, arm.spec.limits)):
                    raise StationError("ValueError: the setup pose is outside this arm's joint limits")
                travel = max(abs(a - b) for a, b in zip(target, arm.q))
                if travel > SETUP_POSE_MAX_RAD:
                    raise StationError(f"ValueError: a setup pose may move a joint at most {SETUP_POSE_MAX_RAD:g} rad")
                T = max(2.0, 1.875 * travel / SETUP_POSE_SPEED_RAD_S)
                if seconds < T + 0.5:
                    raise StationError(f"ValueError: this setup pose needs at least {T + 0.5:.1f} seconds")
                path = {"t0": now, "T": T, "start": list(arm.q), "target": [float(v) for v in target]}
            self.run += 1
            self.lease = {"id": uuid.uuid4().hex, "run": self.run, "kind": kind, "seconds": float(seconds),
                          "active": True, "reason": None, "accepted": now, "last_heartbeat": now,
                          "motion_start": now, "_session": ref.get("session_id"), "arm": arm_name,
                          "_retained": list(arm.q_cmd) + [arm.jaw_cmd]}
            self.state = "running"
            if path is not None:
                self.lease["_path"] = path
            if kind == "hand_guide":                   # the arm goes limp for the operator's hands, until the lease ends
                arm.powered = False
                release = getattr(getattr(arm, "adapter", None), "release", None)
                if release is not None:
                    release()
            if kind == "mixed_jog":
                arm.command(q=ref["target_rad"])
            elif kind == "move_home":
                arm.command(q=arm.spec.home)           # the leased Home of ONE arm keeps its grip (the jaw command stays)
            self.note("lease_start", kind=kind, lease=self.lease["id"], arm=arm_name)
            return {"lease": self.public_lease(), "status": self.status()}

    def heartbeat(self, body: dict) -> dict:
        with self.lock:
            if not self.lease or body.get("lease_id") != self.lease["id"]:
                raise StationError("ValueError: Unknown action lease")
            self.lease["last_heartbeat"] = self.clock()
            return {"lease": self.public_lease(), "status": self.status()}

    def supervisor_stop(self, body: dict) -> dict:
        with self.lock:
            if not self.lease or body.get("lease_id") != self.lease["id"]:
                raise StationError("ValueError: Unknown action lease; refusing to stop an unrelated run")
            if self.lease["active"]:
                self._end("supervisor_stop", client_end=True)     # the client closing its own lease; an operator's Stop is hold()
            return {"lease": self.public_lease(), "status": self.status()}

    def _end(self, reason: str, client_end: bool = False):
        lease = self.lease
        if not lease or not lease["active"]:
            return
        if lease["kind"] == "hand_guide":              # held again where the hands left it, never pulled back to the old target
            arm = self.lease_arm()
            arm.powered = True
            arm.command(q=list(arm.q), jaw=arm.jaw)
            engage = getattr(getattr(arm, "adapter", None), "command", None)
            if engage is not None:
                engage(list(arm.q), float(arm.jaw))
        for arm in self._arms():
            if reason == "action_goal_reached" and arm is self.lease_arm():
                continue                           # a reached move_home keeps its goal command (no coast past Home)
            # the stock writer holds where the arms are; a motion that ended by itself lets the jaw finish its last command
            arm.hold_here(urgent=reason not in ORDINARY_ENDS, keep_jaw=reason in ORDINARY_ENDS or client_end)
        lease.update(active=False, reason=reason, ended_at=self.clock())
        lease["holding"] = True
        if self.state in ("running", "starting", "stopping"):
            self.state = "idle"
        self.note("lease_end", reason=reason, lease=lease["id"])

    def _watchdog(self, now):
        lease = self.lease
        if not lease or not lease["active"]:
            return
        if now - lease["last_heartbeat"] > HEARTBEAT_TIMEOUT:
            self._end("client_disconnected")
        elif lease["kind"] == "move_home" and self.lease_arm().at(self.lease_arm().spec.home, tol=0.002):
            self._end("action_goal_reached")
        elif lease["kind"] == "setup_pose" and lease.get("_arrived") and self.lease_arm().at(lease["_path"]["target"], tol=0.02):
            self._end("action_goal_reached")
        elif now - lease["motion_start"] >= lease["seconds"]:
            self._end("action_window_finished")
        elif self._dead(self.lease_arm(), now):
            self.last_error = {"type": "MotorError", "message": self._dead(self.lease_arm(), now)}
            self._end("station_error")
            self.state = "error"

    def _refill(self, now):
        lease = self.lease
        if self.predict is None:
            return
        retained = lease["_retained"]
        session = self.sessions[lease["_session"]]
        arm_name = lease.get("arm") or "left"
        arm = self.arm(arm_name)
        chunk = "q_l_chunk" if arm_name == "left" else "q_r_chunk"
        body = {"session_id": session["id"], "token": session["token"], "observed_at": now,
                "measured": {"left": self.left.row(now), "right": self.right.row(now)},
                "robot_body": "yam" if self.dual else "yam_metal", "enabled_arms": [arm_name],
                "commanded": {chunk: retained}}
        try:
            answer = self._call_predict(body)
            target = answer["target"]
            if not 0 <= self.clock() - answer["acquired_at"] < ANSWER_AGE_S:
                raise StationError("motion action expired before native execution")
            joints = len(arm.q)                    # an answer is the arm's joints, then the jaw
            if (len(target) != joints+1 or not all(finite(v) for v in target) or not 0 <= target[joints] <= 1
                    or max(abs(a-b) for a, b in zip(target[:joints], retained[:joints])) > JOINT_STEP
                    or abs(target[joints]-retained[joints]) > JAW_STEP):
                raise StationError("Invalid or excessive mixed motion command increment")
        except Exception as exc:                   # any refusal ends the lease; the arm holds
            lease["stop_reason"] = str(exc)
            self._end("prediction_error")
            return
        lease["_retained"] = list(target)
        lease["answers"] = lease.get("answers", 0) + 1
        arm.command(q=target[:joints], jaw=target[joints])

    def _call_predict(self, body):
        if callable(self.predict):
            return self.predict(body)
        req = urllib.request.Request(self.predict, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json", "X-Learned-Grasp-Native": "1"})
        with urllib.request.urlopen(req, timeout=0.4) as resp:
            return json.load(resp)

    # ---- vendor station verbs --------------------------------------------------------------------------------------
    def stop(self, body=None) -> dict:
        """The vendor station's Stop (Live view / POST /stop)."""
        with self.lock:
            if self.state == "homing":
                raise StationError("StationError: stop refused: a home move is in progress; it ends within seconds")
            if self.state == "resetting":
                raise StationError("StationError: stop refused: a reset is in progress; it ends within seconds")
            if self.state == "stopping":
                raise StationError("StationError: stop refused: already stopping")
            if self.state in ("running", "starting"):
                self._end("station_stop" if not self.lease or not self.lease["active"] else "operator_took_control")
                self.state = "idle"
            return self.status()

    def home(self, body=None) -> dict:
        with self.lock:
            if self.state not in ("idle", "error") or (self.lease and self.lease["active"]):
                raise StationError(f"StationError: home refused: station is {self.state}; stop the run first")
            if self.status()["dead_arms"]:
                raise StationError("StationError: home refused: arm 'left' driver is dead; reset the arms first")
            self._begin_home(open_grippers=True, why="home")
            return self.status()

    def _begin_home(self, *, open_grippers: bool, why: str, then=None):
        self.state = "homing"
        for arm in self._arms():
            if arm is self.right and self.parked_right:
                continue
            arm.command(q=arm.spec.home, jaw=arm.spec.jaw_open if open_grippers else None)
        self._task = ("home", {"why": why, "end": self.clock() + HOME_S, "then": then})
        self.note("home_start", why=why, open_grippers=open_grippers)

    def reset(self, body=None) -> dict:
        with self.lock:
            if self.state not in ("idle", "error") or (self.lease and self.lease["active"]):
                raise StationError(f"StationError: reset refused: station is {self.state}; stop the run first")
            self.state = "resetting"
            for arm in self._arms():
                if arm is self.right and self.parked_right:
                    continue
                arm.powered = False                # "lets both arms go"
            self._task = ("reset", {"end": self.clock() + REOPEN_S, "cancel_home": False})
            self.note("reset_start")
            return self.status()

    def thermal_retreat(self, arm_name="left"):
        """The thermal retreat's fallback blend: HOMING, a step-wise blend to Home, the grip retained."""
        with self.lock:
            if self.lease and self.lease["active"]:
                self._end("thermal_retreat")
            arm = self.arm(arm_name)
            self.state = "homing"
            start = list(arm.q)
            self._task = ("thermal", {"arm": arm, "start": start, "step": 0, "interrupted": False})
            self.note("thermal_start", arm=arm_name)

    def auto_home(self, kind: str):
        """The vendor's automatic Homes ('budget_cap', 'key_refused'); grippers open. A supervised active lease
        suppresses them (the supervisor wraps the vendor's auto-Home paths)."""
        with self.lock:
            if self.lease and self.lease["active"]:
                self.note("auto_home_suppressed", kind=kind)
                if kind == "key_refused":
                    self.last_error = {"type": "AuthError", "message": "key refused"}
                return False
            if self.state in ("running",):
                self.state = "idle"
            self._begin_home(open_grippers=True, why=f"auto_home:{kind}")
            return True

    def _run_task(self, now):
        if self._task is None:
            return
        kind, data = self._task
        if kind == "home" and now >= data["end"]:
            self._task = None
            self.state = "idle"
            self.note("home_end", why=data["why"])
        elif kind == "reset" and now >= data["end"]:
            for arm in self._arms():
                arm.powered = True
                arm.hold_here()
            self.note("reopened")
            if data.get("cancel_home"):
                self._task = None
                self.state = "idle"
                self.note("home_cancelled_after_reopen")
            else:
                self._task = None
                self._begin_home(open_grippers=True, why="reset")
        elif kind == "thermal":
            arm = data["arm"]
            data["step"] += 1
            a = min(1.0, data["step"]/THERMAL_STEPS)
            arm.command(q=[s + (h - s)*a for s, h in zip(data["start"], arm.spec.home)])   # jaw retained
            if data["step"] >= THERMAL_STEPS:
                self._task = None
                self.state = "idle"
                self.note("thermal_end", arm=arm.spec.name)

    # ---- lease-free hold -----------------------------------------------------------------------------------------
    def hold(self, body=None, *, verified=True) -> dict:
        """POST /supervisor/hold (schema manifest-station-hold/1); never refused."""
        if not self.lease_free_hold:
            raise LookupError("Unknown supervisor route")
        body = body if isinstance(body, dict) else {}
        wait_s = body.get("wait_s", HOLD_WAIT_S)
        wait_s = min(max(float(wait_s), 0.0), HOLD_WAIT_MAX_S) if finite(wait_s) else HOLD_WAIT_S
        actions = []
        with self.lock:                            # step 1-2: latch, end every writer (never a new target)
            t0 = self.clock()
            self.hold_generation += 1
            self._last_hold_at = t0
            record = {"id": uuid.uuid4().hex, "generation": self.hold_generation,
                      "source": str(body.get("source") or "unknown")[:40], "reason": body.get("reason"),
                      "verified": bool(verified), "client": "sim", "latch_s": LATCH_S}
            if self.lease and self.lease["active"]:
                self._end("supervisor_stop")       # the reason every Manifest client reads as a Stop
                actions.append("lease_ended")
            if self._task is not None:
                kind, data = self._task
                if kind in ("home", "thermal"):
                    self._task = None
                    self.state = "idle"
                    self.note("interrupted", what=kind)
                    actions.append("thermal_retreat_held" if kind == "thermal" else "home_interrupted")
                elif kind == "reset":
                    data["cancel_home"] = True     # the reopen cannot be interrupted; the Home after it is cancelled
                    self.note("reset_home_cancelled")
                    actions.append("reset_home_cancelled")
            if self.state in ("running", "starting", "stopping"):
                self.state = "idle"
                actions.append("run_stopped")
            for arm in self._held_arms():
                if arm.powered and not self._dead(arm, t0):
                    arm.hold_here()                # the loop's own hold: the arm settles where it is
        # step 3: wait up to wait_s for the arms to settle, then measure (settled, then measure)
        while self.clock() < t0 + wait_s - 1e-9 and not self._settled():
            if hasattr(self.clock, "advance"):
                self.advance(1/TICK_HZ)            # SimClock: time passes in driver cycles, deterministically
            else:
                time.sleep(SETTLE_POLL_S)          # RealClock: the HTTP front end's ticker moves the arms
        with self.lock:
            now = self.clock()
            writer = self._writer()
            arms = {arm.spec.name: self._measure(arm, now, writer) for arm in self._held_arms()}
            reply = {"schema": "manifest-station-hold/1", "version": 1, "hold": record,
                     "actions": actions or ["nothing_moving"], "pending": False, "loop_error": None,
                     "lease": None if not self.lease else {"id": self.lease["id"], "active": self.lease["active"],
                                                           "reason": self.lease.get("reason")},
                     "elapsed_ms": round((now - t0)*1000, 1), "controller_state": self.state,
                     "last_error": self.last_error, "writer": writer, "arms": arms,
                     "parked_arms": ["right"] if self.parked_right else [],
                     "holding": bool(arms) and all(row["holding"] for row in arms.values()),
                     "measured_monotonic_s": now, "held_at": t0, "state": self.state}
            self.holds = (self.holds + [dict(record, states={k: v["state"] for k, v in arms.items()})])[-8:]
            self.note("hold", arms={k: v["state"] for k, v in arms.items()}, verified=bool(verified))
            return reply

    def _held_arms(self):
        return [arm for arm in self._arms() if not (arm is self.right and self.parked_right)]

    def _writer(self) -> str | None:
        """What still writes the arms: 'run', 'reset', 'home', 'lease', or None."""
        if self.state in ("starting", "running", "stopping"):
            return "run"
        if self.state == "resetting" or (self._task is not None and self._task[0] == "reset"):
            return "reset"
        if self.state == "homing" or self._task is not None:
            return "home"
        if self.lease and self.lease["active"]:
            return "lease"
        return None

    def _settled(self) -> bool:
        writer = self._writer()
        if writer == "reset":
            return True                            # a Reset's reopen cannot be held: answer at once
        if writer is not None:
            return False
        now = self.clock()
        for arm in self._held_arms():
            if self._dead(arm, now) or not arm.powered:
                continue                           # it cannot settle; the answer says why
            row = arm.row(now)
            # still, and on a FRESH sample (waits out a late tick within the budget; a frozen driver ends 'stale')
            if (row is None or now - row["source_monotonic_s"] > STALE_S
                    or max(abs(v) for v in row["velocities_rad_s"]) > MOVING_RAD_S):
                return False
        return True

    def _measure(self, arm, now, writer) -> dict:
        """One arm's measured state now: never assumed, never the simulator's ground truth."""
        row = {"arm": arm.spec.name, "body": arm.spec.body}
        if self.state == "resetting" or writer == "reset" or not arm.powered:
            row.update(state="resetting", confirmed=False, holding=False, writer="reset",
                       cause="Reset is reopening the drivers; the arm may hold under power with nothing driving it")
            return row
        sample = arm.row(now)
        dead = self._dead(arm, now)
        if dead or sample is None:
            row.update(state="fault", confirmed=False, holding=False, cause=dead or "the driver does not answer")
            return row
        q, dq, cmd = sample["positions_rad"], sample["velocities_rad_s"], sample["commanded_rad"]
        speed = max(abs(v) for v in dq)
        age = max(0.0, now - sample["source_monotonic_s"])
        if writer is not None or speed > MOVING_RAD_S:
            state = "moving"
        elif age > STALE_S:
            state = "stale"
        else:
            state = "holding"
        row.update(state=state, confirmed=state == "holding", holding=state == "holding", positions_rad=list(q),
                   commanded_rad=list(cmd), velocities_rad_s=list(dq), gripper=sample["gripper"],
                   commanded_gripper=sample["commanded_gripper"], max_speed_rad_s=round(speed, 4),
                   tracking_rad=round(max(abs(a - b) for a, b in zip(q, cmd)), 5), sample_age_ms=round(age*1000, 1),
                   sample_id=sample["sample_id"])
        if state == "stale":
            row["cause"] = (f"the newest sample is {age*1000:.0f} ms old (> {STALE_S*1000:g} ms): the hold cannot be "
                            f"confirmed")
        if writer is not None:
            row["writer"] = writer
        return row

    # ---- route table for the HTTP front end --------------------------------------------------------------------------
    def routes(self):
        get = {"/status": lambda b: self.status(), "/supervisor/pose": lambda b: self.pose()}
        post = {"/supervisor/observe": self.observe, "/supervisor/action": self.action,
                "/supervisor/heartbeat": self.heartbeat, "/supervisor/stop": self.supervisor_stop,
                "/stop": self.stop, "/home": self.home, "/reset": self.reset,
                "/supervisor/reload-reference": self._reload}
        if self.lease_free_hold:
            post["/supervisor/hold"] = self.hold
        return get, post

    def _reload(self, body=None):
        if self.state != "idle" or (self.lease and self.lease["active"]):
            raise StationError("ValueError: Reference code can only reload while holding idle")
        return {"loaded": "mac_reference"}
