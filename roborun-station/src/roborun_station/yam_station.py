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
- GET /supervisor/pose: the stamped manifest-native-pose/1 rows;
- a `policy` lease (roborun_station.policy): one or both arms, up to 600 s with a 1 s heartbeat; the runner pushes timed
  chunks (POST /supervisor/policy/chunk), the station checks each one and plays it on its own tick, clipped per tick, and an
  empty buffer holds; POST /supervisor/policy/end ends it as an ordinary end. POST /supervisor/policy/stop {run_id} is the
  per-task Stop of one policy run (the runtime's POST /v1/tasks/{id}/stop): it ends that run's lease the way a task's own
  /supervisor/stop does, and remembers the run, so a run still warming up is refused its lease and sees itself stopped in
  status()/pose() (stopped_runs).
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

from . import policy as pol
from .core import Faults, RealClock, SimArm, finite, metal_right_spec, yam_left_spec, yam_right_spec

ORDINARY_ENDS = ("action_window_finished", "action_goal_reached", "policy_finished")   # the commands ran out; nobody asked for a stop
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


STOPPED_RUNS_KEPT = 32       # policy runs remembered as stopped by id (a run id is a fresh 32-hex task id)


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
        self.stopped_runs: list[str] = []          # policy runs stopped by id (newest last): no lease for them any more
        self.holds: list[dict] = []                # recent hold records (id, source, verified, states)
        self._last_hold_at = float("-inf")       # the latch: no lease starts within LATCH_S of a hold
        self.policy_tool_speed_m_s = pol.POLICY_TOOL_SPEED_M_S   # a policy run's tool speed cap (robot.yaml may set it)

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
            if self.lease and self.lease["active"] and self.lease["kind"] == "policy" and self.state == "running":
                self._play_policy(now)
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

    def lease_arms(self) -> list:
        """Every arm the lease drives (a policy lease may drive both)."""
        lease = self.lease or {}
        return [self.arm(name) for name in (lease.get("arms") or [lease.get("arm") or "left"])]

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
                    "parked_arms": ["right"] if self.parked_right else [], "hold_generation": self.hold_generation,
                    "stopped_runs": list(self.stopped_runs)}

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
               "arms": arms, "controller_state": self.state, "mixed_motion_left_version": 1,
               "hold_generation": self.hold_generation,         # a client that started before a Stop sees it here
               "stopped_runs": list(self.stopped_runs)}         # and a run stopped by its id (policy_stop) sees that
        if self.lease and self.lease["active"]:
            out["execution_lease"] = {"id": self.lease["id"], "active": True, "kind": self.lease["kind"]}
            if self.lease["kind"] == "policy":
                with self.lock:                        # the tick changes the buffers under this lock
                    out["execution_lease"].update(arms=list(self.lease["arms"]), run_id=self.lease.get("run_id"),
                                                  policy=self._policy_public(now))
        return out

    def observe(self, body=None) -> dict:
        with self.lock:
            oid = uuid.uuid4().hex
            self.observations[oid] = self.clock()
            return {"observation_id": oid, "status": self.status(), "measured": self.pose()["measured"],
                    "limits": {a.spec.name: [list(row) for row in a.spec.limits] for a in self._arms()},
                    "jaw_range": {a.spec.name: [a.spec.jaw_closed, a.spec.jaw_open] for a in self._held_arms()},
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
                if self.lease["kind"] == "policy":
                    raise StationError(f"ValueError: An action is already leased: a policy run is in progress "
                                       f"(task {self.lease.get('run_id')}); Stop it first")
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
            longest = HAND_GUIDE_MAX_S if kind == "hand_guide" else pol.POLICY_MAX_S if kind == "policy" else 12
            if not finite(seconds) or not 0.5 <= seconds <= longest:
                raise StationError(f"ValueError: seconds must be between 0.5 and {longest:g}")
            if kind == "video_motion":
                if ref.get("session_id") not in self.sessions or not 4 <= seconds <= 12:
                    raise StationError("ValueError: Expired or invalid motion native session")
                arm_name = self.sessions[ref["session_id"]].get("arm") or "left"
            elif kind in ("mixed_jog", "move_home", "setup_pose", "hand_guide"):
                arm_name = ref.get("arm") or "left"
            elif kind == "policy":
                return self._start_policy(body, ref, float(seconds))
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

    # ---- policy leases (roborun_station.policy) -----------------------------------------------------------------------
    def _start_policy(self, body, ref, seconds):
        names = ref.get("arms") or [ref.get("arm") or "left"]
        if (not isinstance(names, list) or not names or len(set(names)) != len(names)
                or any(n not in ("left", "right") for n in names)):
            raise StationError("ValueError: a policy lease names its arms: ['left'], ['right'] or ['left', 'right']")
        names = [n for n in ("left", "right") if n in names]
        arms = [self.arm(n) for n in names]
        now = self.clock()
        for arm in arms:
            if self._dead(arm, now):
                raise StationError("ValueError: arm driver is dead; reset the arms first")
        gripper = ref.get("gripper", True)
        if not isinstance(gripper, bool):
            raise StationError("ValueError: gripper must be true or false")
        if gripper:
            for arm in arms:
                if not pol.jaw_range_usable((arm.spec.jaw_closed, arm.spec.jaw_open)):
                    raise StationError(f"ValueError: the {arm.spec.name} arm has no jaw range; policy runs need one")
        cap = self.policy_tool_speed_m_s
        asked = ref.get("max_tool_speed_m_s")
        if asked is not None:
            if not finite(asked) or asked <= 0:
                raise StationError("ValueError: max_tool_speed_m_s must be a number above 0")
            cap = asked if cap is None else min(cap, float(asked))        # a request can lower the cap, never raise it
        since = ref.get("since_hold_generation")      # the hold generation the run saw when it started (optional)
        if since is not None:
            if isinstance(since, bool) or not isinstance(since, int):
                raise StationError("ValueError: since_hold_generation must be an integer")
            if since < self.hold_generation:
                raise StationError("ValueError: Stop was pressed after this run started; nothing moved; run again")
            if since > self.hold_generation:
                raise StationError("ValueError: the arms' station restarted after this run started; nothing moved; "
                                   "run again")
        run_id = str(ref.get("run_id") or "")[:64] or None
        if run_id is not None and run_id in self.stopped_runs:
            raise StationError(f"ValueError: Stop was pressed for this run (task {run_id}); nothing moved")
        self.run += 1
        self.lease = {"id": uuid.uuid4().hex, "run": self.run, "kind": "policy", "seconds": seconds, "active": True,
                      "reason": None, "accepted": now, "last_heartbeat": now, "motion_start": now, "_session": None,
                      "arm": names[0], "arms": names, "run_id": run_id, "heartbeat_s": pol.POLICY_HEARTBEAT_S,
                      "tool_speed_m_s": cap, "gripper": gripper, "limits": pol.limits(),
                      "_retained": list(arms[0].q_cmd) + [arms[0].jaw_cmd],
                      "_policy": {"buffers": {n: pol.PolicyBuffer() for n in names}, "t_last": now, "seq": None,
                                  "dry_since": now, "in_row": 0, "joints": {a.spec.name: len(a.q) for a in arms},
                                  "jaw_range": {a.spec.name: (a.spec.jaw_closed, a.spec.jaw_open) for a in arms}},
                      "policy": {"chunks": 0, "played_rows": 0, "dry_ticks": 0, "clipped_ticks": 0, "refused": {},
                                 "skipped_rows": 0}}
        self.state = "running"
        self._policy_started(arms, cap)
        self.note("lease_start", kind="policy", lease=self.lease["id"], arm=names[0], arms=names)
        return {"lease": self.public_lease(), "status": self.status(), "limits": pol.limits()}

    def _policy_started(self, arms, cap):
        """Hook: the arms' drivers take the run's tool speed cap (AdapterStation). The simulated arms have no tool model."""
        self.lease["tool_speed_active"] = False

    def _policy_ended(self):
        """Hook: the arms' drivers get their own tool speed cap back (AdapterStation)."""

    def _policy_public(self, now):
        lease = self.lease
        state = lease["_policy"]
        return dict(lease["policy"], buffer_s=round(max((b.seconds_left(now) for b in state["buffers"].values()), default=0.0), 4),
                    tool_speed_m_s=lease.get("tool_speed_m_s"), tool_speed_active=lease.get("tool_speed_active"))

    def _policy_lease(self, body):
        if not self.lease or body.get("lease_id") != self.lease["id"]:
            raise StationError("ValueError: Unknown action lease")
        if self.lease["kind"] != "policy":
            raise StationError("ValueError: this lease is not a policy lease")

    def policy_chunk(self, body: dict) -> dict:
        """POST /supervisor/policy/chunk {lease_id, seq, hz, skip_s, rows: {side: [[q.., gripper fraction], ...]}}."""
        with self.lock:
            self._policy_lease(body)
            lease = self.lease
            now = self.clock()
            if not lease["active"]:
                err = pol.ChunkRefused(f"the lease has ended ({lease.get('reason')})", "lease")
                err.reply["lease"] = self.public_lease()
                raise err
            state, stats = lease["_policy"], lease["policy"]
            try:
                seq = body.get("seq")
                if seq is not None and (not isinstance(seq, int) or isinstance(seq, bool)
                                        or (state["seq"] is not None and seq <= state["seq"])):
                    raise pol.ChunkRefused("seq must be an integer above the last chunk's", "seq")
                names = lease["arms"]
                commanded = {n: (list(self.arm(n).q_cmd), self.arm(n).jaw_cmd) for n in names}
                rows, skipped = pol.check_chunk(body, sides=names, joints=state["joints"],
                                                grippers={n: lease["gripper"] for n in names},
                                                jaw_ranges=state["jaw_range"], commanded=commanded, arrival=now)
            except pol.ChunkRefused as exc:
                stats["refused"][exc.code] = stats["refused"].get(exc.code, 0) + 1
                state["in_row"] += 1
                self.note("policy_refused", code=exc.code, why=str(exc))
                if state["in_row"] >= pol.POLICY_REFUSALS_END:
                    lease["stop_reason"] = f"{pol.POLICY_REFUSALS_END} chunks refused in a row; the last: {exc}"
                    self._end("policy_error")
                exc.reply["lease"] = self.public_lease()
                raise
            state["in_row"] = 0
            state["seq"] = body.get("seq", state["seq"])
            for n in names:
                state["buffers"][n].replace(rows[n])
            stats["chunks"] += 1
            stats["skipped_rows"] += skipped
            last = {}
            for n in names:
                arm = self.arm(n)
                q = list(arm.q_cmd)
                last[n] = q + ([pol.fraction_from_jaw(arm.jaw_cmd, state["jaw_range"][n])] if lease["gripper"] else [])
            return {"accepted": True, "skipped_rows": skipped, "buffer_s": self._policy_public(now)["buffer_s"],
                    "clipped_ticks": stats["clipped_ticks"], "last_played": last, "lease": self.public_lease()}

    def policy_end(self, body: dict) -> dict:
        """POST /supervisor/policy/end {lease_id, reason}: the run is over; the arms settle where they are (an ordinary end)."""
        with self.lock:
            self._policy_lease(body)
            if self.lease["active"]:
                self.lease["end_note"] = str(body.get("reason") or "")[:120] or None
                self._end("policy_finished")
            return {"lease": self.public_lease(), "status": self.status()}

    def policy_stop(self, body: dict) -> dict:
        """POST /supervisor/policy/stop {run_id}: the per-task Stop of one policy run, by its task id. A running lease of that
        run ends as a task's own /supervisor/stop ends it (reason supervisor_stop, the arms hold where they are); a run that
        has no lease yet (its model still warming up) is refused one from now on. Any other run or lease is left alone.
        Never refused for a well-formed id: stopping a run that already ended is fine."""
        run_id = body.get("run_id") if isinstance(body, dict) else None
        if not isinstance(run_id, str) or not 1 <= len(run_id) <= 64:
            raise StationError("ValueError: run_id must be the policy run's task id")
        with self.lock:
            if run_id in self.stopped_runs:
                self.stopped_runs.remove(run_id)
            self.stopped_runs = (self.stopped_runs + [run_id])[-STOPPED_RUNS_KEPT:]
            lease = self.lease
            ended = False
            if lease and lease["kind"] == "policy" and lease.get("run_id") == run_id and lease["active"]:
                lease["end_note"] = "per-task Stop"
                self._end("supervisor_stop", client_end=True)
                ended = True
            self.note("policy_stop", run_id=run_id, lease_ended=ended)
            mine = lease is not None and lease["kind"] == "policy" and lease.get("run_id") == run_id
            return {"run_id": run_id, "lease_ended": ended, "lease": self.public_lease() if mine else None,
                    "status": self.status()}

    def _play_policy(self, now):
        lease = self.lease
        state, stats = lease["_policy"], lease["policy"]
        played = False
        for name in lease["arms"]:
            arm = self.arm(name)
            got = state["buffers"][name].target(now, list(arm.q_cmd), arm.jaw_cmd, state["t_last"])
            if got is None:
                continue                                     # nothing left: the arm keeps its last target (holds)
            q, jaw = got
            q, jaw, clipped = pol.clip_step(list(arm.q_cmd), arm.jaw_cmd, q, jaw)
            stats["clipped_ticks"] += clipped
            arm.command(q=q, jaw=jaw)
            played = True
        state["t_last"] = now
        if played:
            stats["played_rows"] += 1
            state["dry_since"] = None
            lease["_retained"] = list(self.lease_arm().q_cmd) + [self.lease_arm().jaw_cmd]
        else:
            stats["dry_ticks"] += 1
            if state["dry_since"] is None:
                state["dry_since"] = now
            elif now - state["dry_since"] >= pol.POLICY_DRY_END_S:
                lease["stop_reason"] = f"no rows to play for {pol.POLICY_DRY_END_S:g} s"
                self._end("policy_error")

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
        if lease["kind"] == "policy":                  # nothing more plays: the buffers go before the arms are held
            for buffer in lease["_policy"]["buffers"].values():
                buffer.clear()
            self._policy_ended()
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
        if now - lease["last_heartbeat"] > lease.get("heartbeat_s", HEARTBEAT_TIMEOUT):
            self._end("client_disconnected")
        elif lease["kind"] == "move_home" and self.lease_arm().at(self.lease_arm().spec.home, tol=0.002):
            self._end("action_goal_reached")
        elif lease["kind"] == "setup_pose" and lease.get("_arrived") and self.lease_arm().at(lease["_path"]["target"], tol=0.02):
            self._end("action_goal_reached")
        elif now - lease["motion_start"] >= lease["seconds"]:
            self._end("action_window_finished")
        else:
            for arm in self.lease_arms():
                dead = self._dead(arm, now)
                if dead:
                    self.last_error = {"type": "MotorError", "message": dead}
                    self._end("station_error")
                    self.state = "error"
                    break

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
                                                           "reason": self.lease.get("reason"), "kind": self.lease["kind"],
                                                           "run_id": self.lease.get("run_id")},
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
                "/supervisor/reload-reference": self._reload,
                "/supervisor/policy/chunk": self.policy_chunk, "/supervisor/policy/end": self.policy_end,
                "/supervisor/policy/stop": self.policy_stop}
        if self.lease_free_hold:
            post["/supervisor/hold"] = self.hold
        return get, post

    def _reload(self, body=None):
        if self.state != "idle" or (self.lease and self.lease["active"]):
            raise StationError("ValueError: Reference code can only reload while holding idle")
        return {"loaded": "mac_reference"}
