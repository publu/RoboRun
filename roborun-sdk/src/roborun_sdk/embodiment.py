"""Your arm on Roborun: write an ArmAdapter for its motors; the SDK turns it into a Roborun station.

    from roborun_sdk.embodiment import ArmAdapter

    class MyArm(ArmAdapter):
        home = [0, -0.5, 1.0, 0, 0.5, 0]                       # rad, the arm joints (as many as your arm has; its profile says the same)
        limits = [(-3, 3)] * 6                                 # one (low, high) per joint
        def connect(self):  self.bus = open_my_bus()
        def read(self):     return dict(positions_rad=self.bus.joints(), gripper=self.bus.jaw())   # gripper: 0 closed .. 1 open
        def command(self, positions_rad, gripper):  self.bus.send(positions_rad, gripper)

    roborun station --adapter mymodule:MyArm                  # one arm (the other slot parked)
    roborun station --adapter mymodule:MyArm --right mymodule:MyOtherArm

What Roborun does around it (the same station logic every Roborun robot runs, not reimplemented per arm): motion
leases with the heartbeat watchdog, the ~15 Hz motion pulls from this robot's Roborun with per-answer step limits
(0.04 rad joints, 0.081 jaw) and answer-age checks, Stop / hold (latched, answered from YOUR measured state), Home,
Reset, the pose rows the UI and the planner read, the motion session files. Your adapter only moves the motors to the
targets it is given and reports where they are; it is called at the station's 30 Hz tick:
    command(positions_rad, gripper)   every tick, the current target (smooth, already step-limited)
    read() -> {positions_rad, gripper[, velocities_rad_s, temperatures_c]}   every tick, measured
    hold()      Stop: hold where you are (default: command the last measured position)
    release()   Reset: let the motors go (default: nothing)
    reopen()    Reset, after a failure: open the hardware again (default: close(), then connect())
    health()    {ok, error, ...}: shown in /status (default: judged from read() failures)
A read() that raises leaves the arm's newest sample aging: the station then reports it stale and refuses motion,
which is the safe default for a dropped bus.
"""
from __future__ import annotations

import importlib
import json
import math
import signal
import sys
import threading
import time
from pathlib import Path


GRIP_FORCE_CAP = 0.5                # the YAM station's gripper force cap: rows report effort on this scale


class ArmAdapter:
    """Subclass this for your arm. Joint angles in radians (len(home) arm joints, six by default), gripper 0 (closed) .. 1 (open).

    read() returns {positions_rad, gripper} and optionally velocities_rad_s, temperatures_c and gripper_effort: how hard
    the jaw presses, 0 (free) .. 1 (at its force limit) -- e.g. motor current / its limit. A jaw that stops on an object
    with effort is how Roborun knows it is holding something; without it a grasp stays unverified."""
    home: list = [0.0] * 6
    limits: list = [(-math.pi, math.pi)] * 6
    joint_speed_rad_s = 0.6        # only used to estimate coasting when Stop arrives mid-move
    jaw_speed = 0.6

    def connect(self): pass
    def read(self) -> dict: raise NotImplementedError
    def command(self, positions_rad, gripper): raise NotImplementedError
    def hold(self):
        st = self.read(); self.command(st['positions_rad'], st['gripper'])
    def settle(self):
        """The commands ended without a Stop. Default: the same as hold()."""
        self.hold()
    def release(self): pass
    def close(self): pass
    def reopen(self):
        self.close(); self.connect()
    def health(self) -> dict:
        return dict(ok=True)


def wait_for_stop_signal():
    """Blocks until SIGTERM (`roborun down`, systemd) or Ctrl-C, so the code after it closes the arms either way."""
    stop = threading.Event()
    for number in (signal.SIGTERM, signal.SIGINT):
        signal.signal(number, lambda *_: stop.set())
    while not stop.wait(1.0):
        pass


class KinematicArm(ArmAdapter):
    """A software arm that follows its commands at a bounded speed: test the whole path (UI -> motion -> station)
    before your hardware is wired. --adapter roborun_sdk.embodiment:KinematicArm (optionally home/limits from a profile)."""
    def __init__(self, home=None, limits=None):
        if home is not None: self.home = list(home)
        if limits is not None: self.limits = [tuple(l) for l in limits]
        elif len(self.limits) != len(self.home): self.limits = [(-math.pi, math.pi)] * len(self.home)
        self.q = list(self.home); self.jaw = 1.0; self.target = (list(self.home), 1.0); self.t = time.monotonic()

    def command(self, positions_rad, gripper):
        self.target = (list(positions_rad), float(gripper))

    def read(self):
        now = time.monotonic(); dt = min(.1, now - self.t); self.t = now
        q_t, j_t = self.target
        step = self.joint_speed_rad_s * dt
        vel = []
        for i, (a, b) in enumerate(zip(self.q, q_t)):
            d = max(-step, min(step, b - a)); self.q[i] = a + d; vel.append(d / dt if dt > 0 else 0.0)
        self.jaw += max(-self.jaw_speed * dt, min(self.jaw_speed * dt, j_t - self.jaw))
        return dict(positions_rad=list(self.q), gripper=self.jaw, velocities_rad_s=vel)


def load_adapter(ref, **kw):
    """'package.module:ClassName' -> an instance (connected)."""
    mod, _, cls = ref.partition(':')
    if not cls: raise SystemExit(f'--adapter {ref!r}: module:ClassName')
    if '.' not in sys.path: sys.path.insert(0, '.')
    obj = getattr(importlib.import_module(mod), cls)(**kw)
    if not isinstance(obj, ArmAdapter):
        raise SystemExit(f'{ref} is not a roborun_sdk.embodiment.ArmAdapter')
    obj.connect()
    return obj


def _station_classes():
    """The Roborun station logic (roborun_station) with arms backed by adapters instead of simulation."""
    from roborun_station import FakeYamStation
    from roborun_station.core import ArmSpec, SimArm

    class AdapterArm(SimArm):
        def __init__(self, adapter, name, clock, faults, seed=0):
            self.joints = len(adapter.home)
            limits = adapter.limits
            if len(limits) != self.joints:
                if limits is not ArmAdapter.limits:
                    raise ValueError(f'{type(adapter).__name__}: {len(limits)} limits for {self.joints} joints')
                limits = [(-math.pi, math.pi)] * self.joints
            spec = ArmSpec(name=name, body='yam', home=tuple(adapter.home), limits=tuple(tuple(l) for l in limits),
                           joint_speed_rad_s=adapter.joint_speed_rad_s, jaw_speed=adapter.jaw_speed, noise_rad=0.0,
                           velocity_noise_counts=0, adapter=f'adapter:{type(adapter).__name__}', drops_when_limp=False)
            self.adapter = adapter
            self.errors, self.error_text, self.was_ok = 0, None, True
            super().__init__(spec, clock, faults, seed=seed)
            try:                                         # a real arm stays where it is until it is told to move
                self._sample_real()
                self.q_cmd, self.jaw_cmd = list(self.q), self.jaw
                self._sample(clock())
            except Exception:
                pass

        def _sample_real(self):
            st = self.adapter.read()
            q = [float(v) for v in st['positions_rad']][:self.joints]
            if len(q) != self.joints or not all(math.isfinite(v) for v in q):
                raise ValueError(f'read(): positions_rad must be {self.joints} finite joint angles')
            self.q, self.jaw = q, float(st['gripper'])
            self.dq = [float(v) for v in (st.get('velocities_rad_s') or [0.0] * self.joints)][:self.joints]
            if st.get('temperatures_c'): self.temps = [float(v) for v in st['temperatures_c']][:self.joints + 1]
            now = time.monotonic(); last = getattr(self, '_jaw_seen', None)
            self.jaw_speed = abs(self.jaw - last[1]) / max(1e-3, now - last[0]) if last else 0.0
            self._jaw_seen = (now, self.jaw)
            e = st.get('gripper_effort')
            self.grip_effort = None if e is None else float(min(1.0, max(0.0, float(e))))
            self.jaw_state = st.get('jaw_state')           # the arm's own jaw guard: free, at_stop (closed on nothing) or gripping

        def _sample(self, now):
            super()._sample(now)
            if getattr(self, 'grip_effort', None) is not None and self.last_row is not None:
                self.last_row['gripper_force_control'] = dict(available=True, fresh=True, cap=GRIP_FORCE_CAP,
                                                              effort=self.grip_effort * GRIP_FORCE_CAP,
                                                              speed=float(self.jaw_speed), observed_at=time.monotonic())
            if self.last_row is not None:
                self.last_row['jaw_state'] = getattr(self, 'jaw_state', None)     # None: this arm cannot tell
                thermal = self.health().get('thermal')
                if thermal:                              # telemetry: every pose row says how warm the arm is
                    self.last_row['thermal'] = {k: thermal.get(k) for k in ('state', 'hottest_c', 'motor')}

        def health(self):
            try:
                out = dict(self.adapter.health() or {})
            except Exception as exc:
                out = dict(ok=False, error=f'{type(exc).__name__}: {exc}'[:200], kind='hardware')
            out.setdefault('ok', True)
            if out['ok'] and self.error_text:            # a plain adapter: its read() or command() keeps failing
                out.update(ok=False, error=self.error_text, kind='hardware')
            return out

        def fault(self):
            """Why this arm cannot move, or None."""
            row = self.health()
            return None if row['ok'] else (row.get('error') or 'the arm reports a fault')

        def _note_error(self, exc):
            self.errors += 1
            self.error_text = f'{type(exc).__name__}: {exc}'[:200]
            if self.errors in (1, 10, 100) or self.errors % 1000 == 0:
                print(json.dumps(dict(event='adapter_error', arm=self.spec.name, error=self.error_text)), flush=True)

        def step(self, now):
            with self.lock:
                ok = self.health()['ok'] if self.powered else False
                if self.was_ok and not ok and self.powered:
                    self.q_cmd, self.jaw_cmd = list(self.q), self.jaw      # the driver holds; the station stops asking for more
                self.was_ok = ok or not self.powered
                try:
                    if self.powered and ok:
                        self.adapter.command(list(self.q_cmd), float(self.jaw_cmd))
                    self._sample_real()
                except Exception as exc:                 # no sample: the station sees it age and refuses motion
                    self._note_error(exc)
                    return
                self.errors, self.error_text = 0, None
                self.last_step = now
                self._sample(now)

        def reopen(self):
            """Reset: this arm's hardware comes back, or the reason stays on the status."""
            self.adapter.reopen()
            with self.lock:
                self.errors, self.error_text, self.was_ok = 0, None, True
                self._sample_real()
                self.q_cmd, self.jaw_cmd = list(self.q), self.jaw
                self._sample(self.clock())

        def hold_here(self, urgent=True, keep_jaw=False):
            jaw = self.jaw_cmd
            super().hold_here(urgent, keep_jaw)
            try:
                (self.adapter.hold if urgent else self.adapter.settle)()
                rest = getattr(self.adapter, 'held_target', lambda: None)()
                if rest is not None:                     # the station keeps sending where the arm is really coming to rest
                    self.command(q=rest)
                if keep_jaw:
                    self.command(jaw=jaw)
            except Exception:
                pass

    class AdapterStation(FakeYamStation):
        """The station: the backend writes motion sessions to <sessions>/<id>.json, the station reads them there."""
        def __init__(self, left, right=None, sessions_dir=None, **kw):
            super().__init__(right='yam' if right is not None else 'parked', **kw)
            self.left = AdapterArm(left, 'left', self.clock, self.faults)
            if right is not None:
                self.right = AdapterArm(right, 'right', self.clock, self.faults, seed=1)
            self.sessions_dir = Path(sessions_dir) if sessions_dir else None
            self._reopening = None
            self._reopen_errors = {}

        def _adapter_arms(self):
            return [arm for arm in (self.left, self.right) if isinstance(arm, AdapterArm)]

        def _dead(self, arm, now):
            found = super()._dead(arm, now)
            if found or not isinstance(arm, AdapterArm):
                return found
            return self._reopen_errors.get(arm.spec.name) or arm.fault()

        def _watchdog(self, now):
            super()._watchdog(now)
            lease = self.lease
            if not lease or not lease['active'] or lease['kind'] != 'video_motion':
                return
            arm = self.lease_arm()
            refused = getattr(arm.adapter, 'rule_refusal', lambda: None)() if isinstance(arm, AdapterArm) else None
            if refused:                                  # the arm is holding: the task must hear why, not wait out its window
                self._end(refused)

        def status(self):
            out = super().status()
            arms = {arm.spec.name: arm.health() for arm in self._adapter_arms()}
            errors = {name: self._reopen_errors.get(name) or row.get('error') for name, row in arms.items()
                      if not row['ok'] or name in self._reopen_errors}
            out.update(arm_status=arms, arm_errors=errors)
            if errors and not out.get('last_error'):
                name = sorted(errors)[0]
                out['last_error'] = dict(type='ArmFault', arm=name, message=errors[name], next='POST /reset reopens it')
            return out

        def reset(self, body=None):
            out = super().reset(body)                    # refused while running: then nothing below happens
            self._reopen_errors = {}
            arms = self._adapter_arms()
            failed = [arm for arm in arms if not arm.health()['ok']]
            for arm in arms:
                if arm in failed:
                    try: arm.adapter.release()
                    except Exception: pass
                else:                                    # a healthy arm keeps its pose and whatever it is holding
                    arm.powered = True
                    arm.hold_here()
            def reopen():
                for arm in failed:
                    try:
                        arm.reopen()
                    except Exception as exc:
                        self._reopen_errors[arm.spec.name] = f'reopen failed: {type(exc).__name__}: {exc}'[:300]
                        print(json.dumps(dict(event='arm_reopen_failed', arm=arm.spec.name, error=self._reopen_errors[arm.spec.name])), flush=True)
            self._reopening = threading.Thread(target=reopen, daemon=True, name='arm-reopen')
            self._reopening.start()
            return out

        def _run_task(self, now):
            kind = self._task[0] if self._task else None
            if kind == 'reset' and self._reopening is not None and self._reopening.is_alive():
                self._task[1]['end'] = max(self._task[1]['end'], now + 0.1)
                return
            ending = kind == 'reset' and now >= self._task[1]['end']
            if ending:                                   # Reset never homes: Home opens the jaws, and something may be held
                self._task[1]['cancel_home'] = True
            super()._run_task(now)
            if ending:
                dead = {arm.spec.name: self._dead(arm, now) for arm in self._adapter_arms() if self._dead(arm, now)}
                if dead:                                 # no Home with an arm missing: what works holds, the status says why
                    self._task = None
                    self.state = 'error'
                    for arm in self._adapter_arms():
                        arm.hold_here()
                    name = sorted(dead)[0]
                    self.last_error = dict(type='ArmFault', arm=name, message=dead[name])
                else:
                    self.last_error = None

        def _load_session(self, sid):
            if not isinstance(sid, str) or sid in self.sessions or self.sessions_dir is None:
                return
            try:
                value = json.loads((self.sessions_dir / (sid + '.json')).read_text())
            except (OSError, ValueError):
                return
            if value.get('id') != sid or not 0 <= time.time() - float(value.get('created_at', 0)) < 30:
                return
            self.register_session(sid, token=value.get('token'), arm=value.get('arm') if value.get('arm') in ('left', 'right') else 'left',
                                  seconds=value.get('seconds', 6))

        def action(self, body):
            ref = (body or {}).get('reference') or {}
            if ref.get('kind') == 'video_motion':
                self._load_session(ref.get('session_id'))
            return super().action(body)

        def _call_predict(self, body):
            return super()._call_predict(dict(body, robot_body='yam'))

    return AdapterStation


def run_station(left, right=None, *, port=8091, sessions_dir=None, predict='http://127.0.0.1:8120/internal/video-motion/predict'):
    from roborun_station import SimServer
    station = _station_classes()(left, right, sessions_dir=sessions_dir, lease_free_hold=True, predict=predict)
    srv = SimServer(station, port=port).start()
    print(json.dumps(dict(event='station', url=srv.url, left=type(left).__name__, right=type(right).__name__ if right else 'parked')), flush=True)
    return station, srv


def cmd_station(a) -> int:
    kw = {}
    if a.profile_home:                                  # KinematicArm and friends: start at the profile's home
        rig = json.loads(Path(a.profile_home).read_text())
        kw = dict(home=rig['arms']['left']['home'])
    from .arms.safety import ArmsBusy, ArmsLock
    lock = ArmsLock('roborun station')
    try:
        lock.acquire()
    except ArmsBusy as exc:
        raise SystemExit(str(exc))
    left = load_adapter(a.adapter, **kw)
    right = load_adapter(a.right, **kw) if a.right else None
    station, srv = run_station(left, right, port=a.port, sessions_dir=a.sessions, predict=a.predict)
    try:
        wait_for_stop_signal()
    finally:
        srv.close()
        for x in (left, right):
            if x is not None: x.close()
        lock.release()
    return 0


def add_parser(sub):
    s = sub.add_parser('station', help='run a Roborun station for your arm(s) from an ArmAdapter')
    s.add_argument('--adapter', required=True, help='module:Class of the left (or only) arm')
    s.add_argument('--right', help='module:Class of the right arm (omit: the right slot is parked)')
    s.add_argument('--port', type=int, default=8091)
    s.add_argument('--sessions', help="where this robot's Roborun writes motion sessions")
    s.add_argument('--predict', default='http://127.0.0.1:8120/internal/video-motion/predict')
    s.add_argument('--profile-home', help="a profile's rig.json: adapters that accept home= start there")
    s.set_defaults(func=cmd_station)
