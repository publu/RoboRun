"""What every real arm shares: a fixed-rate thread that turns the station's commands into smooth motor targets.

A driver subclasses StreamedArm and implements the five *_hardware methods. Motion rules, the same for every arm:
- nothing moves when the arm connects: the motor target starts at the measured pose;
- targets are followed with limited speed, acceleration and jerk, at `rate_hz`, however coarse the commands are;
- commands that stop arriving end in a smooth hold at the current pose;
- any hardware failure latches: read() raises, so the station sees no fresh sample and refuses motion;
- a guard that trips (temperature, blocked joint) keeps the stream alive and holding, and refuses motion until Reset;
- a jaw that pushes without moving stops pushing;
- with a rest pose, closing parks first, and motors only go off once the arm is confirmed there and carried by it.
"""
import collections
import importlib.util
import json
import math
import threading
import time
from pathlib import Path

from ..embodiment import ArmAdapter
from . import safety


MAX_SHAFT_LAG_S = 0.15              # the most a Stop believes a shaft trails its command


class ArmHardwareError(RuntimeError):
    pass


def _load_rule(path):
    """check(side, positions_rad, gripper, state) from the user's rules file."""
    if not str(path).endswith('.py') or not Path(path).is_file():
        raise ValueError(f'rules: {path} is not a Python file (.py) on this computer. Next: fix `rules:` in robot.yaml')
    spec = importlib.util.spec_from_file_location('roborun_user_rules', path)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        raise ValueError(f'rules: {path} could not be loaded ({type(exc).__name__}: {exc}). Next: fix that file') from exc
    if not callable(getattr(module, 'check', None)):
        raise ValueError(f'{path} has no check(side, positions_rad, gripper, state)')
    return module.check


class JerkLimiter:
    """Follows a target position per joint with bounded speed, acceleration and jerk."""

    def __init__(self, positions, *, speed, accel, jerk, gain=8.0, damping=20.0, limits=None):
        self.q = [float(v) for v in positions]
        self.v = [0.0] * len(self.q)
        self.a = [0.0] * len(self.q)
        self.speed, self.accel, self.jerk, self.gain, self.damping = float(speed), float(accel), float(jerk), gain, damping
        self.limits = limits
        self._brake = None

    def reset(self, positions):
        self.q = [float(v) for v in positions]
        self.v = [0.0] * len(self.q)
        self.a = [0.0] * len(self.q)
        self._brake = None

    @property
    def braking(self):
        return self._brake is not None

    def brake(self, accel, jerk, measured=None, velocity=None, lag=None):
        """Stop now: every joint runs down to zero speed together, along one smooth curve that starts from the speed and
        acceleration it has and ends with both at zero. Its length is what `accel` and `jerk` allow for the fastest joint, so a slow
        arm stops sooner and no joint ever steps. Returns the command the joints come to rest at. Targets are ignored until then.

        A position-controlled motor trails its command by about lag * speed. Given the measured joints, their speed and that lag
        (seconds, per joint), the curve is planned for the shaft and the command leads it by the same lag, so the shaft stops when
        the curve ends instead of creeping up to a stopped command. Whatever the estimate, the command starts where it is."""
        if self._brake is not None:
            return list(self._brake['end'])
        q0 = list(self.q) if measured is None else [float(v) for v in measured]
        v0 = list(self.v) if velocity is None else [float(v) for v in velocity]
        lead = [0.0] * len(q0) if lag is None else [float(v) for v in lag]
        speed = max(map(abs, self.v), default=0.0)
        push = max(map(abs, self.a), default=0.0)
        if speed < 1e-4 and push < 1e-3:                  # the command is at rest: measurement noise is no reason to move it
            return list(self.q)
        speed = max(speed, max(map(abs, v0), default=0.0))
        span = max(1.5 * speed / accel, math.sqrt(6.0 * speed / jerk), 2.0 * push / jerk, 0.02)
        offset = [c - (q + v * l) for c, q, v, l in zip(self.q, q0, v0, lead)]
        end = [q + span * (v / 2.0 + a * span / 12.0) + o for q, v, a, o in zip(q0, v0, self.a, offset)]
        if self.limits is not None:
            end = [max(min(lo, c), min(max(hi, c), e)) for e, c, (lo, hi) in zip(end, self.q, self.limits)]
        self._brake = dict(q=q0, v=v0, a=list(self.a), lead=lead, offset=offset, start=list(self.q), span=span, t=0.0, end=end)
        return list(end)

    def _step_brake(self, dt):
        b = self._brake
        b['t'] += dt
        s, T = min(1.0, b['t'] / b['span']), b['span']
        for i, (q0, v0, a0, lead, offset) in enumerate(zip(b['q'], b['v'], b['a'], b['lead'], b['offset'])):
            shaft = q0 + T * (v0 * (s - s ** 3 + s ** 4 / 2) + a0 * T * (s ** 2 / 2 - 2 * s ** 3 / 3 + s ** 4 / 4))
            self.v[i] = v0 * (1 - 3 * s ** 2 + 2 * s ** 3) + a0 * T * s * (1 - s) ** 2
            self.a[i] = v0 * (6 * s ** 2 - 6 * s) / T + a0 * (1 - 4 * s + 3 * s ** 2)
            q = shaft + self.v[i] * lead + offset
            if self.limits is not None:                   # as in step(): never past a limit
                lo, hi = self.limits[i]
                q = max(min(lo, b['start'][i]), min(max(hi, b['start'][i]), q))
            self.q[i] = q
        if s >= 1.0:
            self.q, self.v, self.a = list(b['end']), [0.0] * len(self.q), [0.0] * len(self.q)
            self._brake = None
        return list(self.q)

    def stopping_point(self):
        """Where the joints come to rest if braking starts now."""
        return [q + v * abs(v) / (2 * self.accel) for q, v in zip(self.q, self.v)]

    def step(self, target, dt):
        if self._brake is not None:
            return self._step_brake(dt)
        for i, goal in enumerate(target):
            error = goal - self.q[i]
            wanted = min(self.speed, self.gain * abs(error), math.sqrt(2 * 0.7 * self.accel * abs(error)))
            wanted = math.copysign(wanted, error)
            a_wanted = max(-self.accel, min(self.accel, (wanted - self.v[i]) * self.damping))
            change = self.jerk * dt
            self.a[i] += max(-change, min(change, a_wanted - self.a[i]))
            self.v[i] = max(-self.speed, min(self.speed, self.v[i] + self.a[i] * dt))
            before = self.q[i]
            self.q[i] += self.v[i] * dt
            if self.limits is not None:                   # never past a limit; an arm that starts outside one may only come back
                lo, hi = self.limits[i]
                bounded = max(min(lo, before), min(max(hi, before), self.q[i]))
                if bounded != self.q[i]:
                    self.q[i], self.v[i], self.a[i] = bounded, 0.0, 0.0
            if abs(goal - self.q[i]) < 1e-4 and abs(self.v[i]) < 1e-3:
                self.q[i], self.v[i], self.a[i] = goal, 0.0, 0.0
        return list(self.q)

    def slow_to(self, before, scale):
        """Take back part of the step just made: the joints travel `scale` of it, and carry that speed on."""
        for i, start in enumerate(before):
            self.q[i] = start + (self.q[i] - start) * scale
            self.v[i] *= scale
            self.a[i] *= scale
        return list(self.q)


def _say(event, **fields):
    print(json.dumps(dict(event=event, **fields), default=str), flush=True)


class StreamedArm(ArmAdapter):
    rate_hz = 100.0
    joint_speed_rad_s = 0.6
    joint_accel_rad_s2 = 2.0
    joint_jerk_rad_s3 = 20.0
    stop_accel_rad_s2 = None            # Stop: default twice joint_accel_rad_s2
    stop_jerk_rad_s3 = None             # Stop: default three times joint_jerk_rad_s3
    rule_refused_after_s = 0.4          # a rules file that keeps refusing this long has refused the motion
    jaw_speed = 0.6
    max_jaw_step = None                 # default: what jaw_speed allows in one cycle
    command_timeout_s = 0.5
    sample_timeout_s = 0.25
    ramp_s = 0.0
    stall_guard = True
    stall_error_rad = 0.2               # ~3x the tracking lag of a YAM or Metal at full speed, well above their gravity sag
    stall_velocity_rad_s = 0.05
    stall_time_s = 0.5
    jaw_guard = True
    jaw_stall_time_s = 0.3
    jaw_stall_error = 0.03
    gripper_effort = 1.0
    gripper_force_cap = 0.9
    gripper_hold_effort = 0.7
    gripper_open_margin = 0.003         # YAM: commanding 1.0 keeps the fingers pressed on the open stop, .997 does not
    max_tool_speed_m_s = 0.5            # above the motion's 0.2 m/s speed cap, so it only catches motion that was not shaped
    park_speed_rad_s = 0.3
    park_timeout_s = 30.0
    park_tolerance_rad = 0.05           # a Metal can stop a few hundredths of a rad short of a command under its own weight
    park_sag_rad = 0.02
    dry_run = False
    can_power_off = False               # True: power_down() and switch_off() below really take every motor's power away

    EXTRA_SETTINGS = ()                 # names a driver reads from self.options itself

    def __init__(self, bus=None, side='left', home=None, limits=None, rules=None, **options):
        self.bus, self.side, self.options = bus, side, options
        self.settings = dict(options)
        self.temp_warn_c = options.get('temp_warn_c')
        self.temp_stop_c = options.get('temp_stop_c')
        self.home_before_motors_off = options.get('home_before_motors_off')
        self.powered_at_connect = str(options.get('powered_at_connect', 'hold'))
        if self.powered_at_connect not in ('hold', 'refuse'):
            raise ValueError(f'powered_at_connect is hold or refuse, not {self.powered_at_connect!r}')
        for name in safety.POSITIVE:
            if options.get(name) is not None:
                setattr(self, name, safety.number(name, options[name], above=0))
        for name in safety.FRACTIONS:
            if options.get(name) is not None:
                setattr(self, name, safety.number(name, options[name], 0.0, 1.0))
        if self.gripper_hold_effort < safety.GRIP_HOLD_MIN:
            raise ValueError(f'gripper_hold_effort must be at least {safety.GRIP_HOLD_MIN:g}: a weaker hold reads as an empty jaw, '
                             f'not {self.gripper_hold_effort:g}')
        for name in safety.NON_NEGATIVE:
            if options.get(name) is not None:
                setattr(self, name, safety.number(name, options[name], 0.0))
        for name in safety.FLAGS:
            if name in options:
                setattr(self, name, bool(options[name]))
        if 'max_tool_speed_m_s' in options:
            value = options['max_tool_speed_m_s']
            self.max_tool_speed_m_s = None if value in (None, False) else safety.number('max_tool_speed_m_s', value, above=0)
        rest = options.get('rest_pose')
        self.rest_pose = None if rest is None else [float(v) for v in rest]
        self.unused_options = sorted(k for k in options if k not in safety.KNOWN and k not in self.EXTRA_SETTINGS)
        self.setting_warnings = []
        self._rule = _load_rule(rules) if rules else None
        if home is not None: self.home = [float(v) for v in home]
        if limits is not None: self.limits = [tuple(l) for l in limits]
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()   # one hardware write at a time: the stream's command, or a release
        self._stop = threading.Event()
        self._thread = None
        self._sample = None
        self._sample_at = 0.0
        self._error = None
        self._trip = None
        self._target = None
        self._jaw_target = None
        self._jaw = None
        self._command_at = 0.0
        self._limp = False
        self._shaper = None
        self._closed = False
        self._park = None
        self._park_done = threading.Event()
        self._park_result = None
        self._last_q = None
        self._guards_from = 0.0
        self._thermal = self._stall = None
        self._jaw_guard = safety.JawGuard(self.jaw_stall_error, self.jaw_stall_time_s, self.gripper_force_cap)
        self._tool, self._tool_at, self._tool_note = None, None, 'no arm model'
        self._rule_refused_at = None
        self._shaft_speed, self._lag_sum, self._lag_weight = None, None, None
        self._export_at = 0.0
        self._dry_said = (0.0, None)
        self.dry_log = collections.deque(maxlen=200)
        self.adopted = []                               # motors found already powered at connect
        self.close_report = None

    # -- a driver implements these ------------------------------------------------------------------------------
    def open_hardware(self):
        """Open the bus and motors without moving them."""
        raise NotImplementedError

    def open_dry(self):
        """Open for reading only: nothing is enabled, configured or written. Needed for `dry_run`."""
        raise ArmHardwareError(f'{type(self).__name__} cannot do a dry run: it needs an open_dry() that enables nothing')

    def read_hardware(self) -> dict:
        """{positions_rad, gripper (0 closed .. 1 open)} and optionally velocities_rad_s, temperatures_c, gripper_effort."""
        raise NotImplementedError

    def write_hardware(self, positions_rad, gripper):
        raise NotImplementedError

    def limp_hardware(self):
        """Stop holding: torque off, or gravity compensation only."""
        raise NotImplementedError

    def close_hardware(self):
        pass

    def motors_off_hardware(self) -> str | None:
        """The arm sits at its rest pose and the stream has stopped: fade the holding out, and switch off only if the arm
        stays put. Returns None when the motors are off, else why they were left on."""
        return 'this arm has no way to switch its motors off from here'

    def thermal_motors(self):
        """[(name, motor model, 'joint'|'gripper')], one per entry of temperatures_c."""
        return [(f'joint{i + 1}', '', 'joint') for i in range(len(self.limits))] + [('gripper', '', 'gripper')]

    def default_tool_model(self):
        """{xml, site} of this arm's own model, when the driver has one."""
        return None

    def jaw_squeeze(self):
        """How far past the contact point (fraction of the stroke) the jaw is told to go while gripping."""
        return 0.02

    def survey(self) -> dict:
        """Read-only check for `roborun hw check`: must not enable or move any motor."""
        raise NotImplementedError(f'{type(self).__name__} has no read-only check yet')

    # -- the station's side -------------------------------------------------------------------------------------
    def connect(self):
        if self.rest_pose is not None and len(self.rest_pose) != len(self.limits):
            raise ArmHardwareError(f'rest_pose has {len(self.rest_pose)} joints, this arm has {len(self.limits)}')
        self._thermal = safety.ThermalGuard(self.thermal_motors(), self.temp_warn_c, self.temp_stop_c)
        (self.open_dry if self.dry_run else self.open_hardware)()
        first = self.read_hardware()
        q = [float(v) for v in first['positions_rad']]
        if len(q) != len(self.limits):
            raise ArmHardwareError(f'the arm reports {len(q)} joints but {len(self.limits)} have limits')
        self._load_tool_model()
        now = time.monotonic()
        self._stall = safety.StallGuard(len(q), self.stall_error_rad, self.stall_velocity_rad_s, self.stall_time_s)
        self._jaw_guard.reset()
        with self._lock:
            self._sample, self._sample_at = first, now
            self._target, self._jaw_target, self._jaw = list(q), float(first['gripper']), float(first['gripper'])
            self._shaper = JerkLimiter(q, speed=self.joint_speed_rad_s, accel=self.joint_accel_rad_s2, jerk=self.joint_jerk_rad_s3,
                                       limits=list(self.limits))
            self._last_q, self._tool_at = list(q), None
            self._guards_from = now + self.ramp_s + 0.1
            self._error, self._trip, self._limp, self._park = None, None, False, None
            self._check_temperature(first, now, settled=True)
        self._stop = threading.Event()
        self._closed = False
        self._thread = threading.Thread(target=self._run, daemon=True, name=f'arm-{self.side}')
        self._thread.start()

    def read(self) -> dict:
        with self._lock:
            if self._error is not None:
                raise ArmHardwareError(self._error)
            if self._sample is None or time.monotonic() - self._sample_at > self.sample_timeout_s:
                raise ArmHardwareError('no fresh sample from the arm')
            return dict(self._sample)

    def _rule_state(self):
        with self._lock:
            state = dict(self._sample or {})
            state.update(commanded_rad=list(self._shaper.q) if self._shaper else None, jaw_state=self._jaw_guard.state,
                         thermal=self._thermal.report() if self._thermal else None, side=self.side)
        pose = None
        if self._tool is not None and state.get('positions_rad'):
            pose = self._tool.pose(state['positions_rad'])
        state['tool_pose'] = pose                       # 4x4, arm base frame; None when the arm's model is not available
        state['tool_position_m'] = None if pose is None else [pose[0][3], pose[1][3], pose[2][3]]
        return state

    def command(self, positions_rad, gripper):
        if len(positions_rad) != len(self.limits):            # a short command must not silently freeze the last joints
            raise ArmHardwareError(f'command has {len(positions_rad)} joints, this arm has {len(self.limits)}')
        if self._rule is not None:
            try:
                answer = self._rule(self.side, [float(v) for v in positions_rad], float(gripper), self._rule_state())
                if answer is False:
                    if self._rule_refused_at is None:
                        self._rule_refused_at = time.monotonic()
                    return self.hold()
                self._rule_refused_at = None
                if answer not in (None, True):
                    positions_rad, gripper = answer
                    if len(positions_rad) != len(self.limits):
                        raise ValueError(f'returned {len(positions_rad)} joints')
            except Exception as exc:                          # a broken rule must not let motion through
                with self._lock:
                    self._error = f'your rules file failed: {type(exc).__name__}: {exc}'[:300]
                raise ArmHardwareError(self._error)
        q = [max(lo, min(hi, float(v))) for v, (lo, hi) in zip(positions_rad, self.limits)]
        with self._lock:
            if self._error is not None:
                raise ArmHardwareError(self._error)
            if self._trip is not None:
                raise ArmHardwareError(self._trip['reason'])
            if self._park is not None:
                raise ArmHardwareError('the arm is parking')
            if self._limp:                                    # re-engage from wherever the arm was left
                now = [float(v) for v in self._sample['positions_rad']]
                self._shaper.reset(now)
                self._jaw = float(self._sample['gripper'])
                self._limp = False
                self._guards_from = time.monotonic() + self.ramp_s + 0.1
                self._stall.reset()
                self._jaw_guard.reset()
            self._target, self._jaw_target = q, max(0.0, min(1.0, float(gripper)))
            self._command_at = time.monotonic()

    def hold(self):
        """Stop: brake now, as hard as the stop limits allow, and stay where the arm comes to rest. A jaw that is moving freely stops
        where it is measured, not where its command had already got to (the motor trails its command: holding the command, a
        closing jaw would go on closing by that lag after the Stop). A jaw gripping something keeps its grip: a Stop never drops it."""
        with self._lock:
            if self._shaper is not None and not self._limp and self._park is None:
                self._target = self._braked_target()
                self._jaw_target = self._jaw
                measured = (self._sample or {}).get('gripper')
                if (self._jaw_guard.state == 'free' and measured is not None and (self._sample or {}).get('gripper_available') is not False
                        and math.isfinite(float(measured))):
                    self._jaw = self._jaw_target = max(0.0, min(1.0, float(measured)))

    def settle(self):
        """The commands simply ended (not a Stop): run out at the normal limits and stay there."""
        with self._lock:
            if self._shaper is not None and not self._limp and self._park is None and not self._shaper.braking:
                self._target = self._shaper.stopping_point()
                self._jaw_target = self._jaw

    def _braked_target(self):
        accel = self.stop_accel_rad_s2 or 2.0 * self.joint_accel_rad_s2
        jerk = self.stop_jerk_rad_s3 or 3.0 * self.joint_jerk_rad_s3
        measured = [float(v) for v in self._sample['positions_rad']] if self._sample else None
        ready = measured is not None and self._shaft_speed is not None and len(measured) == len(self._shaper.q)
        return self._shaper.brake(max(accel, self.joint_accel_rad_s2), max(jerk, self.joint_jerk_rad_s3), measured if ready else None,
                                  list(self._shaft_speed) if ready else None, self._shaft_lag() if ready else None)

    def _track_shaft(self, measured, velocity):
        """Keeps the measured speed (smoothed) and how far behind its command each shaft runs, for a Stop to brake the shaft itself."""
        if self._shaft_speed is None or len(self._shaft_speed) != len(measured):
            self._shaft_speed, self._lag_sum, self._lag_weight = [0.0] * len(measured), [0.0] * len(measured), [0.0] * len(measured)
        for i, (command, q, v) in enumerate(zip(self._shaper.q, measured, velocity)):
            self._shaft_speed[i] += 0.25 * (float(v) - self._shaft_speed[i])
            self._lag_sum[i] = 0.98 * self._lag_sum[i] + float(v) * (command - q)
            self._lag_weight[i] = 0.98 * self._lag_weight[i] + float(v) * float(v)

    def _shaft_lag(self):
        return [max(0.0, min(MAX_SHAFT_LAG_S, s / w)) if w > 1e-3 else 0.0 for s, w in zip(self._lag_sum, self._lag_weight)]

    def held_target(self):
        """Where the arm is coming to rest after hold() / settle()."""
        with self._lock:
            return None if self._target is None else list(self._target)

    def rule_refusal(self):
        """Why the user's rules file has stopped this arm's motion, or None. Refusing every command for
        rule_refused_after_s counts: a single refused command may just be a rule trimming one step."""
        at = self._rule_refused_at
        if at is None or time.monotonic() - at < self.rule_refused_after_s:
            return None
        return f'your rules file refused this motion of the {self.side} arm'

    def release(self):
        with self._lock:
            self._limp = True
            self._finish_park('released')
        if not self.dry_run:
            with self._write_lock:                # after a stream write already under way, never before it
                self.limp_hardware()

    def health(self) -> dict:
        with self._lock:
            error = self._error
            trip = dict(self._trip) if self._trip else None
            if error is None and self._thread is not None and not self._thread.is_alive() and not self._closed:
                error = 'the arm stream stopped'
            out = dict(ok=error is None and trip is None, error=error or (trip['reason'] if trip else None),
                       kind='hardware' if error else trip['kind'] if trip else None, dry_run=self.dry_run,
                       thermal=self._thermal.report() if self._thermal else None, jaw=self._jaw_guard.state,
                       parking=self._park is not None, park=self._park_result, adopted=list(self.adopted),
                       tool_speed=dict(limit_m_s=self.max_tool_speed_m_s, active=self._tool is not None and self.max_tool_speed_m_s is not None,
                                       note=None if self._tool is not None else self._tool_note),
                       warnings=list(self.setting_warnings))
        return out

    def reopen(self):
        """Reset: a failed adapter is closed and opened again; a tripped guard clears once its cause has. A healthy arm is left alone."""
        with self._lock:
            failed = self._error is not None or self._thread is None or not self._thread.is_alive()
            trip = self._trip
            if trip is not None and trip['kind'] == 'thermal' and not self._thermal.cooled():
                raise ArmHardwareError(f'{trip["reason"]}: still too hot to continue')
            if not failed:
                self._trip = None
                if self._thermal is not None and self._thermal.state == 'stop':
                    self._thermal.clear()
                self._stall.reset()
                self._jaw_guard.reset()
                return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        try:
            self.close_hardware()
        except Exception:
            pass                                               # what is being reopened is what failed
        self.connect()

    def park(self, timeout=None):
        """Move to the rest pose and confirm it from measured positions. The jaw stays as it is: whatever it holds stays held."""
        target = self._rest_target()
        limit = self.park_timeout_s if timeout is None else timeout
        with self._lock:
            if self._error is not None or self._thread is None or not self._thread.is_alive():
                return dict(state='failed', why=self._error or 'the arm is not running')
            if self._trip is not None and self._trip['kind'] != 'thermal':
                return dict(state='failed', why=self._trip['reason'])
            self._start_park(target, limit)
        self._park_done.wait(limit + 1.0)
        with self._lock:
            self._finish_park('timeout')
            return dict(self._park_result)

    def _rest_target(self):
        rest = self.rest_pose if self.rest_pose is not None else self.home
        return [max(lo, min(hi, float(v))) for v, (lo, hi) in zip(rest, self.limits)]

    def _start_park(self, target, limit):
        if self._limp:
            self._shaper.reset(self._sample['positions_rad'])
            self._limp = False
        self._park_done.clear()
        self._park_result = None
        self._park = dict(target=target, deadline=time.monotonic() + limit, still=None)
        self._shaper.speed = min(self.joint_speed_rad_s, self.park_speed_rad_s)

    def _finish_park(self, state, why=None):
        if self._park is None:
            return
        self._park = None
        self._shaper.speed = self.joint_speed_rad_s
        self._park_result = dict(state=state, why=why)
        self._park_done.set()

    def _parks_on_close(self):
        if self.dry_run:
            return False
        wanted = self.home_before_motors_off
        return self.rest_pose is not None if wanted is None else bool(wanted)

    def close(self):
        if self._closed:
            return
        report = dict(parked=False, motors_off=False, why=None)
        if self._thread is not None and self._parks_on_close():
            result = self.park()
            report.update(parked=result['state'] == 'confirmed', why=result.get('why') or (None if result['state'] == 'confirmed' else result['state']))
        with self._lock:
            gripping = self._jaw_guard.state == 'gripping'
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        self._closed = True
        try:
            if report['parked']:
                if gripping:
                    report['why'] = 'the jaw is holding something: motors left on'
                else:
                    report['why'] = self.motors_off_hardware()
                    report['motors_off'] = report['why'] is None
        except Exception as exc:
            report['why'] = f'{type(exc).__name__}: {exc}'[:200]
        self.close_report = report
        if self._parks_on_close():
            _say('arm_closed', arm=self.side, **report)
        self.close_hardware()

    # -- the end of a supervised session --------------------------------------------------------------------------
    def force_motors_off(self):
        """Every motor off now, whether or not the arm is carried. Only after the operator was told to support it."""
        raise NotImplementedError

    def switch_off(self):
        """From a new connection: every motor off, however the last session ended. Returns the motors still on."""
        raise NotImplementedError

    def power_down(self, grace_s=5.0, say=print, wait=time.sleep):
        """Ends a guided step with no motor left on. Holding fades first and the arm is watched; an arm that sinks without
        it gets `grace_s` for a hand to reach it, because it is then switched off anyway."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        self._closed = True
        try:
            why = self.motors_off_hardware()
            if why and why != 'the motors were not on':
                say(f'{why}. Support the arm now: its motors switch off in {grace_s:g} seconds.')
                wait(grace_s)
                self.force_motors_off()
        finally:
            self.close_hardware()

    def let_go(self, force=False):
        """`roborun down`: motors off only if something else carries the arm. Holding fades while the arm is watched; an arm
        that starts to sink is taken back where it is and keeps being held. Returns None when off, else why it still holds.
        `force`: the operator has said the arm is supported."""
        if not self.can_power_off:
            self.close()
            return None
        with self._lock:
            gripping = self._jaw_guard.state == 'gripping'
        if gripping and not force:
            return 'the jaw is holding something'
        if not force and not self._at_rest():
            return 'it is not at its rest pose, so nothing is known to carry it'
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        try:
            why = self.motors_off_hardware()
        except Exception as exc:
            why = f'{type(exc).__name__}: {exc}'[:200]
        if why and why != 'the motors were not on':
            if not force:
                self._hold_again()
                return why
            self.force_motors_off()
        self._closed = True
        self.close_hardware()
        return None

    def _at_rest(self):
        with self._lock:
            q = list((self._sample or {}).get('positions_rad') or ())
        return bool(q) and all(abs(float(a) - b) <= max(self.park_tolerance_rad, 0.05) for a, b in zip(q, self._rest_target()))

    def _hold_again(self):
        first = self.read_hardware()
        q = [float(v) for v in first['positions_rad']]
        now = time.monotonic()
        with self._lock:
            self._sample, self._sample_at = first, now
            self._shaper.reset(q)
            self._target, self._jaw_target, self._jaw = list(q), float(first['gripper']), float(first['gripper'])
            self._command_at, self._limp, self._last_q = 0.0, False, list(q)
            self._guards_from = now + self.ramp_s + 0.1
            self._stall.reset()
            self._jaw_guard.reset()
        self._soft, self._ramp_steps = False, 10 ** 9        # full stiffness at once, at the pose it is in: no ramp to sink through
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f'arm-{self.side}')
        self._thread.start()

    # -- guards -------------------------------------------------------------------------------------------------
    def _load_tool_model(self):
        self._tool, self._tool_note = None, 'no arm model'
        spec = self.options.get('tool_model') or self.default_tool_model()
        if not spec:
            return
        try:
            if isinstance(spec, str):
                spec = dict(xml=spec)
            self._tool = safety.ToolModel(spec['xml'], len(self.limits), spec.get('site', 'grasp_site'))
            self._tool_note = None
        except Exception as exc:                               # the limit is then off, and status says why
            self._tool_note = f'{type(exc).__name__}: {exc}'[:200]
            if 'tool_model' in self.options:
                self.setting_warnings.append(f'tool_model could not be loaded: {self._tool_note}')

    def _trip_now(self, kind, reason, hold_measured=False):
        """Called under the lock. The stream goes on holding; motion is refused until Reset."""
        if self._trip is not None:
            return
        self._trip = dict(kind=kind, reason=reason)
        if hold_measured:
            q = [float(v) for v in self._sample['positions_rad']]
            self._shaper.reset(q)
            self._target = q
        else:
            self._target = self._braked_target()
        self._jaw_target = self._jaw
        self._command_at = 0.0
        _say('arm_guard', arm=self.side, kind=kind, reason=reason)

    def _check_temperature(self, sample, now, settled=False):
        change = self._thermal.update(sample.get('temperatures_c'), now, settled=settled)
        if change is None:
            return
        report = self._thermal.report()
        if change == 'warn':
            _say('arm_temperature', arm=self.side, level='warn', motor=report['motor'], temperature_c=report['hottest_c'])
            return
        reason = f'{self.side} arm {report["motor"]} is at {report["hottest_c"]:.0f} C: stopped, let it cool, then Reset'
        self._trip_now('thermal', reason)
        if self.rest_pose is not None and not settled and not self._limp and not self.dry_run:   # never moves at connect
            self._start_park(self._rest_target(), self.park_timeout_s)

    def _limit_tool_speed(self, before, q, period):
        if self._tool is None or self.max_tool_speed_m_s is None or self._shaper.braking:
            return q
        at = self._tool.position(q)
        if self._tool_at is None:
            self._tool_at = self._tool.position(before)
        speed = math.dist(at, self._tool_at) / period
        if speed > self.max_tool_speed_m_s:
            q = self._shaper.slow_to(before, self.max_tool_speed_m_s / speed)
            at = self._tool.position(q)
        self._tool_at = at
        return q

    def _update_park(self, measured, velocity, now):
        park = self._park
        self._target = park['target']
        arrived = all(abs(m - t) <= self.park_tolerance_rad for m, t in zip(measured, park['target']))
        if arrived and all(abs(v) < self.stall_velocity_rad_s for v in velocity):
            park['still'] = now if park['still'] is None else park['still']
            if now - park['still'] >= 0.3:
                self._finish_park('confirmed')
        else:
            park['still'] = None
            if now > park['deadline']:
                self._finish_park('failed', f'not at the rest pose after {self.park_timeout_s:g} s')

    def _cycle(self, sample, now, period):
        """One stream cycle under the lock: the (joints, jaw) to send, or None."""
        self._sample, self._sample_at = sample, now
        self._check_temperature(sample, now)
        measured = [float(v) for v in sample['positions_rad']]
        velocity = sample.get('velocities_rad_s')
        if velocity is None:
            velocity = [(a - b) / period for a, b in zip(measured, self._last_q)]
        self._last_q = measured
        if self._limp:
            return None
        self._track_shaft(measured, velocity)
        if self._command_at and now - self._command_at > self.command_timeout_s:
            if not self._shaper.braking:              # a brake already ends where the arm will rest
                self._target = self._shaper.stopping_point()
            self._jaw_target = self._jaw
            self._command_at = 0.0
        if self._park is not None:
            self._update_park(measured, velocity, now)
        guarded = not self.dry_run and now >= self._guards_from
        if guarded and self.stall_guard:
            joint = self._stall.update(self._shaper.q, measured, velocity, now)
            if joint is not None:
                error = self._shaper.q[joint] - measured[joint]
                reason = (f'{self.side} arm joint {joint + 1} is blocked: {abs(error):.2f} rad from its target and not moving')
                self._finish_park('failed', reason)
                self._trip_now('stall', reason, hold_measured=True)
        before = list(self._shaper.q)
        q = self._limit_tool_speed(before, self._shaper.step(self._target, period), period)
        step = self.jaw_speed * period
        if self.max_jaw_step is not None:
            step = min(step, self.max_jaw_step)
        self._jaw += max(-step, min(step, self._jaw_target - self._jaw))
        jaw = self._jaw
        if guarded and self.jaw_guard and sample.get('gripper_available') is not False:
            jaw = self._jaw_guard.update(self._jaw_target, self._jaw, float(sample['gripper']), sample.get('gripper_effort'),
                                         now, self.jaw_squeeze())
            if self._jaw_guard.state != 'free':
                self._jaw = jaw
        return q, jaw

    def _log_dry(self, q, jaw):
        row = dict(positions_rad=[round(v, 4) for v in q], gripper=round(jaw, 4))
        self.dry_log.append(row)
        at, last = self._dry_said
        now = time.monotonic()
        if now - at >= 0.5 and row != last:
            self._dry_said = (now, row)
            _say('dry_run_would_send', arm=self.side, **row)

    def _export(self, sample, now):
        path = self.options.get('temperature_export')
        if not path or now - self._export_at < 1.0 or not sample.get('temperatures_c'):
            return
        self._export_at = now
        try:
            file = Path(path).expanduser()
            file.parent.mkdir(parents=True, exist_ok=True)
            if file.exists() and file.stat().st_size > 10_000_000:
                file.replace(file.with_suffix(file.suffix + '.1'))
            with open(file, 'a') as out:
                out.write(json.dumps(dict(t=time.time(), arm=self.side, temperatures_c=sample['temperatures_c'],
                                          state=self._thermal.state)) + '\n')
        except OSError as exc:
            self.options = dict(self.options, temperature_export=None)
            self.setting_warnings.append(f'temperature_export stopped: {exc}')

    # -- the stream ---------------------------------------------------------------------------------------------
    def _run(self):
        period = 1.0 / self.rate_hz
        next_at = time.monotonic()
        stop = self._stop
        while not stop.is_set():
            try:
                sample = self.read_hardware()
                now = time.monotonic()
                with self._lock:
                    out = self._cycle(sample, now, period)
                if out is not None:
                    with self._write_lock:
                        if not self._limp:            # a release since this cycle wins: its write must be the last
                            (self._log_dry if self.dry_run else self.write_hardware)(*out)
                self._export(sample, now)
            except Exception as exc:
                with self._lock:
                    self._error = f'{type(exc).__name__}: {exc}'[:300]
                    self._finish_park('failed', self._error)
                return
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()
