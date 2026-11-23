"""What the table-driven arms share: joints <-> motors, limits, calibration, the safe ways in and out.

The arm itself is data (data/arms/<id>.json, `options`): one row per motor. A motor family (damiao, robstride, feetech,
dynamixel) only says how to talk to its motors, through the hooks at the bottom of MotorArm.

The table has one row per arm joint, as many as the arm has. A row can also be
    {"fixed": 0.0}      a joint that is not driven: it reads that value and nothing is sent
and motors listed under `held` stay at the pose measured at connect.

Positions in a family's hooks are radians in the motor's own frame: joint = sign * (motor - zero - whole turns).
"""
import math
import threading
import time
from dataclasses import dataclass, field

from . import calibrate as stored
from .base import ArmHardwareError, StreamedArm

TWO_PI = 2 * math.pi


@dataclass
class Reading:
    pos: float
    vel: float = 0.0
    effort: float = 0.0                # the family's unit (N m, mA or a load fraction); see Motor.effort_full
    temp: float | None = None
    fault: str | None = None
    enabled: bool | None = None


@dataclass
class Motor:
    name: str
    id: int
    model: str = ''
    role: str = 'joint'                # joint | gripper | held
    sign: float = 1.0
    zero: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    lo: float = -math.pi               # joint frame (gripper: motor frame)
    hi: float = math.pi
    closed: float | None = None        # gripper, motor frame
    open: float | None = None
    effort_full: float = 1.0           # gripper: the effort that reads as 1.0
    turn: float = 0.0                  # whole turns found at connect (a power cycle can reset the motor's turn count)
    spec: dict = field(default_factory=dict)

    def to_joint(self, raw): return self.sign * (raw - self.zero - self.turn)
    def to_motor(self, q): return self.zero + self.turn + self.sign * q
    def to_fraction(self, raw):
        return max(0.0, min(1.0, (raw - self.turn - self.closed) / (self.open - self.closed)))
    def from_fraction(self, jaw): return self.turn + self.closed + jaw * (self.open - self.closed)


def _motor(row, role, limits=None):
    known = {k: row[k] for k in ('model', 'sign', 'zero', 'kp', 'kd', 'closed', 'open', 'effort_full') if k in row}
    m = Motor(name=row['name'], id=int(row['id']), role=role, spec=dict(row), **known)
    if limits is not None:
        m.lo, m.hi = float(limits[0]), float(limits[1])
    elif row.get('limits'):
        m.lo, m.hi = (float(v) for v in row['limits'])
    return m


class MotorArm(StreamedArm):
    multi_turn = True                  # the motor counts past one turn, so a whole-turn offset can be corrected
    can_power_off = True

    def __init__(self, bus=None, side='left', home=None, limits=None, *, arm_id='arm', joints=(), gripper=None, held=(),
                 calibration='none', limits_by_side=None, limit_grace=0.1, gripper_open_rad=None, gripper_closed_rad=None,
                 **options):
        limits = (limits_by_side or {}).get(side, limits)
        options.setdefault('ramp_s', 0.5)
        super().__init__(bus=bus, side=side, home=home, limits=limits, **options)
        n = len(joints)
        if n < 3 or len(self.limits) != n or len(self.home) != n or gripper is None:
            raise ValueError(f'{arm_id}: the table needs a row, a limit and a home value per joint ({n} joints, '
                             f'{len(self.limits)} limits, {len(self.home)} home values) and a gripper')
        self.arm_id, self.calibration, self.limit_grace = arm_id, calibration, float(limit_grace)
        self.joints = [None if 'fixed' in row else _motor(row, 'joint', lim) for row, lim in zip(joints, self.limits)]
        self.fixed = [float(row.get('fixed', 0.0)) for row in joints]
        self.gripper = _motor(gripper, 'gripper')
        self._jaw_stops = (gripper_closed_rad, gripper_open_rad)
        self._apply_jaw_stops()
        self.held = [_motor(row, 'held') for row in held]
        self.motors = [m for m in self.joints if m is not None] + [self.gripper] + self.held
        self._io = threading.RLock()
        self._last = {}
        self._held_at = {}
        self._engaged = self._soft = False
        self._ramp_steps = 0
        self._adopted = set()

    def _apply_jaw_stops(self):
        closed, opened = self._jaw_stops
        if closed is not None: self.gripper.closed = float(closed)
        if opened is not None: self.gripper.open = float(opened)

    # -- what a motor family implements --------------------------------------------------------------------------
    def _open_bus(self): raise NotImplementedError
    def _close_bus(self): raise NotImplementedError
    def _probe(self):
        """Read-only: ({motor name: facts}, [problems]). Missing and wrong motors are problems."""
        raise NotImplementedError
    def _read_states(self):
        """{motor name: Reading} for every motor; raises when a motor stops answering."""
        raise NotImplementedError
    def _engage(self, states):
        """Torque on without motion: every motor keeps the measured position."""
        raise NotImplementedError
    def _command(self, targets, ramp):
        """targets: [(Motor, motor-frame radians)]; ramp 0..1 eases the stiffness in."""
        raise NotImplementedError
    def _soften(self):
        """Stop holding firmly without dropping the arm."""
        raise NotImplementedError
    def _torque_off(self):
        raise NotImplementedError
    def _on_zero_pose(self, raw):
        """Calibration: the arm is at its zero pose. Returns (facts to store, positions to use as zero)."""
        return {}, raw
    def _check_calibration(self, data):
        return []
    def _counts_turns(self, motor):
        return self.multi_turn

    # -- the StreamedArm side ------------------------------------------------------------------------------------
    def open_hardware(self):
        data = self._load_calibration()
        self._open_bus()
        try:
            _, problems = self._probe()
            problems = problems + (self._check_calibration(data) if data else [])
            if problems:
                raise ArmHardwareError(f'{self.arm_id} at {self.bus}: ' + '; '.join(problems))
            states = self._read_states()
            self._check_faults(states)
            self._resolve_turns(states)
            self._check_cool_enough(states)
            self._held_at = {m.name: states[m.name].pos for m in self.held}
            self._adopted = {m.name for m in self.motors if self._is_powered(m, states[m.name])}
            self.adopted = sorted(self._adopted)
            if self._adopted and self.powered_at_connect == 'refuse':
                raise ArmHardwareError(f'{self.arm_id} at {self.bus}: {", ".join(self.adopted)} still powered by an earlier '
                                       'session (powered_at_connect: refuse). Support the arm and power-cycle it')
            self._engage(states)
            self._engaged, self._soft, self._ramp_steps = True, False, 0
        except BaseException:
            self._close_bus()
            raise

    def open_dry(self):
        """Everything open_hardware checks, and nothing it writes."""
        data = self._load_calibration()
        self._open_bus()
        try:
            _, problems = self._probe()
            problems = problems + (self._check_calibration(data) if data else [])
            if problems:
                raise ArmHardwareError(f'{self.arm_id} at {self.bus}: ' + '; '.join(problems))
            states = self._read_states()
            self._check_faults(states)
            self._resolve_turns(states)
            self._engaged, self._soft = False, False
        except BaseException:
            self._close_bus()
            raise

    def _is_powered(self, motor, reading):
        return reading.enabled is True

    def _check_cool_enough(self, states):
        guard = self._thermal
        for index, (motor, limit) in enumerate(zip(self._thermal_order(), guard.stop)):
            temp = None if motor is None else states[motor.name].temp
            if temp is not None and temp >= limit:
                raise ArmHardwareError(f'{self.arm_id}: {motor.name} is at {temp:.0f} C (stop limit {limit:.0f} C): '
                                       'refusing to enable until it has cooled')

    def _thermal_order(self):
        return list(self.joints) + [self.gripper]

    def thermal_motors(self):
        rows = [(m.name, m.model, 'joint') if m is not None else (f'joint{i + 1}', '', 'absent') for i, m in enumerate(self.joints)]
        return rows + [(self.gripper.name, self.gripper.model, 'gripper')]

    def _jaw_scale(self):
        return self.gripper_effort * (self.gripper_hold_effort if self._jaw_guard.state == 'gripping' else 1.0)

    def read_hardware(self):
        states = self._read_states()
        self._check_faults(states)
        self._last = states
        q, velocity, temps = [], [], []
        for motor, fixed in zip(self.joints, self.fixed):
            state = None if motor is None else states[motor.name]
            q.append(fixed if motor is None else motor.to_joint(state.pos))
            velocity.append(0.0 if motor is None else motor.sign * state.vel)
            temps.append(None if motor is None else state.temp)
        jaw = states[self.gripper.name]
        temps.append(jaw.temp)
        out = dict(positions_rad=q, gripper=self.gripper.to_fraction(jaw.pos), velocities_rad_s=velocity,
                   gripper_effort=min(1.0, abs(jaw.effort) / (self.gripper.effort_full * max(self.gripper_effort, 1e-3))))  # 1 = this arm's force limit
        known = [t for t in temps if t is not None]
        if known:                                          # a joint without a motor reports the hottest real one
            out['temperatures_c'] = [max(known) if t is None else t for t in temps]
        return out

    def write_hardware(self, positions_rad, gripper):
        with self._io:
            with self._lock:
                if self._limp:                             # a release arrived while this cycle was computing
                    return
            if self._soft:
                self._soft, self._ramp_steps = False, 0
                self._adopted = set()
            targets = []
            for motor, q in zip(self.joints, positions_rad):
                if motor is not None:
                    q = max(motor.lo - self.limit_grace, min(motor.hi + self.limit_grace, float(q)))
                    targets.append((motor, motor.to_motor(q)))
            jaw = max(0.0, min(1.0 - self.gripper_open_margin, float(gripper)))
            targets.append((self.gripper, self.gripper.from_fraction(jaw)))
            targets += [(m, self._held_at[m.name]) for m in self.held]
            self._ramp_steps += 1
            self._command(targets, min(1.0, self._ramp_steps / max(1.0, self.ramp_s * self.rate_hz)))

    def limp_hardware(self):
        with self._io:
            self._soft = True
            self._soften()

    def motors_off_hardware(self):
        """Stiffness goes first and the arm is watched: only an arm its rest pose really carries is switched off."""
        if not self._engaged:
            return 'the motors were not on'
        with self._io:
            self._soft = True
            self._soften()
            before = {name: reading.pos for name, reading in self._read_states().items()}
            end = time.monotonic() + 0.5
            drift = 0.0
            while time.monotonic() < end:
                states = self._read_states()
                drift = max(drift, *(abs(states[name].pos - pos) for name, pos in before.items()))
                time.sleep(0.01)
            if drift > self.park_sag_rad:
                return f'the arm moved {drift:.3f} rad without stiffness: this rest pose does not carry it, motors left on'
            self._torque_off()
            self._engaged = False
            still_on = self._motors_still_on()
            if still_on:
                self._engaged = True
                return f'{", ".join(still_on)} did not switch off'
        return None

    def _motors_still_on(self):
        states = self._read_states()
        return [m.name for m in self.motors if states[m.name].enabled is True]

    def force_motors_off(self):
        with self._io:
            self._torque_off()
            self._engaged = False

    def switch_off(self):
        self._open_bus()
        try:
            self._torque_off()
            return self._motors_still_on()
        finally:
            self._close_bus()

    def close_hardware(self):
        with self._io:
            try:
                if self._engaged:
                    self._soft = True
                    self._soften()
            finally:
                self._engaged = False
                self._close_bus()

    def survey(self):
        out = dict(arm=self.arm_id, address=self.bus, ok=False, enabled_anything=False, motors={}, problems=[],
                   calibration=self._calibration_status())
        try:
            self._open_bus()
        except Exception as exc:
            out['problems'].append(f'cannot open {self.bus}: {exc}')
            return out
        try:
            out['motors'], problems = self._probe()
            out['problems'] += problems
            if not problems:
                states = self._read_states()
                for motor in self.motors:
                    s = states[motor.name]
                    out['motors'][motor.name].update(position_rad=round(s.pos, 4), temperature_c=s.temp, fault=s.fault)
                    if s.enabled is not None:
                        out['motors'][motor.name]['torque_on'] = s.enabled
                    if s.fault: out['problems'].append(f'{motor.name}: {s.fault}')
        except Exception as exc:
            out['problems'].append(f'{type(exc).__name__}: {exc}')
        finally:
            self._close_bus()
        out['ok'] = not out['problems']
        return out

    # -- connect-time checks -------------------------------------------------------------------------------------
    def _check_faults(self, states):
        for motor in self.motors:
            if states[motor.name].fault:
                raise ArmHardwareError(f'{motor.name} (id {motor.id}): {states[motor.name].fault}')

    def _resolve_turns(self, states):
        problems = []
        for motor in self.motors:
            turns = (0, 1, -1, 2, -2) if self._counts_turns(motor) else (0,)
            raw = states[motor.name].pos
            if motor.role == 'gripper':
                lo, hi = sorted((motor.closed, motor.open))
                candidates = [k for k in turns if lo - .3 <= raw - k * TWO_PI <= hi + .3]
                shown = raw
            else:
                lo, hi = motor.lo, motor.hi
                candidates = [k for k in turns if lo - self.limit_grace <= motor.sign * (raw - motor.zero - k * TWO_PI)
                              <= hi + self.limit_grace]
                shown = motor.sign * (raw - motor.zero)
            if candidates:
                motor.turn = candidates[0] * TWO_PI
            else:
                problems.append(f'{motor.name} reads {shown:+.3f} rad, outside [{lo:.3f}, {hi:.3f}]')
        if problems:
            raise ArmHardwareError(f'{self.arm_id}: refusing to enable, the arm is not where its zero says it is ('
                                   + '; '.join(problems) + f'). Check the pose, or run `roborun hw calibrate '
                                   f'{self.arm_id}@{self.bus}`')

    # -- calibration ---------------------------------------------------------------------------------------------
    def _needs_file(self):
        return self.calibration in ('required', 'gripper')

    def _calibration_status(self):
        if self.calibration == 'none':
            return 'not needed'
        if stored.load(self.arm_id, self.bus):
            return 'stored'
        return 'missing' if self._needs_file() else 'optional, using the table'

    def _load_calibration(self):
        if self.calibration == 'none':
            return None
        data = stored.load(self.arm_id, self.bus)
        if data is None:
            if self._needs_file():
                raise ArmHardwareError(f'{self.arm_id} at {self.bus} has no calibration: move it by hand once with '
                                       f'`roborun hw calibrate {self.arm_id}@{self.bus}`')
            return None
        ids = {m.name: m.id for m in self.motors}
        if data.get('motors') != ids:
            raise ArmHardwareError(f'the calibration in {stored.path(self.arm_id, self.bus)} is for motors '
                                   f'{data.get("motors")}, this arm has {ids}: run roborun hw calibrate again')
        self.gripper.closed, self.gripper.open = float(data['gripper']['closed']), float(data['gripper']['open'])
        self._apply_jaw_stops()
        for index, motor in enumerate(self.joints):
            row = (data.get('joints') or {}).get(motor.name) if motor is not None else None
            if row is None:
                continue
            motor.zero = float(row['zero'])
            motor.lo, motor.hi = max(motor.lo, float(row['lo'])), min(motor.hi, float(row['hi']))
            self.limits[index] = (motor.lo, motor.hi)
            self.home[index] = max(motor.lo, min(motor.hi, self.home[index]))
        for motor in self.held:
            row = (data.get('joints') or {}).get(motor.name)
            if row is not None:
                motor.zero = float(row['zero'])
        return data

    def calibration_open(self):
        self._open_bus()
        _, problems = self._probe()
        if problems:
            self._close_bus()
            raise ArmHardwareError(f'{self.arm_id} at {self.bus}: ' + '; '.join(problems))
        return self._read_states()

    def calibration_read(self):
        return {name: s.pos for name, s in self._read_states().items()}

    def calibration_close(self):
        self._close_bus()
