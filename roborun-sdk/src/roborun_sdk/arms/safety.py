"""The guards every real arm runs, and the names of the settings that tune them.

Everything here is a local setting: `motion:` (every arm) or `arms.<side>:` (one arm) in robot.yaml. The defaults make the
file optional. A name that is not listed here and that the driver does not take is reported, never refused.

Temperature limits, where each number comes from:
  Damiao DM-J motors (YAM and others)  YAM: new work is refused from 70 C and work stops at 80 C; a shoulder motor
                                       cut out in its own firmware at 101 C while holding a cup at full reach.
  MakerMods Metal                      Metal owner: enable refused at 55 C (jaw 45 C), hold at 65 C (jaw 55 C).
  ROBOTIS Dynamixel                    e-Manual, Temperature Limit (address 31): XL330 70, XL430 72, XM430 and XM540 80 C.
                                       Stop 5 C under it, warn 15 C under it.
  Feetech STS3215                      product specification: torque off above 70 C. Same margins.
  RobStride RS00, RS02                 joints: no maker figure found, treated like the Damiao joints until measured.
                                       Maker Arm gripper (RS00): the maker's limit, enable refused at 45 C, stop at 55 C.
  anything else                        warn 60 C, stop 70 C: the lowest firmware limit in this table.
"""
import fcntl
import json
import math
import os
import tempfile
import threading
import time
from hashlib import sha256
from pathlib import Path

GENERIC_TEMPS = (60.0, 70.0)
_DAMIAO = (70.0, 80.0)
MOTOR_TEMPS = {
    'dm4310': _DAMIAO, 'dm4340': _DAMIAO, 'dm4340p': _DAMIAO, 'dm6248': _DAMIAO, 'dm3507': _DAMIAO, 'dm8009': _DAMIAO,
    'rs00': _DAMIAO, 'rs02': _DAMIAO,
    'metal': (55.0, 65.0),
    'sts3215': (55.0, 65.0),
    'xl330-m077': (55.0, 65.0), 'xl330-m288': (55.0, 65.0), 'xl430-w250': (57.0, 67.0),
    'xm430-w350': (65.0, 75.0), 'xm540-w270': (65.0, 75.0),
}
JAW_TEMPS = {'metal': (45.0, 55.0), 'rs00': (45.0, 55.0)}
TEMP_STOP_AFTER_S = 0.5             # one bad frame never stops an arm
TEMP_CLEAR_MARGIN_C = 3.0

POSITIVE = {
    'rate_hz': 'stream rate to the motors', 'joint_speed_rad_s': 'joint speed limit', 'joint_accel_rad_s2': 'joint acceleration limit',
    'joint_jerk_rad_s3': 'joint jerk limit', 'stop_accel_rad_s2': 'joint deceleration on Stop (default: twice the acceleration limit)',
    'stop_jerk_rad_s3': 'joint jerk on Stop (default: three times the jerk limit)', 'jaw_speed': 'jaw speed, fraction per second', 'max_jaw_step': 'largest jaw change per stream cycle',
    'command_timeout_s': 'commands older than this end in a hold', 'sample_timeout_s': 'samples older than this are refused',
    'stall_error_rad': 'tracking error that counts as blocked', 'stall_velocity_rad_s': 'below this speed a joint counts as not moving',
    'stall_time_s': 'how long a joint may stay blocked', 'jaw_stall_time_s': 'how long the jaw may push without moving',
    'jaw_stall_error': 'jaw error (fraction) that counts as pushing', 'park_speed_rad_s': 'joint speed while parking',
    'park_timeout_s': 'how long parking may take', 'park_tolerance_rad': 'how close to the rest pose counts as parked',
    'park_sag_rad': 'movement that shows the rest pose does not carry the arm',
}
FRACTIONS = {
    'gripper_effort': 'share of the jaw motor force to use', 'gripper_force_cap': 'effort reading at which a still jaw counts as gripping',
    'gripper_hold_effort': 'share of gripper_effort kept once gripping', 'gripper_open_margin': 'how far the open command stays off the open stop',
}
# The robot software tells a held object from a free jaw by the effort it reads, on the embodiment's 0.5 scale: up to .18 is what a free
# jaw shows, so a hold must read above it. This share of the jaw force reads .20.
GRIP_HOLD_MIN = 0.4
NON_NEGATIVE = {'ramp_s': 'stiffness ramp after enabling'}
FLAGS = {'stall_guard': 'stop a blocked joint', 'jaw_guard': 'stop a jaw pushing without moving', 'dry_run': 'read only, log what would be sent'}
OTHER = {
    'max_tool_speed_m_s': 'tool speed limit, needs the arm model', 'temp_warn_c': 'warn at this temperature: a number or one per motor',
    'temp_stop_c': 'stop at this temperature: a number or one per motor', 'rest_pose': 'joint angles the arm can rest at without power',
    'home_before_motors_off': 'park before switching off (default: when rest_pose is set)',
    'powered_at_connect': 'an arm found powered: hold (default) or refuse', 'tool_model': '{xml, site}: the model for the tool pose',
    'temperature_export': 'append temperatures to this file as JSON lines',
}
KNOWN = set(POSITIVE) | set(FRACTIONS) | set(NON_NEGATIVE) | set(FLAGS) | set(OTHER) | {'rules'}


def number(name, value, low=None, high=None, above=None):
    try:
        out = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{name} must be a number, not {value!r}') from None
    if not math.isfinite(out) or (above is not None and out <= above) or (low is not None and out < low) or (high is not None and out > high):
        wanted = 'a positive number' if above == 0 else 'at least 0' if high is None else f'between {low:g} and {high:g}'
        raise ValueError(f'{name} must be {wanted}, not {value!r}')
    return out


def temperature_limits(model, role='joint'):
    name = str(model or '').lower()
    if role == 'absent':                               # a joint this arm does not have
        return (math.inf, math.inf)
    if role == 'gripper' and name in JAW_TEMPS:
        return JAW_TEMPS[name]
    return MOTOR_TEMPS.get(name, GENERIC_TEMPS)


class ThermalGuard:
    """Per motor: ok, warn, stop. `motors`: [(name, model, role)], one per entry of temperatures_c."""

    def __init__(self, motors, warn=None, stop=None):
        self.motors = list(motors)
        defaults = [temperature_limits(model, role) for _, model, role in self.motors]
        self.warn = self._levels('temp_warn_c', warn, [d[0] for d in defaults])
        self.stop = self._levels('temp_stop_c', stop, [d[1] for d in defaults])
        self.state, self.hottest, self.motor, self.temperatures = 'ok', None, None, None
        self._over_since = None

    def _levels(self, name, value, defaults):
        if value is None:
            return defaults
        if isinstance(value, (list, tuple)):
            if len(value) != len(defaults):
                raise ValueError(f'{name} has {len(value)} values, this arm has {len(defaults)} motors')
            return [number(name, v, above=0) for v in value]
        return [number(name, value, above=0)] * len(defaults)

    def _name(self, index):
        return self.motors[index][0] if index < len(self.motors) else f'motor {index + 1}'

    def update(self, temperatures, now, settled=False):
        """Returns 'warn' or 'stop' when the state just got worse, else None. `settled`: no waiting (the state at connect)."""
        rows = [(float(t), i) for i, t in enumerate(temperatures or ())
                if t is not None and i < len(self.stop) and math.isfinite(self.stop[i])]
        if not rows:
            return None
        self.temperatures = [None if t is None else float(t) for t in temperatures]
        self.hottest, index = max(rows)
        self.motor = self._name(index)
        over = [(t, i) for t, i in rows if t >= self.stop[i]]
        if over:
            self.hottest, index = max(over)
            self.motor = self._name(index)
            self._over_since = now if self._over_since is None else self._over_since
            if self.state != 'stop' and (settled or now - self._over_since >= TEMP_STOP_AFTER_S):
                self.state = 'stop'
                return 'stop'
            return None
        self._over_since = None
        if self.state == 'stop':
            return None                                # latched: Reset clears it once the motor has cooled
        if any(t >= self.warn[i] for t, i in rows):
            if self.state == 'ok':
                self.state = 'warn'
                return 'warn'
        elif self.state == 'warn' and all(t < self.warn[i] - TEMP_CLEAR_MARGIN_C for t, i in rows):
            self.state = 'ok'
        return None

    def cooled(self):
        rows = [(t, i) for i, t in enumerate(self.temperatures or ()) if t is not None and i < len(self.stop)]
        return all(t < self.stop[i] - TEMP_CLEAR_MARGIN_C for t, i in rows)

    def clear(self):
        self.state, self._over_since = 'ok', None

    def report(self):
        out = dict(state=self.state, hottest_c=self.hottest, motor=self.motor)
        if self.temperatures is not None:
            out['temperatures_c'] = self.temperatures
        return out


class StallGuard:
    """A joint told to be elsewhere that is not moving, for longer than `hold_s`."""

    def __init__(self, joints, error, velocity, hold_s):
        self.error, self.velocity, self.hold_s = error, velocity, hold_s
        self.since = [None] * joints

    def reset(self):
        self.since = [None] * len(self.since)

    def update(self, commanded, measured, velocities, now):
        for i, (c, m, v) in enumerate(zip(commanded, measured, velocities)):
            if abs(c - m) > self.error and abs(v) < self.velocity:
                self.since[i] = now if self.since[i] is None else self.since[i]
                if now - self.since[i] >= self.hold_s:
                    return i
            else:
                self.since[i] = None
        return None


class JawGuard:
    """A jaw that pushes and does not move stops pushing: on an object it keeps a chosen squeeze, at a stop it keeps none."""
    STILL = 0.005                                      # fraction of the stroke
    NEAR_CLOSED = 0.03

    def __init__(self, error, hold_s, force_cap):
        self.error, self.hold_s, self.force_cap = error, hold_s, force_cap
        self.reset()

    def reset(self):
        self.state, self.held, self.closing = 'free', None, False
        self._ref, self._ref_at = None, None

    def update(self, target, shaped, measured, effort, now, squeeze):
        """The jaw command to send."""
        if self.state != 'free':
            away = target > self.held + self.error if self.closing else target < self.held - self.error
            if away:
                self.reset()
                return shaped if abs(shaped - measured) < self.error else measured
            return max(0.0, self.held - squeeze) if self.state == 'gripping' else self.held
        pushing = abs(shaped - measured) > self.error
        pressed = effort is not None and effort >= self.force_cap and target < measured
        # only time spent pushing counts: a jaw that sat idle for a minute has not been blocked for a minute
        if self._ref is None or abs(measured - self._ref) > self.STILL or not (pushing or pressed):
            self._ref, self._ref_at = measured, now
        still_for = now - self._ref_at
        if (pushing and still_for >= self.hold_s) or (pressed and still_for >= self.hold_s / 3):
            self.closing = target < measured
            self.held = measured
            self.state = 'gripping' if self.closing and measured > self.NEAR_CLOSED else 'at_stop'
            return self.update(target, shaped, measured, effort, now, squeeze)
        return shaped


class ToolModel:
    """Tool pose from an arm model (MJCF). Needs mujoco; a caller treats any failure here as 'no model'."""

    def __init__(self, xml, joints, site='grasp_site'):
        import mujoco
        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(Path(xml).expanduser()))
        self.data = mujoco.MjData(self.model)
        self.joints = joints
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site)
        if self.site < 0:
            raise ValueError(f'{xml} has no site {site!r}')
        if self.model.nq < joints:
            raise ValueError(f'{xml} has {self.model.nq} coordinates, the arm has {joints} joints')
        self._lock = threading.Lock()

    def position(self, q):
        with self._lock:
            self.data.qpos[:self.joints] = q
            self._mj.mj_kinematics(self.model, self.data)
            return [float(v) for v in self.data.site_xpos[self.site]]

    def pose(self, q):
        """4x4 nested list, in the model's base frame."""
        with self._lock:
            self.data.qpos[:self.joints] = q
            self._mj.mj_kinematics(self.model, self.data)
            r, p = self.data.site_xmat[self.site], self.data.site_xpos[self.site]
            return [[float(r[0]), float(r[1]), float(r[2]), float(p[0])], [float(r[3]), float(r[4]), float(r[5]), float(p[1])],
                    [float(r[6]), float(r[7]), float(r[8]), float(p[2])], [0.0, 0.0, 0.0, 1.0]]


class ArmsBusy(RuntimeError):
    pass


def _home():
    from .. import robot
    return Path(os.environ.get('ROBORUN_HOME') or robot.HOME)


class ArmsLock:
    """Only one program drives this robot's arms: one lock per Roborun home, and one per arm address on this computer
    (two homes can name the same adapter). The operating system drops the locks when the holder dies."""

    def __init__(self, what, addresses=()):
        self.what, self.files = what, []
        machine = Path(tempfile.gettempdir()) / f'roborun-arms-{os.getuid()}'
        self.paths = [_home() / 'run' / 'arms.lock']
        for address in sorted({str(a) for a in addresses if a}):
            self.paths.append(machine / (sha256(address.encode()).hexdigest()[:24] + '.lock'))

    def acquire(self):
        try:
            for path in self.paths:
                path.parent.mkdir(parents=True, exist_ok=True)
                handle = open(path, 'a+')
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError:
                    handle.seek(0)
                    holder = handle.read().strip()
                    handle.close()
                    raise ArmsBusy(self._busy_text(holder)) from None
                handle.seek(0); handle.truncate()
                handle.write(json.dumps(dict(pid=os.getpid(), what=self.what, since=time.time())))
                handle.flush()
                self.files.append(handle)
        except BaseException:
            self.release()
            raise
        return self

    @staticmethod
    def _busy_text(holder):
        try:
            row = json.loads(holder)
            who = f'{row["what"]} (process {row["pid"]})'
        except (ValueError, KeyError, TypeError):
            who = 'another Roborun program'
        return f'the arms are in use by {who}: stop it first (roborun down)'

    def release(self):
        for handle in self.files:
            try:
                handle.close()
            except OSError:
                pass
        self.files = []

    def __enter__(self):
        return self.acquire()

    def __exit__(self, *_):
        self.release()
