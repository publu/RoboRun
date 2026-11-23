"""YAM-family arms on Damiao motors, driven through the vendored i2rt stack (third_party/i2rt).

The joints use i2rt's convention, which is also Roborun's YAM convention: zero is the folded rest pose and every sign
is the arm model's. Nothing moves at open: the motors come up in gravity compensation with the gravity torque present
from the first CAN cycle, and StreamedArm then holds the measured pose.

The jaw is the one thing i2rt would move at startup: its linear grippers have more than one motor turn of stroke, so
i2rt sweeps them against both stops every launch. Here that sweep is a separate, deliberate step, done once per
gripper with the jaws empty, and remembered:

    roborun hw calibrate --gripper yam@slcan://SERIAL

A teaching handle (yam_teaching_handle_v1/_v2) is an end with no gripper: the arm runs as with no_gripper, and read()
adds `handle` (trigger, encoder angle, buttons), read-only. Leader-follower teleoperation is not part of this driver.
"""
import argparse
import contextlib
import json
import math
import os
import re
import struct
import sys
import tempfile
import time
from functools import partial
from importlib import resources
from pathlib import Path

import numpy as np

from . import canbus
from . import safety
from .base import ArmHardwareError, StreamedArm

THIRD_PARTY = Path(__file__).resolve().parents[1] / 'third_party'
if str(THIRD_PARTY) not in sys.path:
    sys.path.insert(0, str(THIRD_PARTY))

import i2rt  # noqa: E402

if Path(i2rt.__file__).resolve().parent != THIRD_PARTY / 'i2rt':
    raise ImportError(f'another i2rt is already loaded from {i2rt.__file__}', name='i2rt')

from i2rt.motor_config_tool.dm_motor_registers import read_register  # noqa: E402
from i2rt.motor_config_tool.utils import RawCanInterface  # noqa: E402
from i2rt.motor_drivers import can_interface  # noqa: E402
from i2rt.motor_drivers.dm_driver import ControlMode, DMSingleMotorCanInterface  # noqa: E402
from i2rt.motor_drivers.motor_check import run_startup_checks  # noqa: E402
from i2rt.motor_drivers.utils import MotorType  # noqa: E402
from i2rt.motor_drivers.dm_driver import CONTROL_FREQ, PassiveEncoderInfo  # noqa: E402
from i2rt.robots import get_robot as _i2rt_get_robot  # noqa: E402
from i2rt.robots.get_robot import _yam_motor_list, get_yam_robot  # noqa: E402
from i2rt.robots.utils import (ArmType, GripperType, _load_arm_config, _load_gripper_config,  # noqa: E402
                               combine_arm_and_gripper_xml)

_BUSES = {}                         # channel -> the buses i2rt opened through us: a start that fails midway leaves its own open


def _open_for_i2rt(channel, bitrate):
    bus = canbus.open_bus(channel, bitrate)
    _BUSES.setdefault(channel, []).append(bus)
    return bus


def _close_left_open(channel):
    for bus in _BUSES.pop(channel, ()):
        try:
            bus.shutdown()
        except Exception:
            pass


can_interface.BUS_OPENER = _open_for_i2rt

# -- the teaching handle: read-only ------------------------------------------------------------------------------------
# The handle has no motor: a passive encoder on the arm's bus reports the trigger and two buttons when asked (request
# 0x02 on 0x50e, report on 0x50f; i2rt's PassiveEncoderReader). i2rt's own handle chain first validates the encoder and
# rewrites its report and ADC rates, over a socketcan-only side socket. Roborun only ever sends the report request,
# through the arm's own bus, at most every HANDLE_POLL_S, and a handle that stops answering never stops the arm.
HANDLE_REQUEST_ID, HANDLE_REPORT_ID = 0x50E, 0x50F
HANDLE_POLL_S = 0.02
HANDLE_RANGE_RAD = 0.7              # trigger travel, as i2rt's reader normalizes it
HANDLE_GIVE_UP = 5                  # misses in a row before the handle is reported unavailable and no longer asked
_HANDLE_PRESENT = {}                # channel -> whether its handle answered before the motors started


def handle_report(iface):
    """One report from the handle on this interface's bus, or None. Sends only the report request."""
    try:
        msg = iface._send_message_get_response(HANDLE_REQUEST_ID, HANDLE_REQUEST_ID, [0xFF, 0x02], max_retry=1,
                                               expected_id=HANDLE_REPORT_ID)
    except AssertionError:
        return None
    _device, position, velocity, inputs = struct.unpack('!BhhB', bytes(msg.data))
    angle = position * 2 * math.pi / 4096
    info = PassiveEncoderInfo(id=HANDLE_REQUEST_ID, position=min(abs(angle), HANDLE_RANGE_RAD) / HANDLE_RANGE_RAD,
                              velocity=velocity * 2 * math.pi / 4096, io_inputs=[bool(inputs & 1), bool(inputs & 2)])
    info.angle_rad = angle
    return info


class HandleReader:
    """Stands in for i2rt's encoder chain inside its motor loop: read_states() never raises, never blocks long, and always
    returns the one report i2rt expects; a report that is not fresh carries stale=True and reads as unavailable."""

    def __init__(self, iface):
        self.iface, self.asked, self.at, self.misses = iface, True, 0.0, 0
        self.last = PassiveEncoderInfo(id=HANDLE_REQUEST_ID, position=0.0, velocity=0.0, io_inputs=[False, False])
        self.last.angle_rad, self.last.stale = 0.0, True

    def read_states(self):
        now = time.monotonic()
        if self.asked and now - self.at >= HANDLE_POLL_S:
            self.at = now
            try:
                report = handle_report(self.iface)
            except Exception:
                report = None
            if report is None:
                self.misses += 1
                if self.misses >= HANDLE_GIVE_UP:
                    self.asked = False
                    self.last.stale = True
            else:
                report.stale, self.last, self.misses = False, report, 0
        return [self.last]


def _handle_chain(iface):
    """i2rt asks for its handle chain here. A handle that did not answer before the motors started gets none at all."""
    if not _HANDLE_PRESENT.get(getattr(iface, 'channel', None), True):
        return None
    return HandleReader(iface)


_i2rt_get_robot.get_encoder_chain = _handle_chain


def handle_state(report):
    """What read() says about the handle: trigger 0 at rest .. 1 squeezed, the raw encoder angle, the two buttons."""
    if report is None or getattr(report, 'stale', False):
        return dict(available=False)
    return dict(available=True, trigger=round(float(report.position), 4), angle_rad=round(float(report.angle_rad), 4),
                velocity_rad_s=round(float(report.velocity), 4), buttons=list(report.io_inputs))

JAW_MARGIN_RAD = 0.2                # how far past a calibrated stop a jaw reading is still believed
JAW_SAVE_EVERY_S = 1.0
SWEEP_TORQUE_NM = 0.5               # the torques and timings i2rt's own gripper calibration uses
SWEEP_MAX_S = 2.0
SWEEP_STEP_S = 0.05
SWEEP_STILL_RAD = 0.01
SWEEP_STILL_STEPS = 3
SURVEY_REGISTERS = ('sw_ver', 'Gr', 'CTRL_MODE', 'PMAX', 'VMAX', 'TMAX', 'TIMEOUT', 'xout')
SETTLE_STEPS = 10
SETTLE_S = 0.3
GRIP_FORCE_N = 50.0                 # the force limit i2rt's own YAM setup gives its gripper
FADE_STEPS = 20                     # gravity compensation fades out over this many steps before motors go off
FADE_STEP_S = 0.05
FADE_WATCH_STEPS = 10


def _state_dir() -> Path:
    return Path(os.environ.get('ROBORUN_HOME', Path.home() / '.config' / 'roborun')) / 'arms'


def _profile_home(side):
    rig = json.loads((resources.files('roborun_sdk') / 'data' / 'profiles' / 'yam.json').read_text())
    return rig['arms'][side]['home'] if side in rig['arms'] else None


def _raise_adapter_error(exc):
    """i2rt wraps what it catches; a problem with the adapter itself comes back out in its own words."""
    seen = set()
    while exc is not None and id(exc) not in seen:
        if isinstance(exc, canbus.AdapterError):
            raise exc
        seen.add(id(exc))
        exc = exc.__cause__ or exc.__context__


class YamArm(StreamedArm):
    feedback_timeout_s = 0.3
    can_power_off = True

    def __init__(self, bus=None, side='left', home=None, limits=None, arm_type='yam', gripper_type='linear_4310',
                 gripper_limits=None, payload_kg=None, gravity_comp_factor=None, gripper_open_rad=None,
                 gripper_closed_rad=None, gripper_force_n=None, **options):
        self.arm_type = ArmType.from_string_name(arm_type)
        self.gripper_type = GripperType.from_string_name(gripper_type)
        if self.arm_type == ArmType.NO_ARM:
            raise ValueError('YamArm drives an arm: arm_type no_arm has none')
        if str(bus or '').startswith(can_interface.ETHERCAT_CHANNEL_PREFIX):
            raise ArmHardwareError(f'{bus}: EtherCAT boards are not supported. Next: connect the arm through a CAN adapter '
                                   '(`roborun hw detect` lists them)')
        self.config = _load_arm_config(self.arm_type)
        if len(self.config.motor_list) != 6:
            raise ValueError(f'{arm_type} has {len(self.config.motor_list)} joints; Roborun arms have 6')
        if home is None and arm_type == 'yam':
            home = _profile_home(side)
        super().__init__(bus=bus, side=side, home=home, limits=limits, **options)
        self.handle = self.gripper_type.is_teaching_handle    # an end with no gripper: its encoder is only read
        self.has_jaw = self.gripper_type != GripperType.NO_GRIPPER and not self.handle
        self.gripper_limits = None if gripper_limits is None else [float(v) for v in gripper_limits]
        if (gripper_open_rad is None) != (gripper_closed_rad is None):
            raise ValueError('set gripper_open_rad and gripper_closed_rad together')
        if gripper_open_rad is not None:
            self.gripper_limits = [float(gripper_closed_rad), float(gripper_open_rad)]
        self.payload_kg = None if payload_kg is None else safety.number('payload_kg', payload_kg, 0.0)
        self.gravity_comp_factor = None
        if gravity_comp_factor is not None:
            joints = len(self.config.motor_list)
            values = [gravity_comp_factor] * joints if isinstance(gravity_comp_factor, (int, float)) else list(gravity_comp_factor)
            if len(values) != joints:
                raise ValueError(f'gravity_comp_factor has {len(values)} values, this arm has {joints} joints')
            self.gravity_comp_factor = np.array([safety.number('gravity_comp_factor', v, 0.0, 2.0) for v in values])
        self.gripper_force_n = GRIP_FORCE_N if gripper_force_n is None else safety.number('gripper_force_n', gripper_force_n, above=0)
        self.robot = None
        self._dry = None
        self._ramp_steps = 0
        self._full_grip_map = None
        self._grip_share = 1.0
        self._jaw_saved = (0.0, None)

    # -- the jaw's calibration ----------------------------------------------------------------------------------
    @property
    def calibration(self):                                      # what `roborun hw calibrate` asks every arm
        raise SystemExit('a YAM keeps its joint zeros in the motors; for the gripper run: roborun '
                         f'hw calibrate --gripper {self.arm_type.value}@{self.bus}')

    @property
    def _jaw_file(self) -> Path:
        name = re.sub(r'[^A-Za-z0-9._-]+', '_', f'{self.gripper_type.value}-{self.bus}')
        return _state_dir() / f'{name}.json'

    def _jaw_memory(self):
        try:
            row = json.loads(self._jaw_file.read_text())
            return row if math.isfinite(row['closed'] + row['open']) else None
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def _save_jaw(self, **fields):
        row = dict(self._jaw_memory() or {}, **fields)
        self._jaw_file.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile('w', dir=self._jaw_file.parent, delete=False, suffix='.tmp') as out:
            json.dump(row, out)
        os.replace(out.name, self._jaw_file)

    def _jaw_needs_stops(self):
        return self.has_jaw and self.gripper_type.get_gripper_limits(self.arm_type) is None

    def _jaw_stops(self):
        """[closed, open] in motor radians, or None when the gripper's config already fixes them."""
        if self.gripper_limits is not None and self.has_jaw:
            return self.gripper_limits
        if not self._jaw_needs_stops():
            return None
        row = self._jaw_memory()
        if row is None:
            raise ArmHardwareError(
                f'the {self.gripper_type.value} gripper on {self.bus} is not calibrated. With the jaws empty, run: '
                f'roborun hw calibrate --gripper {self.arm_type.value}@{self.bus} (only the jaw moves)')
        return [row['closed'], row['open']]

    def _align_jaw(self, stops):
        """After a restart the motor reads its position modulo one turn and the stroke is longer than that: pick the
        turn that lies inside the calibrated stroke; the two ends overlap, and there the last known position decides."""
        chain = self.robot.motor_chain
        reading = float(chain.read_states()[-1].pos)
        low, high = min(stops), max(stops)
        options = [reading + 2 * math.pi * k for k in range(-3, 4)
                   if low - JAW_MARGIN_RAD <= reading + 2 * math.pi * k <= high + JAW_MARGIN_RAD]
        if not options:
            raise ArmHardwareError(f'the jaw motor reads {reading:.3f} rad, outside its calibrated stroke '
                                   f'[{low:.3f}, {high:.3f}]: calibrate the gripper again')
        last = (self._jaw_memory() or {}).get('last')
        anchor = float(last) if last is not None else (low + high) / 2
        best = min(options, key=lambda v: abs(v - anchor))
        chain.motor_offset[-1] -= (best - reading) * chain.motor_direction[-1]
        expected = (best - stops[0]) / (stops[1] - stops[0])    # the first sample must already be in the right turn
        deadline = time.monotonic() + 1.0
        while abs(float(self.robot.get_observations()['gripper_pos'][0]) - expected) > 0.05:
            if time.monotonic() > deadline:
                raise ArmHardwareError('the jaw position did not settle after aligning it')
            time.sleep(0.002)

    # -- StreamedArm ---------------------------------------------------------------------------------------------
    def thermal_motors(self):
        rows = [(f'joint{i + 1}', kind, 'joint') for i, (_, kind) in enumerate(self.config.motor_list)]
        if self.has_jaw:
            rows.append(('gripper', self.gripper_type.get_motor_type(self.arm_type), 'gripper'))
        return rows

    def default_tool_model(self):
        return dict(xml=combine_arm_and_gripper_xml(self.arm_type, self.gripper_type), site='grasp_site')

    def _end_effector_mass(self):
        """The mass i2rt should use for the last link: what the model says, plus the payload."""
        if not self.payload_kg:
            return None
        import mujoco
        model = mujoco.MjModel.from_xml_path(combine_arm_and_gripper_xml(self.arm_type, self.gripper_type))
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'joint{len(self.config.motor_list)}')
        return float(model.body_mass[model.jnt_bodyid[joint]]) + self.payload_kg

    def can_frames_per_s(self):
        """i2rt's loop: one frame and its reply per motor at its control rate; the handle's request and report."""
        return 2 * len(self._motor_rows()) * CONTROL_FREQ + (2 / HANDLE_POLL_S if self.handle else 0)

    def _interface(self):
        return DMSingleMotorCanInterface(channel=self.bus, bustype='socketcan', name='roborun_read')

    def _motor_rows(self):
        return _yam_motor_list(self.arm_type, self.gripper_type)

    def _powered_motors(self):
        """Motors an earlier session left enabled. i2rt's start carries each motor's gravity torque from its first enabled
        moment, so these are never handed a zero-gain frame; this only reports them, or refuses when asked to."""
        iface = self._interface()
        powered = []
        try:
            for motor_id, kind in self._motor_rows():
                state = iface.read_state_while_disabled(motor_id, kind)
                code = int(str(state.error_code), 16)
                if code == 1:
                    powered.append(f'motor {motor_id}')
                elif code:                                      # enabling a faulted motor would never return
                    raise ArmHardwareError(f'motor {motor_id} reports a fault ({state.error_message}). Nothing was enabled. '
                                           'Next: let it cool or fix the cause, then power-cycle the arm')
        except AssertionError:                                  # a motor that ignores the status request: nothing to report
            pass
        finally:
            iface.close()
        return powered

    def _read_handle_once(self):
        iface = self._interface()
        try:
            return handle_state(handle_report(iface))
        finally:
            iface.close()

    def _start_robot(self, stops):
        if not self.bus:
            raise ArmHardwareError('a YAM needs a CAN address: yam@can0, yam@slcan://SERIAL or yam@gs_usb://SERIAL')
        try:
            with contextlib.redirect_stdout(sys.stderr):
                return get_yam_robot(
                    channel=self.bus, arm_type=self.arm_type, gripper_type=self.gripper_type, zero_gravity_mode=True,
                    gripper_limits_override=None if stops is None else np.asarray(stops, dtype=float),
                    ee_mass=self._end_effector_mass(), gravity_comp_factor=self.gravity_comp_factor,
                    gravity_hold_from_first_cycle=True)
        except ArmHardwareError:
            _close_left_open(self.bus)
            raise
        except Exception as exc:
            _close_left_open(self.bus)
            _raise_adapter_error(exc)
            raise ArmHardwareError(f'{type(exc).__name__}: {exc}') from exc

    def _set_grip_force(self, share=1.0):
        """i2rt's limiter sets the force of a blocked jaw, so the hold force is set here too, not through the jaw target."""
        limiter = self.robot._gripper_force_limiter
        if limiter is None:
            return
        force = self.gripper_force_n * self.gripper_effort * share
        if self._full_grip_map is None:
            self._full_grip_map = partial(limiter.gripper_force_torque_map, gripper_force=self.gripper_force_n * self.gripper_effort)
        limiter.max_force = self.robot._limit_gripper_force = force
        limiter.gripper_force_torque_map = partial(limiter.gripper_force_torque_map, gripper_force=force)
        self._grip_share = share

    def jaw_squeeze(self):
        robot = self.robot
        if robot is None or not self.has_jaw:
            return super().jaw_squeeze()
        stops = robot._gripper_limits
        torque = self._grip_torque(float(robot.motor_chain.read_states()[-1].pos)) * self.gripper_hold_effort
        return float(torque / robot._kp[-1] / abs(float(stops[1]) - float(stops[0])))

    def open_hardware(self):
        stops = self._jaw_stops()
        try:
            self.adopted = self._powered_motors()
        except ArmHardwareError:
            raise
        except Exception as exc:
            raise ArmHardwareError(f'{type(exc).__name__}: {exc}') from exc
        if self.adopted and self.powered_at_connect == 'refuse':
            raise ArmHardwareError(f'{", ".join(self.adopted)} still powered by an earlier session '
                                   '(powered_at_connect: refuse). Support the arm and power-cycle it')
        text = canbus.load_warning(self.bus, self.can_frames_per_s(), who=self.arm_type.value)
        if text:
            print(json.dumps(dict(event='can_bus_load', arm=self.side, message=text)), flush=True)
            self.setting_warnings = list(getattr(self, 'setting_warnings', ())) + [text]
        if self.handle:
            _HANDLE_PRESENT[self.bus] = self._read_handle_once()['available']
        self.robot = self._start_robot(stops)
        self._ramp_steps = 0
        self._full_grip_map = None
        try:
            self._set_grip_force()
            if stops is not None:
                self._align_jaw(stops)
            model = self.robot.get_robot_info()['joint_limits'][:6]        # i2rt refuses to run past these
            self.limits = [(max(lo, float(a) + 0.15), min(hi, float(b) - 0.15))
                           for (lo, hi), (a, b) in zip(self.limits, model)]
        except BaseException:
            self._stop_robot()
            raise

    def open_dry(self):
        """Reads the motors through the status request, which they answer in any state. Nothing is enabled or written."""
        if not self.bus:
            raise ArmHardwareError('a YAM needs a CAN address: yam@can0, yam@slcan://SERIAL or yam@gs_usb://SERIAL')
        try:
            run_startup_checks(self.bus, self._motor_rows(), check_motor_types=True, check_motor_config=True,
                               control_mode=ControlMode.MIT, repair=False, expected_registers=self.config.motor_registers)
            self._dry = dict(iface=self._interface(), turns={})
        except Exception as exc:
            raise ArmHardwareError(f'{type(exc).__name__}: {exc}') from exc
        try:
            stops = self._jaw_stops() if self.has_jaw else None
            if stops is None and self.has_jaw:
                stops = self.gripper_type.get_gripper_limits(self.arm_type)
        except ArmHardwareError:
            stops = None                                        # an uncalibrated jaw is reported as unavailable
        self._dry['stops'] = None if stops is None else [float(v) for v in stops]

    def _read_dry(self):
        dry, rows = self._dry, self._motor_rows()
        states = [dry['iface'].read_state_while_disabled(motor_id, kind) for motor_id, kind in rows]
        joints = len(self.config.motor_list)
        q, velocity = [], []
        for i, state in enumerate(states[:joints]):
            direction = float(self.config.directions[i])
            value = float(state.position) * direction
            turn = dry['turns'].setdefault(i, round(value / (2 * math.pi)) * 2 * math.pi)
            q.append(value - turn)
            velocity.append(float(state.velocity) * direction)
        out = dict(positions_rad=q, velocities_rad_s=velocity, gripper=1.0, gripper_available=False,
                   temperatures_c=[float(max(s.temperature_mos, s.temperature_rotor)) for s in states])
        if self.handle:
            out['handle'] = handle_state(handle_report(dry['iface']))
        if self.has_jaw and dry['stops'] is not None:
            closed, opened = dry['stops']
            raw = float(states[-1].position) * float(self.gripper_type.get_motor_direction(self.arm_type))
            low, high = min(closed, opened), max(closed, opened)
            options = [raw + 2 * math.pi * k for k in range(-3, 4) if low - JAW_MARGIN_RAD <= raw + 2 * math.pi * k <= high + JAW_MARGIN_RAD]
            if options:
                best = min(options, key=lambda v: abs(v - (low + high) / 2))
                out.update(gripper=float(np.clip((best - closed) / (opened - closed), 0.0, 1.0)), gripper_available=True)
        return out

    def read_hardware(self):
        if self._dry is not None:
            return self._read_dry()
        robot, chain = self.robot, self.robot.motor_chain
        states = chain.read_states()
        now = time.time()
        ages = [now - s.timestamp for s in states]
        oldest = int(np.argmax(ages))
        if not chain.running or not robot._server_thread.is_alive():
            raise ArmHardwareError(f'the motor loop stopped; motor {states[oldest].id} was heard from last '
                                   f'{ages[oldest]:.2f} s ago')
        if ages[oldest] > self.feedback_timeout_s:
            raise ArmHardwareError(f'motor {states[oldest].id} has not answered for {ages[oldest]:.2f} s')
        faulted = [s.id for s in states if s.error_code != '0x1']
        if faulted:
            raise ArmHardwareError(f'motor fault on {faulted}')
        seen = robot.get_observations()
        out = dict(positions_rad=[float(v) for v in seen['joint_pos'][:6]],
                   velocities_rad_s=[float(v) for v in seen['joint_vel'][:6]],
                   temperatures_c=[float(max(s.temp_mos, s.temp_rotor)) for s in states],
                   gripper=1.0, gripper_available=self.has_jaw)
        if self.handle:
            reports = chain.get_same_bus_device_states() or []
            out['handle'] = handle_state(reports[0] if reports else None)
        if self.has_jaw:
            jaw = states[-1]
            out['gripper'] = float(np.clip(seen['gripper_pos'][0], 0.0, 1.0))
            out['gripper_effort'] = self._jaw_effort(jaw)
            self._remember_jaw(float(jaw.pos), now)
        return out

    def _grip_torque(self, position):
        full = self._full_grip_map or self.robot._gripper_force_limiter.gripper_force_torque_map
        return float(full(current_angle=position)) + 0.3        # what the limiter holds at a stall, at the full force setting

    def _jaw_effort(self, jaw):
        return float(min(1.0, abs(float(jaw.eff)) / max(self._grip_torque(float(jaw.pos)), 1e-3)))

    def _remember_jaw(self, position, now):
        at, value = self._jaw_saved
        if now - at < JAW_SAVE_EVERY_S:
            return
        self._jaw_saved = (now, value)
        if (value is None or abs(position - value) > 0.02) and self._jaw_memory():
            self._jaw_saved = (now, position)
            try:
                self._save_jaw(last=position)
            except OSError:
                pass

    def write_hardware(self, positions_rad, gripper):
        jaw = [min(float(gripper), 1.0 - self.gripper_open_margin)] if self.has_jaw else []
        target = np.asarray(list(positions_rad) + jaw, dtype=float)
        share = self.gripper_hold_effort if self._jaw_guard.state == 'gripping' else 1.0
        if self.has_jaw and share != self._grip_share:
            self._set_grip_force(share)
        self._ramp_steps += 1
        share = 1.0 if self.ramp_s <= 0 else min(1.0, self._ramp_steps / max(1.0, self.ramp_s * self.rate_hz))
        if share >= 1.0:
            self.robot.command_joint_pos(target)
        else:
            info = self.robot.get_robot_info()
            self.robot.command_joint_state(dict(pos=target, vel=np.zeros_like(target), kp=info['kp'] * share, kd=info['kd']))

    def limp_hardware(self):
        self._ramp_steps = 0
        self.robot.enter_gravity_comp_idle()

    def close_hardware(self):
        if self._dry is not None:
            dry, self._dry = self._dry, None
            dry['iface'].close()
            return
        self._stop_robot()

    def motors_off_hardware(self):
        """Gravity compensation fades out while the arm is watched: it is switched off only if the rest pose carries it."""
        robot = self.robot
        if robot is None:
            return 'the motors were not on'
        chain = robot.motor_chain
        full = np.array(robot.gravity_comp_factor, dtype=float)
        robot.enter_gravity_comp_idle()
        start = np.array([s.pos for s in chain.read_states()], dtype=float)[:len(self.config.motor_list)]
        for step in [*range(FADE_STEPS, -1, -1), *[0] * FADE_WATCH_STEPS]:      # and it is watched a while with none left
            robot.gravity_comp_factor = full * (step / FADE_STEPS)
            time.sleep(FADE_STEP_S)
            now = np.array([s.pos for s in chain.read_states()], dtype=float)[:len(start)]
            drift = float(np.abs(now - start).max())
            if drift > self.park_sag_rad:
                robot.gravity_comp_factor = full
                return f'the arm moved {drift:.3f} rad as gravity compensation faded: this rest pose does not carry it, motors left on'
        if self.has_jaw and self._jaw_memory():
            self._save_jaw(last=float(chain.read_states()[-1].pos))
        self.force_motors_off()
        return None

    def force_motors_off(self):
        robot = self.robot
        if robot is None:
            return
        chain = robot.motor_chain
        robot._stop_event.set()
        robot._server_thread.join(timeout=2)
        chain.running = False
        if hasattr(chain, '_control_thread') and chain._control_thread.is_alive():
            chain._control_thread.join(timeout=2)
        for motor_id, _kind in self._motor_rows():
            chain.motor_interface.motor_off(motor_id)

    def switch_off(self):
        _close_left_open(self.bus)
        iface = self._interface()
        still_on = []
        try:
            for motor_id, kind in self._motor_rows():
                iface.motor_off(motor_id)
                if iface.read_state_while_disabled(motor_id, kind).error_code == '0x1':
                    still_on.append(f'motor {motor_id}')
        finally:
            iface.close()
        return still_on

    def _stop_robot(self):
        robot, self.robot = self.robot, None
        if robot is None:
            return
        chain = robot.motor_chain
        try:
            if chain.running and robot._server_thread.is_alive():
                if not self._limp:                              # a released arm is already there
                    self._ease_into_gravity_comp(robot)
                if self.has_jaw and self._jaw_memory():
                    self._save_jaw(last=float(chain.read_states()[-1].pos))
        finally:
            with contextlib.redirect_stdout(sys.stderr):
                robot.close()

    @staticmethod
    def _ease_into_gravity_comp(robot):
        """The stiffness fades out around the measured pose. The motors stay enabled: disabling would drop the arm.
        Their last frame is gravity torque plus damping, and past the motor's CAN timeout its firmware takes over."""
        info = robot.get_robot_info()
        pose = np.array(robot.get_joint_pos(), dtype=float)
        for step in range(SETTLE_STEPS, 0, -1):
            share = (step - 1) / SETTLE_STEPS
            robot.command_joint_state(dict(pos=pose.copy(), vel=np.zeros_like(pose), kp=info['kp'] * share,
                                           kd=info['kd'] * share + info['grav_comp_kd'] * (1 - share)))
            time.sleep(SETTLE_S / SETTLE_STEPS)
        robot.enter_gravity_comp_idle()
        time.sleep(0.03)

    # -- read-only check -----------------------------------------------------------------------------------------
    def survey(self) -> dict:
        motors = _yam_motor_list(self.arm_type, self.gripper_type)
        out = dict(arm=self.arm_type.value, gripper=self.gripper_type.value, bus=self.bus, side=self.side, ok=False,
                   gripper_calibrated=not self._jaw_needs_stops() or self.gripper_limits is not None
                   or self._jaw_memory() is not None)
        if not self.bus:
            return dict(out, error='no CAN address given')
        try:
            run_startup_checks(self.bus, motors, check_motor_types=True, check_motor_config=True,
                               control_mode=ControlMode.MIT, repair=False, expected_registers=self.config.motor_registers)
            iface = RawCanInterface(channel=self.bus, bustype='socketcan', name='roborun_survey')
            try:
                rows = []
                for motor_id, motor_type in motors:
                    values = {name: read_register(iface, motor_id, name) for name in SURVEY_REGISTERS}
                    rows.append(dict(id=motor_id, type=motor_type, position_rad=float(values.pop('xout')),
                                     registers=values))
            finally:
                iface.close()
        except Exception as exc:
            _raise_adapter_error(exc)                           # no bus at all: the command says so in plain words
            return dict(out, error=f'{type(exc).__name__}: {exc}')
        out.update(ok=True, motors=rows)
        if self.handle:
            out['handle'] = self._read_handle_once()
            if not out['handle']['available']:
                out['handle_warning'] = 'the teaching handle did not answer: the arm runs, and read() reports it unavailable'
        text = canbus.load_warning(self.bus, self.can_frames_per_s(), who=self.arm_type.value)
        if text:
            out['can_bus_warning'] = text
        if any(not row['registers']['TIMEOUT'] for row in rows):
            out['warning'] = 'a motor has TIMEOUT 0: it keeps its last command if the computer stops talking to it'
        return out

    # -- the one deliberate jaw movement -------------------------------------------------------------------------
    def calibrate_gripper(self) -> dict:
        """Close and open the jaw against its stops and remember them. Only the jaw motor is ever enabled: the joints are
        not touched, so an arm stays exactly as it is, resting or held by whatever holds it."""
        if not self._jaw_needs_stops():
            raise ArmHardwareError(f'the {self.gripper_type.value} gripper needs no calibration')
        motor_id, kind = self._motor_rows()[-1]
        direction = float(self.gripper_type.get_motor_direction(self.arm_type))
        span = 2 * float(MotorType.get_motor_constants(kind).POSITION_MAX)
        try:
            run_startup_checks(self.bus, [[motor_id, kind]], check_motor_types=True, check_motor_config=True,
                               control_mode=ControlMode.MIT, repair=False)
            iface = self._interface()
        except Exception as exc:
            _raise_adapter_error(exc)
            raise ArmHardwareError(f'{type(exc).__name__}: {exc}') from exc
        track = dict(raw=None, turns=0.0)

        def follow(state):                                      # the motor reports within +-PMAX: count the wraps
            raw = float(state.position)
            if track['raw'] is not None:
                track['turns'] -= round((raw - track['raw']) / span) * span
            track['raw'] = raw
            return raw + track['turns']

        def push(torque):
            seen, still, last, deadline = [], 0, None, time.monotonic() + SWEEP_MAX_S
            while time.monotonic() < deadline and still < SWEEP_STILL_STEPS:
                step_end = time.monotonic() + SWEEP_STEP_S
                while time.monotonic() < step_end:
                    position = follow(iface.set_control(motor_id, kind, 0.0, 0.0, 0.0, 0.0, torque))
                    time.sleep(0.004)
                seen.append(position)
                still = still + 1 if last is not None and abs(position - last) < SWEEP_STILL_RAD else 0
                last = position
            return seen

        try:
            iface.motor_on(motor_id, kind)
            start = follow(iface.set_control(motor_id, kind, 0.0, 0.0, 0.0, 0.0, 0.0))
            raw_seen = [start] + push(SWEEP_TORQUE_NM) + push(-SWEEP_TORQUE_NM)         # toward closed, then open: it ends open
            final = follow(iface.set_control(motor_id, kind, 0.0, 0.0, 0.0, 0.0, 0.0))
        except ArmHardwareError:
            raise
        except Exception as exc:
            _raise_adapter_error(exc)
            raise ArmHardwareError(f'the jaw motor stopped answering during the calibration: {exc}') from exc
        finally:
            try:
                iface.motor_off(motor_id)
            except Exception:
                pass
            iface.close()
            _close_left_open(self.bus)
        # the frame the running arm reads the jaw in: direction * raw, with whole turns taken out around where it started
        shift = round(direction * start / (2 * math.pi)) * 2 * math.pi
        seen = [direction * value - shift for value in raw_seen]
        high, low = max(seen), min(seen)
        closed, opened = (high, low) if direction > 0 else (low, high)
        limiter = _load_gripper_config(self.gripper_type.value, self.arm_type).limiter_params or {}
        expected = limiter.get('motor_stroke')
        if high - low < (0.7 * expected if expected else 0.5):
            raise ArmHardwareError(f'the jaw only travelled {high - low:.2f} rad' + (f' of about {expected:.2f}' if expected else '')
                                   + ': make sure nothing is between the jaws and that the motor is free')
        self._save_jaw(closed=closed, open=opened, last=direction * final - shift, stroke_rad=high - low, calibrated_at=time.time())
        return dict(closed=closed, open=opened, stroke_rad=high - low, file=str(self._jaw_file))


def main(argv=None) -> int:
    from . import open_arm, parse_spec
    parser = argparse.ArgumentParser(prog='python -m roborun_sdk.arms.yam')
    sub = parser.add_subparsers(dest='command', required=True)
    cal = sub.add_parser('calibrate-gripper', help='sweep the jaw between its stops once (jaws empty; only the jaw moves)')
    cal.add_argument('arm', help='ARM@ADDRESS, e.g. yam@slcan://SERIAL')
    cal.add_argument('--side', default='left')
    args = parser.parse_args(argv)
    name, bus = parse_spec(args.arm)
    try:
        with safety.ArmsLock('calibrate-gripper', [bus]):
            print(json.dumps(open_arm(name, args.side, bus, connect=False).calibrate_gripper(), indent=1))
    except safety.ArmsBusy as exc:
        raise SystemExit(str(exc)) from None
    except Exception as exc:
        from .connect import explain
        raise SystemExit(explain(exc, args.arm)) from None
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
