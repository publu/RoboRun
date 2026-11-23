"""MIT-mode CAN motors (Damiao, RobStride): the frame layout both families share, and the loop around it.

Every MIT frame the host sends is answered with one state frame, so while the arm streams the commands are also the
reads. A family supplies the addressing and the meaning of the feedback bytes.
"""
import json
import time

import can

from .base import ArmHardwareError
from .canbus import BITRATE, bus_load, load_warning, open_bus
from .motor_arm import MotorArm, Reading

KP_MAX, KD_MAX = 500.0, 5.0


def to_uint(value, lo, hi, bits):
    return round((max(lo, min(hi, value)) - lo) * ((1 << bits) - 1) / (hi - lo))


def to_float(value, lo, hi, bits):
    return value * (hi - lo) / ((1 << bits) - 1) + lo


def mit_payload(position, velocity, kp, kd, torque, pmax, vmax, tmax):
    q, v = to_uint(position, -pmax, pmax, 16), to_uint(velocity, -vmax, vmax, 12)
    k, d, t = to_uint(kp, 0, KP_MAX, 12), to_uint(kd, 0, KD_MAX, 12), to_uint(torque, -tmax, tmax, 12)
    data = bytes([q >> 8, q & 0xff, v >> 4, (v & 0xf) << 4 | k >> 8, k & 0xff, d >> 4, (d & 0xf) << 4 | t >> 8, t & 0xff])
    if data[:6] == b'\xff' * 6:                            # that prefix means a command (enable, disable...), never a target
        raise ArmHardwareError('MIT frame collides with the command prefix')
    return data


def feedback_fields(data, pmax, vmax, tmax):
    q = data[1] << 8 | data[2]
    v = data[3] << 4 | data[4] >> 4
    t = (data[4] & 0xf) << 8 | data[5]
    return to_float(q, -pmax, pmax, 16), to_float(v, -vmax, vmax, 12), to_float(t, -tmax, tmax, 12)


class MitCanArm(MotorArm):
    reply_window_s = 0.004
    question_timeout_s = 0.03
    missed_cycles_allowed = 10

    def __init__(self, *args, bitrate=BITRATE, **kw):
        super().__init__(*args, **kw)
        self.bitrate = bitrate
        self.can = None
        self._fd = False
        self._expect_replies = False
        self._missed = {}
        self._seen = {}

    # -- a family supplies these ---------------------------------------------------------------------------------
    def _decode(self, msg):
        """(motor name, Reading) for a state or fault frame, else None."""
        raise NotImplementedError
    def _ask_state(self):
        """Ask every motor for its state without changing anything."""
        raise NotImplementedError
    def _ask_about(self, motors):
        """A motor missed a reply: ask in a way it answers even when switched off or faulted."""
    def _enable_frame(self, motor):
        raise NotImplementedError
    def _disable_frame(self, motor):
        raise NotImplementedError
    def _command_id(self, motor):
        return motor.id
    def _enabled_ok(self, reading):
        return reading.enabled is not False

    # -- bus load ------------------------------------------------------------------------------------------------
    def can_frames_per_s(self):
        """Every motor gets one frame and answers with one, every cycle."""
        return 2 * len(self.motors) * self.rate_hz

    def bus_load_warning(self):
        return load_warning(self.bus, self.can_frames_per_s(), self.bitrate, self.arm_id)

    def open_hardware(self):
        text = self.bus_load_warning()
        if text:                                       # said at connect; the arm still opens, the user decides
            print(json.dumps(dict(event='can_bus_load', arm=self.side, message=text)), flush=True)
            self.setting_warnings = list(getattr(self, 'setting_warnings', ())) + [text]
        super().open_hardware()

    def survey(self):
        out = super().survey()
        out['can_bus_load'] = round(bus_load(self.can_frames_per_s(), self.bitrate), 3)
        text = self.bus_load_warning()
        if text:
            out['warning'] = text
        return out

    # -- the bus -------------------------------------------------------------------------------------------------
    def _open_bus(self):
        if not self.bus:
            raise ArmHardwareError(f'{self.arm_id}: no CAN address given (ARM@can0, ARM@slcan://SERIAL, ...)')
        self.can = open_bus(self.bus, bitrate=self.bitrate)
        self._fd = bool(getattr(self.can, 'roborun_fd', False))     # the link runs CAN FD: so must every frame we send
        self._missed, self._seen, self._expect_replies = {}, {}, False
        self._drain()

    def _close_bus(self):
        if self.can is not None:
            self.can.shutdown()
            self.can = None

    def _drain(self):
        while self.can.recv(timeout=0) is not None:
            pass

    def _send(self, arbitration_id, data):
        self.can.send(can.Message(arbitration_id=arbitration_id, data=bytes(data), is_extended_id=False,
                                  is_fd=self._fd, bitrate_switch=self._fd))

    def _scale(self, motor):
        s = motor.spec
        return float(s['pmax']), float(s['vmax']), float(s['tmax'])

    def _mit(self, motor, position, kp, kd):
        self._send(self._command_id(motor), mit_payload(position, 0.0, min(kp, KP_MAX), min(kd, KD_MAX), 0.0,
                                                        *self._scale(motor)))

    def _collect(self, window):
        """Take state frames until every motor has answered or the window closes."""
        got, end = {}, time.monotonic() + window
        names = {m.name for m in self.motors}
        while True:
            left = end - time.monotonic() if names - got.keys() else 0     # once all answered, only empty the queue
            msg = self.can.recv(timeout=max(0, left))
            if msg is None:
                return got
            item = self._decode(msg)
            if item is not None and item[0] in names and not (item[0] in got and got[item[0]].fault):
                got[item[0]] = item[1]                                     # the newest frame wins; a fault stays

    # -- MotorArm hooks ------------------------------------------------------------------------------------------
    def _read_states(self):
        with self._io:
            if self._soft and self._engaged:
                self._send_damping()
            elif not self._expect_replies:
                self._ask_state()
            self._expect_replies = False
            first = not self._seen
            got = self._collect(self.question_timeout_s if first else self.reply_window_s)
            quiet = []
            for motor in self.motors:
                reading = got.get(motor.name)
                if reading is not None:
                    self._seen[motor.name], self._missed[motor.name] = reading, 0
                    continue
                quiet.append(motor)
                self._missed[motor.name] = self._missed.get(motor.name, 0) + 1
                if motor.name not in self._seen or self._missed[motor.name] > self.missed_cycles_allowed:
                    self._seen.pop(motor.name, None)
                    raise ArmHardwareError(f'{motor.name} (CAN id {motor.id:#x}) stopped answering')
            if quiet:
                self._ask_about(quiet)
            if self._engaged:
                for motor in self.motors:
                    reading = self._seen[motor.name]
                    if reading.fault is None and not self._enabled_ok(reading):
                        reading.fault = 'the motor switched itself off'
            return dict(self._seen)

    def jaw_squeeze(self):
        motor, cap = self.gripper, self.gripper.spec.get('max_torque_nm')
        if not cap or motor.kp <= 0 or motor.open == motor.closed:
            return super().jaw_squeeze()
        return cap * self._jaw_scale() / motor.kp / abs(motor.open - motor.closed)

    def _engage(self, states):
        done = []
        try:
            for motor in self.motors:
                pos = states[motor.name].pos
                if motor.name in self._adopted:                             # already holding: it keeps holding, here
                    self._mit(motor, pos, motor.kp, motor.kd)
                    if self._await_reply(motor) is None:
                        raise ArmHardwareError(f'{motor.name} (id {motor.id:#x}) did not answer')
                    done.append(motor)
                    continue
                # never a zero-gain frame: a motor whose state cannot be read may be powered and carrying the arm
                self._damp(motor, pos)                                      # also: no stale target can resume at enable
                self._await_reply(motor)
                self._send(self._command_id(motor), self._enable_frame(motor))
                reply = self._await_reply(motor)
                if reply is None or reply.fault or reply.enabled is False:
                    why = 'no answer' if reply is None else reply.fault or 'it stayed disabled'
                    raise ArmHardwareError(f'{motor.name} (id {motor.id:#x}) did not enable: {why}')
                done.append(motor)
                self._damp(motor, reply.pos)
                self._await_reply(motor)
        except BaseException:
            for motor in done:                                              # never drop what is already powered
                self._damp(motor, states[motor.name].pos)
            raise
        self._drain()
        self._seen.clear()

    def _await_reply(self, motor):
        end = time.monotonic() + self.question_timeout_s
        while (left := end - time.monotonic()) > 0:
            msg = self.can.recv(timeout=left)
            item = None if msg is None else self._decode(msg)
            if item is not None and item[0] == motor.name:
                return item[1]
        return None

    def _command(self, targets, ramp):
        for motor, position in targets:
            pmax = self._scale(motor)[0]
            cap = motor.spec.get('max_torque_nm')
            if motor is self.gripper and cap:
                cap = cap * self._jaw_scale()
            if cap and motor.kp > 0 and motor.name in self._last:           # the grip force is capped by the position error
                now = self._last[motor.name].pos
                position = max(now - cap / motor.kp, min(now + cap / motor.kp, position))
            stiffness = motor.kp if motor.name in self._adopted else motor.kp * ramp
            self._mit(motor, max(-pmax, min(pmax, position)), stiffness, motor.kd)
        self._expect_replies = True

    def _damp(self, motor, position):
        self._mit(motor, position, 0.0, float(motor.spec.get('damping', motor.kd)))

    def _send_damping(self):
        for motor in self.motors:
            seen = self._seen.get(motor.name) or self._last.get(motor.name)
            if seen is not None:
                self._damp(motor, seen.pos)

    def _soften(self):
        """Torque stays on with zero stiffness and full damping: the arm sinks slowly and can be moved by hand."""
        self._send_damping()
        self._expect_replies = True

    def _torque_off(self):
        with self._io:
            for motor in self.motors:
                self._send(self._command_id(motor), self._disable_frame(motor))
                self._await_reply(motor)
            self._drain()
            self._seen.clear()
