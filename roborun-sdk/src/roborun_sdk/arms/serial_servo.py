"""Serial bus servos: finding the port, and the arm logic Feetech and Dynamixel share.

    serial:///dev/ttyACM0     a device path
    serial://SERIAL           the USB adapter with that serial number: stable across replugging, Linux and macOS
    serial://port/1-2.3       the USB adapter plugged into that port: for adapters with no serial or a shared one
"""
import collections
import errno
import sys
import time

import serial
from serial.tools import list_ports

from .base import ArmHardwareError
from .canbus import serial_permission_fix
from .motor_arm import TWO_PI, MotorArm, Reading


def list_serial_ports():
    rows = []
    for port in list_ports.comports():
        device = port.device.replace('/dev/tty.', '/dev/cu.') if sys.platform == 'darwin' else port.device
        location = (getattr(port, 'location', None) or '').split(':')[0] or None
        rows.append(dict(device=device, serial=port.serial_number or None, product=port.product, usb_port=location,
                         usb=getattr(port, 'vid', None) is not None))
    count = collections.Counter(r['serial'] for r in rows if r['serial'])
    for row in rows:
        if row['serial'] and count[row['serial']] == 1:
            row['address'] = f"serial://{row['serial']}"
        else:
            row['address'] = f"serial://port/{row['usb_port']}" if row['usb_port'] else f"serial://{row['device']}"
            row['note'] = (f"shares serial {row['serial']} with another adapter" if row['serial'] else 'has no serial number') \
                + ': this address follows where it is plugged in'
    return rows


def _connected():
    return ', '.join(r['address'] for r in list_serial_ports() if r['usb']) or 'none'


def resolve_port(address):
    if not address or not address.startswith('serial://'):
        raise ArmHardwareError(f'{address!r} is not a serial address: use serial:///dev/ttyACM0, serial://USB_SERIAL or '
                               'serial://port/USB_PORT (`roborun hw detect` prints them)')
    target = address[len('serial://'):]
    if target.startswith('/'):
        return target
    rows = list_serial_ports()
    matches = [r for r in rows if r['usb_port'] == target[5:]] if target.startswith('port/') \
        else [r for r in rows if r['serial'] == target]
    if len(matches) > 1:
        raise ArmHardwareError(f'{address}: {len(matches)} connected adapters share this serial number. Next: use one of '
                               + ', '.join(f"serial://port/{r['usb_port']}" if r['usb_port'] else f"serial://{r['device']}" for r in matches))
    if not matches:
        raise ArmHardwareError(f'{address}: this adapter is not connected (connected serial adapters: {_connected()}). '
                               'Next: check the USB cable, or run `roborun hw detect`')
    return matches[0]['device']


def open_port(address, baud):
    device = resolve_port(address)
    try:
        port = serial.Serial(device, baudrate=baud, timeout=0.01, write_timeout=0.1, exclusive=True)
    except serial.SerialException as exc:
        code = exc.errno
        if code in (errno.EACCES, errno.EPERM):
            raise ArmHardwareError(f'{address}: this user may not open {device}. Next: {serial_permission_fix(device)}') from exc
        if code in (errno.EBUSY, errno.EAGAIN, errno.EWOULDBLOCK) or 'lock' in str(exc).lower():
            raise ArmHardwareError(f'{address}: {device} is in use by another program. Next: close it '
                                   '(`roborun down` if it is this robot)') from exc
        if code == errno.ENOENT:
            raise ArmHardwareError(f'{address}: {device} does not exist. Next: check the USB cable, or run `roborun hw detect`') from exc
        raise
    if hasattr(port, 'set_low_latency_mode'):
        try:
            port.set_low_latency_mode(True)                # USB adapters otherwise batch replies for up to 16 ms
        except (OSError, ValueError, NotImplementedError):
            pass
    port.reset_input_buffer()
    return port


class SerialServoArm(MotorArm):
    baud = 1_000_000
    counts_per_turn = 4096
    reply_timeout_s = 0.03
    missed_cycles_allowed = 5
    rate_hz = 50.0

    # register addresses, per family
    MODEL = TORQUE = GOAL = MODE = 0
    BLOCK = (0, 0)                                         # the state registers read every cycle

    def __init__(self, *args, baud=None, **kw):
        super().__init__(*args, **kw)
        if baud is not None: self.baud = int(baud)
        self.port = None
        self._buffer = bytearray()
        self._seen, self._missed = {}, {}
        self._strength = None
        self._strength_scale = None

    # -- a family supplies these ---------------------------------------------------------------------------------
    def _send_packet(self, motor_id, instruction, params=b''): raise NotImplementedError
    def _next_status(self, deadline):
        """(id, error, data) of the next valid status packet, or None at the deadline."""
        raise NotImplementedError
    def _sync_read(self, address, length, ids): raise NotImplementedError
    def _sync_write(self, address, length, values): raise NotImplementedError
    def _decode(self, motor, error, data): raise NotImplementedError
    def _model_number(self, motor): raise NotImplementedError
    def _configure(self, motor): pass
    def _set_strength(self, fraction):
        """Holding strength of every motor, 0 (the release level) .. 1 (the full level)."""
        raise NotImplementedError
    def _bus_ids(self):
        return [m.id for m in self.motors]
    def _extra_checks(self):
        return {}, []

    # -- the bus -------------------------------------------------------------------------------------------------
    def _open_bus(self):
        self.port = open_port(self.bus, self.baud)
        self._buffer.clear()
        self._seen, self._missed = {}, {}

    def _close_bus(self):
        if self.port is not None:
            self.port.close()
            self.port = None

    def _fill(self, deadline):
        waiting = self.port.in_waiting
        if waiting:
            self._buffer += self.port.read(waiting)
            return True
        if time.monotonic() >= deadline:
            return False
        chunk = self.port.read(1)
        self._buffer += chunk
        return bool(chunk) or time.monotonic() < deadline

    def _transact(self, motor_id, instruction, params=b''):
        self.port.reset_input_buffer()
        self._buffer.clear()
        self._send_packet(motor_id, instruction, params)
        deadline = time.monotonic() + self.reply_timeout_s
        while (status := self._next_status(deadline)) is not None:
            if status[0] == motor_id:
                return status
        return None

    def _read_register(self, motor_id, address, length):
        raise NotImplementedError

    def _write_register(self, motor_id, address, data):
        raise NotImplementedError

    def _to_counts(self, radians):
        return round(radians * self.counts_per_turn / TWO_PI)

    def _to_radians(self, counts):
        return counts * TWO_PI / self.counts_per_turn

    # -- MotorArm hooks ------------------------------------------------------------------------------------------
    def _probe(self):
        rows, problems = {}, []
        with self._io:
            for motor in self.motors:
                row = rows[motor.name] = dict(id=motor.id, model=motor.model)
                number = self._model_number(motor)
                if number is None:
                    problems.append(f'{motor.name}: no servo answers at id {motor.id} (power, wiring, id, {self.baud} baud?)')
                    continue
                row['model_number'] = number
                torque = self._read_register(motor.id, self.TORQUE, 1)
                row['torque_on'] = None if torque is None else bool(torque[0])
                if number not in self._accepted_models(motor):
                    problems.append(f'{motor.name}: wrong servo at id {motor.id}: model number {number}, the table says '
                                    f'{motor.model} ({sorted(self._accepted_models(motor))})')
            if not problems:
                more_rows, more = self._extra_checks()
                rows.update(more_rows)
                problems += more
        return rows, problems

    def _accepted_models(self, motor):
        raise NotImplementedError

    def _read_states(self):
        with self._io:
            ids = self._bus_ids()
            replies = self._sync_read(self.BLOCK[0], self.BLOCK[1], ids)
            for motor in self.motors:
                reply = replies.get(motor.id)
                if reply is not None:
                    self._seen[motor.name], self._missed[motor.name] = self._decode(motor, *reply), 0
                    continue
                self._missed[motor.name] = self._missed.get(motor.name, 0) + 1
                if motor.name not in self._seen or self._missed[motor.name] > self.missed_cycles_allowed:
                    self._seen.pop(motor.name, None)
                    raise ArmHardwareError(f'{motor.name} (servo id {motor.id}) stopped answering')
            self._check_extra_replies(replies)
            return dict(self._seen)

    def _check_extra_replies(self, replies):
        pass

    def _engage(self, states):
        for motor in self.motors:
            torque = self._read_register(motor.id, self.TORQUE, 1)
            if torque is None:
                raise ArmHardwareError(f'{motor.name}: no answer while preparing it')
            mode = self._read_register(motor.id, self.MODE, 1)
            wanted = int(motor.spec.get('mode', self.default_mode))
            if mode is None or mode[0] != wanted:
                if torque[0]:
                    raise ArmHardwareError(f'{motor.name}: operating mode is {mode and mode[0]}, {wanted} is needed, and '
                                           'torque is on; power-cycle the arm so it can be set safely')
                self._write_checked(motor, self.MODE, bytes([wanted]))
            self._configure(motor)
        self._write_goals([(m, states[m.name].pos) for m in self.motors])
        self._set_strength(1.0 if self._adopted else 0.0)   # a servo already holding the arm is not weakened
        self._sync_write(self.TORQUE, 1, {m.id: b'\x01' for m in self.motors})
        time.sleep(.005)
        for motor in self.motors:
            torque = self._read_register(motor.id, self.TORQUE, 1)
            if torque is None or not torque[0]:
                self._soften_at(states)
                raise ArmHardwareError(f'{motor.name} (servo id {motor.id}) did not take torque')

    def _write_checked(self, motor, address, data):
        if not self._write_register(motor.id, address, data):
            raise ArmHardwareError(f'{motor.name}: writing register {address} failed')

    def _write_goals(self, targets):
        raise NotImplementedError

    def _command(self, targets, ramp):
        self._write_goals(targets)
        ramp = 1.0 if self._adopted else ramp
        if ramp <= 1.0 and (self._strength != ramp or self._strength_scale != self._jaw_scale()):
            self._set_strength(ramp)

    def _soften_at(self, states):
        self._write_goals([(m, states[m.name].pos) for m in self.motors if m.name in states])
        self._set_strength(0.0)

    def _soften(self):
        """Torque stays on, limited to the table's low level, holding the measured pose."""
        states = self._seen or self._last
        self._soften_at(states)

    def calibration_open(self):
        states = super().calibration_open()
        for motor in self.motors:
            torque = self._read_register(motor.id, self.TORQUE, 1)
            states[motor.name].enabled = bool(torque and torque[0])
        return states

    def _is_powered(self, motor, reading):
        torque = self._read_register(motor.id, self.TORQUE, 1)
        return bool(torque and torque[0])

    def _motors_still_on(self):
        out = []
        for motor in self.motors:
            torque = self._read_register(motor.id, self.TORQUE, 1)
            if torque is None or torque[0]:
                out.append(motor.name)
        return out

    def _torque_off(self):
        with self._io:
            self._sync_write(self.TORQUE, 1, {m.id: b'\x00' for m in self.motors})
            time.sleep(.01)
            self._seen.clear()
