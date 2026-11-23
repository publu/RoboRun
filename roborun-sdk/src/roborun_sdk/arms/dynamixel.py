"""ROBOTIS Dynamixel X-series servos, protocol 2.0: Koch v1.1, OMX, WidowX 250 6-DOF, ViperX 300 6-DOF (tables in data/arms).

Packets FF FF FD 00 id len(2) instruction params crc16, with FF FF FD inside a packet stuffed to FF FF FD FD. Register
map from the X-series e-Manual, as in rustypot's tables. A `shadow` motor is a second motor on the same joint: it follows
its master through the Secondary ID register, so it is checked and watched but never addressed with goals.
"""
import time

from .base import ArmHardwareError
from .motor_arm import Reading
from .serial_servo import SerialServoArm

PING, READ, WRITE, STATUS, SYNC_READ, SYNC_WRITE = 1, 2, 3, 0x55, 0x82, 0x83
BROADCAST = 0xfe
REG_HOMING = 20
REG_MODEL, REG_MODE, REG_SECONDARY, REG_TORQUE, REG_HW_ERROR, REG_GOAL_PWM, REG_GOAL = 0, 11, 12, 64, 70, 100, 116
REG_PRESENT_PWM = 124
EEPROM_END = 64
MODE_EXTENDED, MODE_CURRENT_POSITION = 4, 5
MODELS = {'xl330-m077': 1190, 'xl330-m288': 1200, 'xl430-w250': 1060, 'xm430-w350': 1020, 'xm540-w270': 1120}
MA_PER_UNIT = {'xl330-m077': 1.0, 'xl330-m288': 1.0, 'xm430-w350': 2.69, 'xm540-w270': 2.69}   # xl430: 0.1 % load instead
RPM_PER_UNIT = 0.229
HW_ERRORS = {0: 'input voltage', 2: 'over-temperature', 3: 'encoder', 4: 'electrical shock', 5: 'overload'}


def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1 ^ 0x8005 if crc & 0x8000 else crc << 1) & 0xffff
    return crc


def stuff(params):
    return bytes(params).replace(b'\xff\xff\xfd', b'\xff\xff\xfd\xfd')


def unstuff(params):
    return bytes(params).replace(b'\xff\xff\xfd\xfd', b'\xff\xff\xfd')


class DynamixelArm(SerialServoArm):
    MODEL, TORQUE, GOAL, MODE = REG_MODEL, REG_TORQUE, REG_GOAL, REG_MODE
    BLOCK = (REG_PRESENT_PWM, 23)                          # pwm, current, velocity, position .. temperature
    default_mode = 3

    def __init__(self, *args, shadows=(), **kw):
        super().__init__(*args, **kw)
        self.shadows = [dict(row) for row in shadows]
        self._shadow_missed = {}

    def _counts_turns(self, motor):
        return int(motor.spec.get('mode', self.default_mode)) in (MODE_EXTENDED, MODE_CURRENT_POSITION)

    def _send_packet(self, motor_id, instruction, params=b''):
        body = stuff(params)
        head = b'\xff\xff\xfd\x00' + bytes([motor_id]) + (len(body) + 3).to_bytes(2, 'little') + bytes([instruction]) + body
        self.port.write(head + crc16(head).to_bytes(2, 'little'))

    def _next_status(self, deadline):
        buf = self._buffer
        while True:
            start = buf.find(b'\xff\xff\xfd\x00')
            if start > 0:
                del buf[:start]
            if start >= 0 and len(buf) >= 7:
                total = 7 + int.from_bytes(buf[5:7], 'little')
                if total > 512:
                    del buf[:4]
                    continue
                if len(buf) >= total:
                    packet = bytes(buf[:total])
                    if crc16(packet[:-2]) == int.from_bytes(packet[-2:], 'little') and packet[7] == STATUS:
                        del buf[:total]
                        return packet[4], packet[8], unstuff(packet[9:-2])
                    del buf[:4]
                    continue
            if not self._fill(deadline):
                return None

    def _read_register(self, motor_id, address, length):
        status = self._transact(motor_id, READ, address.to_bytes(2, 'little') + length.to_bytes(2, 'little'))
        return None if status is None or len(status[2]) != length else status[2]

    def _write_register(self, motor_id, address, data):
        status = self._transact(motor_id, WRITE, address.to_bytes(2, 'little') + bytes(data))
        return status is not None and status[1] & 0x7f == 0

    def _sync_read(self, address, length, ids):
        self.port.reset_input_buffer()
        self._buffer.clear()
        self._send_packet(BROADCAST, SYNC_READ, address.to_bytes(2, 'little') + length.to_bytes(2, 'little') + bytes(ids))
        out, deadline = {}, time.monotonic() + self.reply_timeout_s
        while len(out) < len(ids) and (status := self._next_status(deadline)) is not None:
            if status[0] in ids and len(status[2]) == length:
                out[status[0]] = (status[1], status[2])
        return out

    def _sync_write(self, address, length, values):
        params = bytearray(address.to_bytes(2, 'little') + length.to_bytes(2, 'little'))
        for motor_id, data in values.items():
            params += bytes([motor_id]) + bytes(data)
        self._send_packet(BROADCAST, SYNC_WRITE, params)

    def _bus_ids(self):
        return [m.id for m in self.motors] + [int(s['id']) for s in self.shadows]

    def _model_number(self, motor):
        data = self._read_register(motor.id, REG_MODEL, 2)
        return None if data is None else int.from_bytes(data, 'little')

    def _accepted_models(self, motor):
        return {MODELS[motor.model]}

    def _alert_text(self, motor_id):
        data = self._read_register(motor_id, REG_HW_ERROR, 1)
        if data is None:
            return 'hardware alert'
        return 'hardware alert: ' + (', '.join(t for bit, t in HW_ERRORS.items() if data[0] >> bit & 1) or f'{data[0]:#x}')

    def _decode(self, motor, error, d):
        current = int.from_bytes(d[2:4], 'little', signed=True)
        velocity = int.from_bytes(d[4:8], 'little', signed=True) * RPM_PER_UNIT * 2 * 3.141592653589793 / 60
        position = int.from_bytes(d[8:12], 'little', signed=True)
        unit = MA_PER_UNIT.get(motor.model)
        effort = current * unit if unit else current / 1000.0
        fault = self._alert_text(motor.id) if error & 0x80 else None
        return Reading(self._to_radians(position), velocity, effort, float(d[22]), fault)

    def _check_extra_replies(self, replies):
        for shadow in self.shadows:
            sid = int(shadow['id'])
            reply = replies.get(sid)
            if reply is None:
                self._shadow_missed[sid] = self._shadow_missed.get(sid, 0) + 1
                if self._shadow_missed[sid] > self.missed_cycles_allowed:
                    raise ArmHardwareError(f'{shadow["name"]} (servo id {sid}) stopped answering')
                continue
            self._shadow_missed[sid] = 0
            if reply[0] & 0x80:
                raise ArmHardwareError(f'{shadow["name"]} (servo id {sid}): {self._alert_text(sid)}')

    def _extra_checks(self):
        rows, problems = {}, []
        for shadow in self.shadows:
            sid, name = int(shadow['id']), shadow['name']
            rows[name] = dict(id=sid, model=shadow['model'], shadow_of=shadow['master'])
            data = self._read_register(sid, REG_MODEL, 2)
            if data is None:
                problems.append(f'{name}: no servo answers at id {sid}')
            elif int.from_bytes(data, 'little') != MODELS[shadow['model']]:
                problems.append(f'{name}: wrong servo at id {sid}: model number {int.from_bytes(data, "little")}')
            else:
                link = self._read_register(sid, REG_SECONDARY, 1)
                if link is None or link[0] != int(shadow['master']):
                    problems.append(f'{name}: its Secondary ID is {link and link[0]}, it must follow motor {shadow["master"]}')
        return rows, problems

    def _homing(self, motor):
        data = self._read_register(motor.id, REG_HOMING, 4)
        return None if data is None else int.from_bytes(data, 'little', signed=True)

    def _check_calibration(self, data):
        """A calibration imported from LeRobot names the homing offsets it was made with: they must still be there."""
        wanted = (data.get('facts') or {}).get('homing') or {}
        return [f'{m.name}: its homing offset is not the calibrated one (servo replaced or re-homed?): calibrate again'
                for m in self.motors if m.name in wanted and self._homing(m) != wanted[m.name]]

    def _configure(self, motor):
        torque = self._read_register(motor.id, REG_TORQUE, 1)
        for address, size, value in motor.spec.get('registers', ()):
            data = int(value).to_bytes(size, 'little', signed=value < 0)
            if self._read_register(motor.id, address, size) == data:
                continue
            if address < EEPROM_END and torque and torque[0]:
                raise ArmHardwareError(f'{motor.name}: register {address} needs torque off; power-cycle the arm')
            self._write_checked(motor, address, data)

    def _write_goals(self, targets):
        values = {}
        for motor, position in targets:
            counts = self._to_counts(position)
            if not self._counts_turns(motor):
                counts = max(0, min(self.counts_per_turn - 1, counts))
            values[motor.id] = counts.to_bytes(4, 'little', signed=True)
        self._sync_write(REG_GOAL, 4, values)

    def _set_strength(self, fraction):
        values = {}
        for motor in self.motors:
            full = int(motor.spec.get('goal_pwm', 885))
            if motor is self.gripper:
                full = round(full * self._jaw_scale())
            soft = min(full, int(motor.spec.get('soft_pwm', 300)))
            values[motor.id] = round(soft + fraction * (full - soft)).to_bytes(2, 'little')
        self._sync_write(REG_GOAL_PWM, 2, values)
        self._strength, self._strength_scale = fraction, self._jaw_scale()
