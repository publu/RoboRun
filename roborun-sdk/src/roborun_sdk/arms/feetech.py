"""Feetech STS3215 serial bus servos: SO-100, SO-101, XLeRobot (tables in data/arms).

Packets FF FF id len instruction params checksum, little-endian registers (feetech-servo-sdk); register map as in
rustypot's STS3215 table. Sign-magnitude values: position and speed use bit 15, load bit 10, the homing offset bit 11.
`present = actual - homing offset`: calibration uses that to put each joint's zero mid-encoder, away from the 0/4095
seam the servo cannot cross in position mode.
"""
import time

from .base import ArmHardwareError
from .motor_arm import Reading
from .serial_servo import SerialServoArm

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 1, 2, 3, 0x82, 0x83
BROADCAST = 0xfe
MODEL_STS3215 = 777
REG_MODEL, REG_MIN_ANGLE, REG_MAX_ANGLE, REG_HOMING, REG_MODE, REG_TORQUE, REG_ACCEL = 3, 9, 11, 31, 33, 40, 41
REG_GOAL, REG_GOAL_TIME, REG_GOAL_SPEED, REG_TORQUE_LIMIT, REG_LOCK, REG_PRESENT = 42, 44, 46, 48, 55, 56
MID = 2047
ERRORS = {1: 'voltage', 2: 'angle', 4: 'over-temperature', 8: 'over-current', 32: 'overload'}


def from_sign_magnitude(value, bit):
    return -(value & ((1 << bit) - 1)) if value >> bit & 1 else value & ((1 << bit) - 1)


def to_sign_magnitude(value, bit):
    return abs(value) | 1 << bit if value < 0 else value


class FeetechArm(SerialServoArm):
    MODEL, TORQUE, GOAL, MODE = REG_MODEL, REG_TORQUE, REG_GOAL, REG_MODE
    BLOCK = (REG_PRESENT, 15)                              # position .. current
    default_mode = 0
    multi_turn = False

    def _send_packet(self, motor_id, instruction, params=b''):
        body = bytes([motor_id, len(params) + 2, instruction]) + bytes(params)
        self.port.write(b'\xff\xff' + body + bytes([~sum(body) & 0xff]))

    def _next_status(self, deadline):
        buf = self._buffer
        while True:
            start = buf.find(b'\xff\xff')
            if start > 0:
                del buf[:start]
            if len(buf) >= 4 and buf[:2] == b'\xff\xff':
                length = buf[3]
                if buf[2] == 0xff or length < 2:
                    del buf[0]
                    continue
                if len(buf) >= length + 4:
                    packet = bytes(buf[:length + 4])
                    if ~sum(packet[2:-1]) & 0xff == packet[-1]:
                        del buf[:length + 4]
                        return packet[2], packet[4], packet[5:-1]
                    del buf[:2]
                    continue
            if not self._fill(deadline):
                return None

    def _read_register(self, motor_id, address, length):
        status = self._transact(motor_id, READ, bytes([address, length]))
        return None if status is None or len(status[2]) != length else status[2]

    def _write_register(self, motor_id, address, data):
        status = self._transact(motor_id, WRITE, bytes([address]) + bytes(data))
        return status is not None and status[1] == 0

    def _sync_read(self, address, length, ids):
        self.port.reset_input_buffer()
        self._buffer.clear()
        self._send_packet(BROADCAST, SYNC_READ, bytes([address, length]) + bytes(ids))
        out, deadline = {}, time.monotonic() + self.reply_timeout_s
        while len(out) < len(ids) and (status := self._next_status(deadline)) is not None:
            if status[0] in ids and len(status[2]) == length:
                out[status[0]] = (status[1], status[2])
        return out

    def _sync_write(self, address, length, values):
        params = bytearray([address, length])
        for motor_id, data in values.items():
            params += bytes([motor_id]) + bytes(data)
        self._send_packet(BROADCAST, SYNC_WRITE, params)

    def _model_number(self, motor):
        data = self._read_register(motor.id, REG_MODEL, 2)
        return None if data is None else int.from_bytes(data, 'little')

    def _accepted_models(self, motor):
        return {MODEL_STS3215}

    def _decode(self, motor, error, d):
        position = int.from_bytes(d[0:2], 'little')
        speed = from_sign_magnitude(int.from_bytes(d[2:4], 'little'), 15)
        load = from_sign_magnitude(int.from_bytes(d[4:6], 'little'), 10)
        fault = ', '.join(text for bit, text in ERRORS.items() if error & bit) or None
        return Reading(self._to_radians(from_sign_magnitude(position, 15)), self._to_radians(speed), load / 1000.0,
                       float(d[7]), fault)

    def _configure(self, motor):
        for address, size, value in motor.spec.get('registers', ()):
            self._write_checked(motor, address, int(value).to_bytes(size, 'little'))
        self._write_checked(motor, REG_ACCEL, bytes([254]))
        self._write_checked(motor, REG_GOAL_TIME, bytes(2))
        self._write_checked(motor, REG_GOAL_SPEED, bytes(2))

    def _write_goals(self, targets):
        values = {}
        for motor, position in targets:
            counts = max(0, min(self.counts_per_turn - 1, self._to_counts(position)))
            values[motor.id] = counts.to_bytes(2, 'little')
        self._sync_write(REG_GOAL, 2, values)

    def _set_strength(self, fraction):
        values = {}
        for motor in self.motors:
            full = int(motor.spec.get('torque_limit', 1000))
            if motor is self.gripper:
                full = round(full * self._jaw_scale())
            soft = min(full, int(motor.spec.get('soft_torque', 350)))
            values[motor.id] = round(soft + fraction * (full - soft)).to_bytes(2, 'little')
        self._sync_write(REG_TORQUE_LIMIT, 2, values)
        self._strength, self._strength_scale = fraction, self._jaw_scale()

    def _homing(self, motor):
        data = self._read_register(motor.id, REG_HOMING, 2)
        return None if data is None else from_sign_magnitude(int.from_bytes(data, 'little'), 11)

    def _check_calibration(self, data):
        wanted = (data.get('facts') or {}).get('homing') or {}
        problems = []
        for motor in self.motors:
            if motor.name in wanted and self._homing(motor) != wanted[motor.name]:
                problems.append(f'{motor.name}: its homing offset is not the calibrated one (servo replaced or '
                                're-homed?): run roborun hw calibrate again')
        return problems

    def _on_zero_pose(self, raw):
        """Writes each servo's homing offset so this pose reads mid-encoder. Torque is off."""
        homing = {}
        with self._io:
            for motor in self.motors:
                old = self._homing(motor)
                if old is None:
                    raise ArmHardwareError(f'{motor.name}: no answer while setting its zero')
                actual = (self._to_counts(raw[motor.name]) + old) % self.counts_per_turn
                offset = max(-MID, min(MID, actual - MID))
                self._write_checked(motor, REG_LOCK, b'\x00')
                self._write_checked(motor, REG_HOMING, to_sign_magnitude(offset, 11).to_bytes(2, 'little'))
                self._write_checked(motor, REG_MIN_ANGLE, bytes(2))
                self._write_checked(motor, REG_MAX_ANGLE, (self.counts_per_turn - 1).to_bytes(2, 'little'))
                self._write_checked(motor, REG_LOCK, b'\x01')
                homing[motor.name] = offset
            time.sleep(.02)
        return dict(homing=homing), self.calibration_read()
