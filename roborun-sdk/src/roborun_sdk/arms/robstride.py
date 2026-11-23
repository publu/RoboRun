"""RobStride motors over CAN in their MIT protocol: the MakerMods Maker Arm (table in data/arms).

Standard 11-bit ids: commands to the motor id, replies on the host id with the motor id in byte 0, parameter replies on
0x300|id. Protocol facts: makermods-robotics/maker-arm-sdk docs/mit-protocol.md and LeRobot's RobStride driver.
A motor still on RobStride's private 29-bit protocol does not answer here and is reported as missing.
"""
import struct
import time

from .mit_can import MitCanArm, feedback_fields
from .motor_arm import Reading

OP_FAULT, OP_ENABLE, OP_DISABLE = 0xfb, 0xfc, 0xfd
PARAM_READ = 0x300
PARAM_RUN_MODE, PARAM_CAN_TIMEOUT = 0x7005, 0x7028
MODE_RUN = 2
FAULT_POLL_S = 0.25
FAULT_BITS = {0: 'motor over-temperature', 1: 'driver chip fault', 2: 'under-voltage', 3: 'over-voltage',
              4: 'phase B over-current', 5: 'phase C over-current', 7: 'encoder not calibrated',
              8: 'hardware identification fault', 9: 'position initialisation fault', 14: 'stall or overload',
              16: 'phase A over-current'}


def fault_text(word):
    return ', '.join(FAULT_BITS.get(bit, f'fault bit {bit}') for bit in range(32) if word >> bit & 1)


class RobStrideArm(MitCanArm):
    def __init__(self, *args, host_id=0xfd, **kw):
        super().__init__(*args, **kw)
        self.host_id = host_id
        self._by_id = {m.id: m for m in self.motors}
        self._has_params = {}
        self._fault_poll_at = 0.0
        self._strangers = set()

    def can_frames_per_s(self):
        return super().can_frames_per_s() + 2 * len(self.motors) / FAULT_POLL_S     # and the fault poll while running

    def _op(self, opcode, argument=0xff):
        return b'\xff' * 6 + bytes([argument, opcode])

    def _enable_frame(self, motor): return self._op(OP_ENABLE)
    def _disable_frame(self, motor): return self._op(OP_DISABLE)

    def _decode(self, msg):
        d = bytes(msg.data)
        if msg.arbitration_id != self.host_id or len(d) not in (5, 8):
            return None
        motor = self._by_id.get(d[0])
        if motor is None:
            self._strangers.add(d[0])
            return None
        if len(d) == 5 or d[5:8] == bytes(3):                    # the fault word, zero-padded on newer firmware
            word = struct.unpack('<I', d[1:5])[0]
            if not word:
                return None
            last = self._seen.get(motor.name) or self._last.get(motor.name)
            return motor.name, Reading(last.pos if last else 0.0, fault=fault_text(word))
        pos, vel, torque = feedback_fields(d, *self._scale(motor))
        mode = d[6] >> 6
        fault = 'motor reports a fault' if d[6] & 0x20 else 'motor reports a warning' if d[6] & 0x10 else None
        # firmware without parameter access leaves the mode bits at zero, so only newer motors can say "disabled"
        enabled = mode == MODE_RUN if self._has_params.get(motor.name) else None
        return motor.name, Reading(pos, vel, torque, ((d[6] & 0xf) << 8 | d[7]) / 10.0, fault, enabled)

    def _ask_state(self):
        """A fault query first: the state request also clears faults, so it only goes to motors that have none."""
        faulted = set()
        for motor in self.motors:
            self._send(motor.id, self._op(OP_FAULT, 0x00))
        end = time.monotonic() + self.question_timeout_s / 3
        while (left := end - time.monotonic()) > 0:
            msg = self.can.recv(timeout=left)
            if msg is None:
                break
            item = self._decode(msg)
            if item is not None and item[1].fault:
                faulted.add(item[0])
                self._seen[item[0]] = item[1]
        for motor in self.motors:
            if motor.name not in faulted:
                self._send(motor.id, self._op(OP_FAULT))

    def _ask_about(self, motors):
        for motor in motors:
            self._send(motor.id, self._op(OP_FAULT, 0x00))

    def _read_states(self):
        now = time.monotonic()
        with self._io:
            if self._engaged and now >= self._fault_poll_at:     # flags in the feedback are absent on old firmware
                self._fault_poll_at = now + FAULT_POLL_S
                for motor in self.motors:
                    self._send(motor.id, self._op(OP_FAULT, 0x00))
            return super()._read_states()

    def _parameter(self, motor, index, fmt='<B3x'):
        self._send(PARAM_READ | motor.id, struct.pack('<H6x', index))
        end = time.monotonic() + self.question_timeout_s
        while (left := end - time.monotonic()) > 0:
            msg = self.can.recv(timeout=left)
            if msg is None:
                return None
            d = bytes(msg.data)
            if msg.arbitration_id == PARAM_READ | motor.id and len(d) == 8 and struct.unpack('<H', d[:2])[0] == index:
                return struct.unpack(fmt, d[4:8])[0]
        return None

    def _probe(self):
        rows, problems = {}, []
        self._strangers = set()
        with self._io:
            self._drain()
            for motor in self.motors:
                row = rows[motor.name] = dict(id=motor.id, model=motor.model)
                run_mode = self._parameter(motor, PARAM_RUN_MODE)
                self._has_params[motor.name] = run_mode is not None
                row['parameter_access'] = run_mode is not None
                if run_mode is not None:
                    row['run_mode'] = run_mode
                    if run_mode != 0:
                        problems.append(f'{motor.name}: run mode is {run_mode}, MIT control (0) is needed')
                self._send(motor.id, self._op(OP_FAULT, 0x00))
                answered, fault, end = False, None, time.monotonic() + self.question_timeout_s
                while (left := end - time.monotonic()) > 0:
                    msg = self.can.recv(timeout=left)
                    if msg is None:
                        break
                    if msg.arbitration_id == self.host_id and msg.data and msg.data[0] == motor.id:
                        answered = True
                        item = self._decode(msg)
                        fault = item[1].fault if item else None
                        break
                    self._decode(msg)
                if not answered:
                    problems.append(f'{motor.name}: no motor answers at CAN id {motor.id} (power, wiring, id, or still '
                                    'on the private protocol?)')
                elif fault:
                    problems.append(f'{motor.name}: {fault}')
        if self._strangers:
            problems.append(f'unexpected motor ids on the bus: {sorted(self._strangers)}')
        return rows, problems
