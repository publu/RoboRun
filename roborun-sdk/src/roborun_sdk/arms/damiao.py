"""Damiao DM-J motors over CAN: Metal Arm, reBot B601-DM, OpenArm 1.0 and v2 (tables in data/arms).

Commands go to the motor's id, state returns on its master id (`recv`). Frame facts: Damiao's manuals, DM_Control_Python,
enactic/openarm_can and makermods-robotics/metal-python-ros. Scale registers are read from every motor and must match
the table: the same bytes mean different radians on a motor set up with other ranges.

Control mode (register 10: 1 MIT, 2 position-speed, 3 speed, 4 position-force). The joints run in MIT mode. A motor in
another mode is never written to on the way up: `roborun hw check ARM@ADDRESS --repair` (or the guided setup) shows
what it would change, asks, and then writes register 10 with every motor disabled, reads it back with a separate read,
and saves it to the motor's flash so it survives a power cycle.

A jaw left in position-force mode (as Metal grippers ship) is driven in that mode instead of being changed: DM_Control_
Python's `control_pos_force` frame on 0x300 + id carries a position, a speed limit (rad/s x 100) and a current limit
(per unit of the motor's maximum current x 10000), so the motor's own current loop caps the grip force even if this
computer stops talking to it, and the maker's software that expects that mode keeps working.
"""
import shlex
import struct
import time

from .base import ArmHardwareError
from .canbus import fd_commands
from .mit_can import MitCanArm, feedback_fields
from .motor_arm import Reading

PARAM_ID = 0x7ff
CMD_READ, CMD_WRITE, CMD_SAVE, CMD_REFRESH = 0x33, 0x55, 0xaa, 0xcc
RID_MST_ID, RID_ESC_ID, RID_TIMEOUT, RID_CTRL_MODE, RID_PMAX, RID_VMAX, RID_TMAX = 7, 8, 9, 10, 21, 22, 23
MODE_MIT, MODE_POS_FORCE = 1, 4
MODES = {1: 'MIT', 2: 'position-speed', 3: 'speed', 4: 'position-force'}
POS_FORCE_BASE = 0x300
SAVE_SETTLE_S = 0.05
ERRORS = {8: 'over-voltage', 9: 'under-voltage', 10: 'over-current', 11: 'driver over-temperature',
          12: 'motor over-temperature', 13: 'communication lost', 14: 'overload'}


def mode_name(mode):
    return f'{mode} ({MODES.get(mode, "unknown")})'


class DamiaoArm(MitCanArm):
    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self._by_recv = {int(m.spec['recv']): m for m in self.motors}
        self._pos_force = set()
        self.repairs = []

    def _enable_frame(self, motor): return b'\xff' * 7 + b'\xfc'
    def _disable_frame(self, motor): return b'\xff' * 7 + b'\xfd'

    def _ask_state(self):
        self._ask_about(self.motors)

    def _ask_about(self, motors):
        for motor in motors:
            self._send(PARAM_ID, bytes([motor.id & 0xff, motor.id >> 8, CMD_REFRESH, 0, 0, 0, 0, 0]))

    def _decode(self, msg):
        motor = self._by_recv.get(msg.arbitration_id)
        if motor is None or len(msg.data) != 8:
            return None
        d = msg.data
        if d[2] in (CMD_READ, CMD_WRITE) and d[0] == motor.id & 0xff and d[1] == motor.id >> 8:   # a late register answer
            return None
        status = d[0] >> 4
        pos, vel, torque = feedback_fields(d, *self._scale(motor))
        fault = ERRORS.get(status, f'unknown status {status:#x}') if status >= 8 else None
        return motor.name, Reading(pos, vel, torque, float(max(d[6], d[7])), fault, status == 1)

    # -- position-force jaw --------------------------------------------------------------------------------------
    def _mit(self, motor, position, kp, kd):
        if motor.name not in self._pos_force:
            return super()._mit(motor, position, kp, kd)
        share = 0.0 if kp <= 0 or motor.kp <= 0 else min(1.0, kp / motor.kp)      # the stiffness ramp eases the current in
        current = self._pos_force_current(motor) * share
        speed = float(motor.spec.get('pos_force_speed', 2.0))
        pmax = self._scale(motor)[0]
        data = struct.pack('<fHH', max(-pmax, min(pmax, float(position))), max(0, min(10000, round(speed * 100))),
                           max(0, min(10000, round(current * 10000))))
        self._send(POS_FORCE_BASE + motor.id, data)

    def _pos_force_current(self, motor):
        """The current limit, as a share of the motor's maximum: the table's `pos_force_current`, else the jaw's torque
        cap over the motor's torque range. Scaled by this arm's grip effort, like the MIT jaw's cap."""
        share = motor.spec.get('pos_force_current')
        if share is None:
            cap, tmax = motor.spec.get('max_torque_nm'), self._scale(motor)[2]
            share = cap / tmax if cap and tmax else 0.1
        share = float(share) * (self._jaw_scale() if motor is self.gripper else 1.0)
        return max(0.0, min(1.0, share))

    # -- registers -----------------------------------------------------------------------------------------------
    def _exchange(self, motor, command, rid, value=bytes(4), answer_from=None):
        """One register frame on 0x7ff; (value bytes, the CAN id it came from) or None. A write is answered with what
        the motor holds now, and an id change by the motor under its new id, so `answer_from` names whom to wait for."""
        who = motor.id if answer_from is None else answer_from
        self._send(PARAM_ID, bytes([motor.id & 0xff, motor.id >> 8, command, rid]) + bytes(value))
        end = time.monotonic() + self.question_timeout_s
        while (left := end - time.monotonic()) > 0:
            msg = self.can.recv(timeout=left)
            if msg is None:
                return None
            d = msg.data
            if len(d) == 8 and d[0] == who & 0xff and d[1] == who >> 8 and d[2] == command and d[3] == rid:
                return bytes(d[4:8]), msg.arbitration_id
        return None

    def _register(self, motor, rid):
        return self._exchange(motor, CMD_READ, rid)

    def _status(self, motor):
        """The motor's state frame, asked for with the refresh request every mode answers. Changes nothing."""
        self._ask_about([motor])
        end = time.monotonic() + self.question_timeout_s
        while (left := end - time.monotonic()) > 0:
            msg = self.can.recv(timeout=left)
            item = None if msg is None else self._decode(msg)
            if msg is None:
                return None
            if item is not None and item[0] == motor.name:
                return item[1]
        return None

    def _probe(self):
        rows, problems = {}, []
        self._pos_force, self.repairs = set(), []
        with self._io:
            self._drain()
            for motor in self.motors:
                recv, row = int(motor.spec['recv']), dict(id=motor.id, recv=motor.spec['recv'], model=motor.model)
                rows[motor.name] = row
                first = self._register(motor, RID_PMAX)
                if first is None:
                    problems.append(f'{motor.name}: no motor answers at CAN id {motor.id:#x} (power, wiring, id?)')
                    continue
                if first[1] != recv:
                    problems.append(f'{motor.name}: motor {motor.id:#x} answers on {first[1]:#x}, the table says {recv:#x}')
                    continue
                values = {RID_PMAX: first[0]}
                for rid in (RID_VMAX, RID_TMAX, RID_CTRL_MODE, RID_TIMEOUT):
                    answer = self._register(motor, rid)
                    values[rid] = None if answer is None else answer[0]
                if any(v is None for v in values.values()):
                    problems.append(f'{motor.name}: motor {motor.id:#x} did not answer its register reads')
                    continue
                found = [struct.unpack('<f', values[r])[0] for r in (RID_PMAX, RID_VMAX, RID_TMAX)]
                mode, timeout = (struct.unpack('<I', values[r])[0] for r in (RID_CTRL_MODE, RID_TIMEOUT))
                row.update(pmax=round(found[0], 3), vmax=round(found[1], 3), tmax=round(found[2], 3), control_mode=mode,
                           can_timeout=timeout)
                wanted = self._scale(motor)
                if any(abs(a - b) > .02 * b for a, b in zip(found, wanted)):
                    problems.append(f'{motor.name}: wrong motor or setup at id {motor.id:#x}: it maps position/speed/'
                                    f'torque to {[round(v, 2) for v in found]}, the table says {list(wanted)}')
                    continue
                if mode == MODE_MIT:
                    continue
                if mode == MODE_POS_FORCE and motor.role == 'gripper' and motor.spec.get('pos_force', True) is not False:
                    self._pos_force.add(motor.name)
                    row['drive'] = 'position-force: driven in the mode it is in, its current limit caps the grip'
                    continue
                self.repairs.append(dict(motor=motor.name, id=motor.id, register='CTRL_MODE', address=RID_CTRL_MODE,
                                         now=mode, to=MODE_MIT))
                problems.append(f'{motor.name}: control mode is {mode_name(mode)}, MIT (1) is needed. Next: roborun '
                                f'hw check {self.arm_id}@{self.bus} --repair (it shows the change and asks first)')
        if rows and all(p.split(':', 1)[0] in rows and 'no motor answers' in p for p in problems) and len(problems) == len(rows) \
                and not self._fd and '://' not in (self.bus or ''):
            problems.append(f'no motor answered on {self.bus} in classic CAN: motors set up for CAN FD (OpenArm\'s -fd '
                            'setup) answer only FD frames. If yours are, run: ' + ' && '.join(
                                'sudo ' + shlex.join(c) for c in fd_commands(self.bus)) + ', then try again')
        return rows, problems

    def survey(self):
        out = super().survey()
        if self.repairs:
            out['repairs'] = [dict(r) for r in self.repairs]
        return out

    # -- the confirmed repair ------------------------------------------------------------------------------------
    def plan_repairs(self):
        """Read-only: (repairs, problems no repair fixes). Nothing is offered unless every motor answered and only its
        control mode is wrong, as i2rt's startup check does: a bus or motor in doubt is not one to write to."""
        self._open_bus()
        try:
            _, problems = self._probe()
        finally:
            self._close_bus()
        repairable = {r['motor'] for r in self.repairs}
        other = [p for p in problems if p.split(':', 1)[0] not in repairable]
        return [dict(r) for r in self.repairs], other

    def apply_repairs(self, repairs, save=True, say=print):
        """Writes each repair with every motor disabled, then reads it back with a separate read. `save` then stores it
        in flash (Damiao's save command stores all of a motor's settings; the others were just read and match the table)
        and reads it back again. Ctrl-C waits for the motor being written to finish and be read back."""
        from .motor_tools import deferred_interrupt
        done = []
        self._open_bus()
        try:
            with self._io:
                self._drain()
                by_name = {m.name: m for m in self.motors}
                for motor in self.motors:                  # nothing is written while any motor of the arm has torque
                    state = self._status(motor)
                    if state is None:
                        raise ArmHardwareError(f'{motor.name} (id {motor.id:#x}) did not answer: nothing was written')
                    if state.enabled:
                        raise ArmHardwareError(f'{motor.name} (id {motor.id:#x}) is enabled (has torque): nothing was '
                                               'written. Support the arm, power-cycle it, and run this again')
                for item in repairs:
                    motor = by_name[item['motor']]
                    now = self._register(motor, item['address'])
                    if now is None or struct.unpack('<I', now[0])[0] != item['now']:
                        raise ArmHardwareError(f'{motor.name}: its {item["register"]} changed since it was read: nothing '
                                               'more was written')
                    with deferred_interrupt():
                        self._exchange(motor, CMD_WRITE, item['address'], struct.pack('<I', item['to']))
                        back = self._register(motor, item['address'])       # a separate read, never the write's echo
                        value = None if back is None else struct.unpack('<I', back[0])[0]
                        if value != item['to']:
                            raise ArmHardwareError(
                                f'{motor.name}: wrote {item["register"]} = {item["to"]}, it reads back as {value}: '
                                'the motor did not take it, nothing was saved' + self._done_text(done))
                        if save:
                            self._send(PARAM_ID, bytes([motor.id & 0xff, motor.id >> 8, CMD_SAVE, 0, 0, 0, 0, 0]))
                            time.sleep(SAVE_SETTLE_S)
                            self._drain()
                            back = self._register(motor, item['address'])
                            value = None if back is None else struct.unpack('<I', back[0])[0]
                            if value != item['to']:
                                raise ArmHardwareError(f'{motor.name}: after saving, {item["register"]} reads {value}: '
                                                       'check it with roborun hw check' + self._done_text(done))
                    done.append(item['motor'])
                    say(f'{motor.name}: {item["register"]} {mode_name(item["now"])} -> {mode_name(item["to"])}, read back'
                        + (', saved to the motor\'s flash' if save else ' (not saved: it returns at power-off)'))
                for motor in self.motors:
                    state = self._status(motor)
                    if state is not None and state.enabled:
                        raise ArmHardwareError(f'{motor.name} reports torque on after the write: turn off the arm\'s power')
        finally:
            self._close_bus()
        return done

    @staticmethod
    def _done_text(done):
        return f' (already changed: {", ".join(done)})' if done else ''
