"""`roborun hw motor`: list the motors on a bus, change a motor's id, set its zero. Damiao, RobStride, Feetech, Dynamixel.

    roborun hw motor list damiao@can0 [--ids 1-16]          read-only: nothing is written, enabled or moved
    roborun hw motor set-id feetech@serial://SERIAL 1 7     asks first; torque must be off
    roborun hw motor set-zero damiao@can0 3                 asks first; the motor must be disabled

The bus may be named by motor family or by an arm that uses it (so101@serial://..., metal_arm@can0). Every write is made
only after the motor was read and found without torque, is then read back with a separate read, and says what it
changed. Nothing here ever enables a motor. The bus is refused while a station (or any Roborun program) holds it.
Ctrl-C at a question writes nothing; during a write it waits until that write has been read back.

Frames and registers: Damiao's manuals and DM_Control_Python (0x7ff register read 0x33 / write 0x55 / save 0xaa, ESC_ID 8,
MST_ID 7, set zero FF..FE); makermods-robotics/maker-arm-sdk docs/mit-protocol.md for RobStride (fault query, state, set
zero FF..FE); the STS3215 table (ID 5, homing offset 31, EEPROM lock 55); the Dynamixel X-series e-Manual (ID 7, homing
offset 20, present position 132).
"""
import contextlib
import json
import signal
import struct
import sys
import threading
import time

from .base import ArmHardwareError

FAMILIES = ('damiao', 'robstride', 'feetech', 'dynamixel')
DEFAULT_IDS = {'damiao': '1-16', 'robstride': '1-16', 'feetech': '1-20', 'dynamixel': '1-20'}


class Refused(ArmHardwareError):
    """A write that was not made, and why."""


@contextlib.contextmanager
def deferred_interrupt():
    """Ctrl-C inside waits until the block is done, then interrupts. A write is never cut between write and read-back."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    hit = []
    old = signal.signal(signal.SIGINT, lambda *_: hit.append(1))
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, old)
    if hit:
        raise KeyboardInterrupt


def parse_ids(text):
    out = []
    for part in str(text).split(','):
        lo, _, hi = part.strip().partition('-')
        out += list(range(int(lo, 0), int(hi or lo, 0) + 1))
    return out


# -- CAN families ----------------------------------------------------------------------------------------------------
class _CanTool:
    timeout_s = 0.02

    def __init__(self, address, bitrate=None):
        from .canbus import BITRATE
        self.address, self.bitrate, self.can = address, bitrate or BITRATE, None

    def __enter__(self):
        from .canbus import open_bus
        self.can = open_bus(self.address, bitrate=self.bitrate)
        self._drain()
        return self

    def __exit__(self, *_):
        self.can.shutdown()

    def _drain(self):
        while self.can.recv(timeout=0) is not None:
            pass

    def _send(self, arbitration_id, data):
        import can
        self.can.send(can.Message(arbitration_id=arbitration_id, data=bytes(data), is_extended_id=False))

    def _wait(self, match):
        end = time.monotonic() + self.timeout_s
        while (left := end - time.monotonic()) > 0:
            msg = self.can.recv(timeout=left)
            if msg is None:
                return None
            if match(msg):
                return msg
        return None


class DamiaoTool(_CanTool):
    family = 'damiao'
    zero_tolerance_rad = 0.01

    def _exchange(self, motor_id, command, rid, value=bytes(4), answer_from=None):
        from .damiao import PARAM_ID
        who = motor_id if answer_from is None else answer_from
        self._send(PARAM_ID, bytes([motor_id & 0xff, motor_id >> 8, command, rid]) + bytes(value))
        msg = self._wait(lambda m: len(m.data) == 8 and m.data[0] == who & 0xff and m.data[1] == who >> 8
                         and m.data[2] == command and m.data[3] == rid)
        return None if msg is None else bytes(msg.data[4:8])

    def _u32(self, motor_id, rid):
        from .damiao import CMD_READ
        data = self._exchange(motor_id, CMD_READ, rid)
        return None if data is None else struct.unpack('<I', data)[0]

    def _f32(self, motor_id, rid):
        from .damiao import CMD_READ
        data = self._exchange(motor_id, CMD_READ, rid)
        return None if data is None else struct.unpack('<f', data)[0]

    def _state(self, motor_id, master, pmax):
        """(status nibble, position rad) from the refresh request every mode answers, or None."""
        from .damiao import CMD_REFRESH, PARAM_ID
        from .mit_can import to_float
        self._send(PARAM_ID, bytes([motor_id & 0xff, motor_id >> 8, CMD_REFRESH, 0, 0, 0, 0, 0]))
        register = lambda d: d[0] == motor_id & 0xff and d[1] == motor_id >> 8 and d[2] in (0x33, 0x55)
        msg = self._wait(lambda m: m.arbitration_id == master and len(m.data) == 8 and not register(m.data))
        if msg is None:
            return None
        return msg.data[0] >> 4, to_float(msg.data[1] << 8 | msg.data[2], -pmax, pmax, 16)

    def read(self, motor_id):
        from .damiao import RID_CTRL_MODE, RID_MST_ID, RID_PMAX, mode_name
        master = self._u32(motor_id, RID_MST_ID)
        if master is None:
            return None
        row = dict(id=motor_id, master_id=master, control_mode=None, enabled=None)
        mode, pmax = self._u32(motor_id, RID_CTRL_MODE), self._f32(motor_id, RID_PMAX)
        if mode is not None:
            row['control_mode'] = mode_name(mode)
        if pmax:
            row['pmax'] = round(pmax, 3)
            state = self._state(motor_id, master, pmax)
            if state is not None:
                row.update(enabled=state[0] == 1, status=state[0], position_rad=round(state[1], 4))
        return row

    def scan(self, ids):
        return [row for row in (self.read(i) for i in ids) if row is not None]

    def _disabled(self, motor_id):
        row = self.read(motor_id)
        if row is None:
            raise Refused(f'no Damiao motor answers at id {motor_id:#x}: nothing was written')
        if row.get('enabled') is None:
            raise Refused(f'motor {motor_id:#x} did not report whether it has torque: nothing was written')
        if row['enabled']:
            raise Refused(f'motor {motor_id:#x} is enabled (has torque): nothing was written. Support the arm, '
                          'power-cycle it and run this again')
        return row

    def plan_set_id(self, old, new):
        row = self._disabled(old)
        if self.read(new) is not None:
            raise Refused(f'a motor already answers at id {new:#x}: two motors on one id cannot be told apart')
        master = row['master_id']
        new_master = new + (master - old) if master - old in (0x10, 0x80, 0x100) else master
        return dict(row=row, new_master=new_master,
                    text=f'Damiao motor {old:#x} -> id {new:#x}, feedback id {master:#x} -> {new_master:#x}. It is then '
                         "saved to the motor's flash (Damiao's save stores all its settings; without it the id returns "
                         'at power-off)')

    def set_id(self, old, new, plan, save=True):
        from .damiao import CMD_SAVE, CMD_WRITE, PARAM_ID, RID_ESC_ID, RID_MST_ID
        with deferred_interrupt():
            self._exchange(old, CMD_WRITE, RID_ESC_ID, struct.pack('<I', new), answer_from=new)   # answered as the new id
            if self._u32(new, RID_ESC_ID) != new:
                raise ArmHardwareError(f'motor {old:#x} did not take id {new:#x} (no answer at the new id): '
                                       'nothing was saved; power-cycle it and list the bus')
            if plan['new_master'] != plan['row']['master_id']:
                self._exchange(new, CMD_WRITE, RID_MST_ID, struct.pack('<I', plan['new_master']))
                if self._u32(new, RID_MST_ID) != plan['new_master']:
                    raise ArmHardwareError(f'motor {new:#x} took its new id but not feedback id {plan["new_master"]:#x}: '
                                           'nothing was saved; power-cycle it and list the bus')
            if save:
                self._send(PARAM_ID, bytes([new & 0xff, new >> 8, CMD_SAVE, 0, 0, 0, 0, 0]))
                time.sleep(0.05)
                self._drain()
            back = self.read(new)
        if back is None or back['master_id'] != plan['new_master']:
            raise ArmHardwareError(f'motor {new:#x} reads back as {back}: check it with roborun hw motor list')
        return dict(back, saved=save)

    def plan_set_zero(self, motor_id):
        row = self._disabled(motor_id)
        return dict(row=row, text=f'Damiao motor {motor_id:#x}: its present position '
                                  f'({row.get("position_rad")} rad) becomes its zero (the motor stores it)')

    def set_zero(self, motor_id, plan):
        with deferred_interrupt():
            self._send(motor_id, b'\xff' * 7 + b'\xfe')
            time.sleep(0.02)
            self._drain()
            back = self.read(motor_id)
        if back is None or back.get('position_rad') is None or abs(back['position_rad']) > self.zero_tolerance_rad:
            raise ArmHardwareError(f'motor {motor_id:#x} reads {back and back.get("position_rad")} rad after setting its '
                                   'zero, not 0: check it with roborun hw motor list')
        if back.get('enabled'):
            raise ArmHardwareError(f'motor {motor_id:#x} reports torque on: turn off its power')
        return back


class RobStrideTool(_CanTool):
    family = 'robstride'
    host_id = 0xfd
    zero_tolerance = 0.002                  # of the position range: the encoder's zero is the middle of the 16-bit field

    def _op(self, motor_id, argument, opcode):
        self._send(motor_id, b'\xff' * 6 + bytes([argument, opcode]))

    def _fault_word(self, motor_id):
        """The fault query: answered in every state and changes nothing."""
        self._op(motor_id, 0x00, 0xfb)
        msg = self._wait(lambda m: m.arbitration_id == self.host_id and len(m.data) in (5, 8) and m.data[0] == motor_id
                         and (len(m.data) == 5 or bytes(m.data[5:8]) == bytes(3)))
        return None if msg is None else struct.unpack('<I', bytes(msg.data[1:5]))[0]

    def _run_mode(self, motor_id):
        from .robstride import PARAM_READ, PARAM_RUN_MODE
        self._send(PARAM_READ | motor_id, struct.pack('<H6x', PARAM_RUN_MODE))
        msg = self._wait(lambda m: m.arbitration_id == PARAM_READ | motor_id and len(m.data) == 8
                         and struct.unpack('<H', bytes(m.data[:2]))[0] == PARAM_RUN_MODE)
        return None if msg is None else msg.data[4]

    def _feedback(self, motor_id):
        """The state request (it also clears a latched fault, so it is sent only to a motor whose fault word is 0)."""
        self._op(motor_id, 0xff, 0xfb)
        msg = self._wait(lambda m: m.arbitration_id == self.host_id and len(m.data) == 8 and m.data[0] == motor_id
                         and bytes(m.data[5:8]) != bytes(3))
        if msg is None:
            return None
        d = msg.data
        return dict(q=(d[1] << 8 | d[2]) / 65535 - 0.5, running=d[6] >> 6 == 2)

    def read(self, motor_id):
        word = self._fault_word(motor_id)
        if word is None:
            return None
        from .robstride import fault_text
        mode = self._run_mode(motor_id)
        return dict(id=motor_id, fault=fault_text(word) or None, parameter_access=mode is not None, run_mode=mode)

    def scan(self, ids):
        return [row for row in (self.read(i) for i in ids) if row is not None]

    def plan_set_id(self, old, new):
        raise Refused('RobStride motors on the MIT protocol have no documented id change: set the id with RobStride\'s '
                      'own tool, then run roborun hw motor list')

    def plan_set_zero(self, motor_id):
        row = self.read(motor_id)
        if row is None:
            raise Refused(f'no RobStride motor answers at id {motor_id}: nothing was written')
        if row['fault']:
            raise Refused(f'motor {motor_id} reports {row["fault"]}: nothing was written; fix that first')
        state = self._feedback(motor_id)
        if state is None:
            raise Refused(f'motor {motor_id} did not send its state: nothing was written')
        if row['parameter_access'] and state['running']:
            raise Refused(f'motor {motor_id} is enabled (has torque): nothing was written. Support the arm, power-cycle '
                          'it and run this again')
        note = '' if row['parameter_access'] else (' This firmware cannot report whether the motor has torque: support '
                                                   'the arm and make sure nothing else drives it.')
        return dict(row=row, text=f'RobStride motor {motor_id}: its present position becomes its zero (the motor '
                                  f'stores it).{note}')

    def set_zero(self, motor_id, plan):
        with deferred_interrupt():
            self._op(motor_id, 0xff, 0xfe)
            time.sleep(0.02)
            self._drain()
            state = self._feedback(motor_id)
        if state is None or abs(state['q']) > self.zero_tolerance:
            raise ArmHardwareError(f'motor {motor_id} does not read zero after setting it ({state}): check it with '
                                   'roborun hw motor list')
        return dict(id=motor_id, zeroed=True)


# -- serial families: the arms' own packet code, without an arm table -------------------------------------------------
def _serial_tool(base):
    class Tool(base):
        def __init__(self, address, baud=None):            # noqa: no table: only the protocol is used
            self.bus, self.baud = address, int(baud or base.baud)
            self.port, self._buffer, self._seen, self._missed = None, bytearray(), {}, {}
            self._io = threading.RLock()

        def __enter__(self):
            self._open_bus()
            return self

        def __exit__(self, *_):
            self._close_bus()
    Tool.__name__ = base.__name__ + 'Tool'
    return Tool


def _servo_tools():
    from .dynamixel import DynamixelArm
    from .feetech import FeetechArm

    class ServoTool:
        MID = 2047                                            # the encoder's middle, where LeRobot's homing puts the pose too

        def read(self, motor_id):
            data = self._read_register(motor_id, self.MODEL, 2)
            if data is None:
                return None
            torque = self._read_register(motor_id, self.TORQUE, 1)
            return dict(id=motor_id, model_number=int.from_bytes(data, 'little'),
                        torque_on=None if torque is None else bool(torque[0]), position=self.present(motor_id),
                        homing_offset=self.homing(motor_id))

        def scan(self, ids):
            return [row for row in (self.read(i) for i in ids) if row is not None]

        def _off(self, motor_id):
            row = self.read(motor_id)
            if row is None:
                raise Refused(f'no servo answers at id {motor_id}: nothing was written')
            if row['torque_on'] is not False:
                raise Refused(f'servo {motor_id} has torque on: nothing was written. Support the arm, power-cycle it '
                              '(or stop the program driving it) and run this again')
            return row

        def plan_set_id(self, old, new):
            row = self._off(old)
            if not 0 <= new <= 252:
                raise Refused(f'{new} is not a servo id (0-252)')
            if self.read(new) is not None:
                raise Refused(f'a servo already answers at id {new}: two servos on one id cannot be told apart')
            return dict(row=row, text=f'servo {old} -> id {new} (stored in the servo\'s EEPROM)')

        def set_id(self, old, new, plan, save=True):
            with deferred_interrupt():
                self._write_id(old, new)
                time.sleep(0.02)
                back = self.read(new)
                gone = self.read(old) is None
            if back is None or not gone or back['model_number'] != plan['row']['model_number']:
                raise ArmHardwareError(f'servo {old} did not move to id {new} (new id answers: {back is not None}, old id '
                                       f'silent: {gone}): check it with roborun hw motor list')
            return back

        def plan_set_zero(self, motor_id):
            row = self._off(motor_id)
            return dict(row=row, text=f'servo {motor_id}: its homing offset changes from {row["homing_offset"]} so this '
                                      f'pose reads {self.MID} (the middle of the encoder; stored in its EEPROM)')

        def set_zero(self, motor_id, plan):
            with deferred_interrupt():
                self._write_zero(motor_id)
                time.sleep(0.02)
                back = self.read(motor_id)
            if back is None or back['position'] is None or abs(back['position'] - self.MID) > 2:
                raise ArmHardwareError(f'servo {motor_id} reads {back and back["position"]} after setting its zero, not '
                                       f'{self.MID}: check it with roborun hw motor list')
            return back

    from .feetech import REG_HOMING, REG_LOCK, REG_PRESENT, WRITE, from_sign_magnitude, to_sign_magnitude

    class FeetechTool(ServoTool, _serial_tool(FeetechArm)):
        family = 'feetech'
        REG_ID = 5

        def present(self, motor_id):
            data = self._read_register(motor_id, REG_PRESENT, 2)
            return None if data is None else from_sign_magnitude(int.from_bytes(data, 'little'), 15)

        def homing(self, motor_id):
            data = self._read_register(motor_id, REG_HOMING, 2)
            return None if data is None else from_sign_magnitude(int.from_bytes(data, 'little'), 11)

        def _unlocked(self, motor_id, address, data, then_id=None):
            if not self._write_register(motor_id, REG_LOCK, b'\x00'):
                raise ArmHardwareError(f'servo {motor_id}: unlocking its EEPROM failed: nothing was written')
            self._send_packet(motor_id, WRITE, bytes([address]) + bytes(data))   # an id change answers as either id
            time.sleep(0.01)
            self.port.reset_input_buffer()
            self._buffer.clear()
            self._write_register(motor_id if then_id is None else then_id, REG_LOCK, b'\x01')

        def _write_id(self, old, new):
            self._unlocked(old, self.REG_ID, bytes([new]), then_id=new)

        def _write_zero(self, motor_id):
            actual = (self.present(motor_id) + self.homing(motor_id)) % self.counts_per_turn
            offset = max(-self.MID, min(self.MID, actual - self.MID))
            self._unlocked(motor_id, REG_HOMING, to_sign_magnitude(offset, 11).to_bytes(2, 'little'))

    from .dynamixel import WRITE as DXL_WRITE

    class DynamixelTool(ServoTool, _serial_tool(DynamixelArm)):
        family = 'dynamixel'
        REG_ID, REG_HOMING, REG_POSITION = 7, 20, 132
        HOMING_LIMIT = 1044479

        def __init__(self, *args, **kw):
            super().__init__(*args, **kw)
            self.shadows, self._shadow_missed = [], {}

        def present(self, motor_id):
            data = self._read_register(motor_id, self.REG_POSITION, 4)
            return None if data is None else int.from_bytes(data, 'little', signed=True)

        def homing(self, motor_id):
            data = self._read_register(motor_id, self.REG_HOMING, 4)
            return None if data is None else int.from_bytes(data, 'little', signed=True)

        def _write_id(self, old, new):
            self._send_packet(old, DXL_WRITE, self.REG_ID.to_bytes(2, 'little') + bytes([new]))

        def _write_zero(self, motor_id):
            actual = self.present(motor_id) - self.homing(motor_id)        # present = actual + homing offset
            offset = max(-self.HOMING_LIMIT, min(self.HOMING_LIMIT, self.MID - actual % self.counts_per_turn))
            if not self._write_register(motor_id, self.REG_HOMING, offset.to_bytes(4, 'little', signed=True)):
                raise ArmHardwareError(f'servo {motor_id}: writing its homing offset failed')

    return dict(feetech=FeetechTool, dynamixel=DynamixelTool)


def tool_for(spec, baud=None):
    """'damiao@can0' or an arm that uses that family: 'so101@serial://...' -> an open-able tool."""
    from . import catalog, parse_spec
    name, address = parse_spec(spec)
    if not address:
        raise SystemExit(f'name the bus too: {name}@ADDRESS (`roborun hw detect` prints them)')
    family = name
    if family not in FAMILIES:
        driver = (catalog().get(name) or {}).get('driver') or ''
        family = next((f for f in FAMILIES if f'.{f}:' in driver), None)
        if family is None:
            raise SystemExit(f'{name}: use one of {", ".join(FAMILIES)}, or an arm built on one of them')
    if family == 'damiao':
        return DamiaoTool(address)
    if family == 'robstride':
        return RobStrideTool(address)
    return _servo_tools()[family](address, baud)


# -- the command -----------------------------------------------------------------------------------------------------
def confirm(text, assume_yes=False, ask=None):
    if assume_yes:
        return True
    if ask is None:
        if not sys.stdin.isatty():
            raise SystemExit('this changes a motor, so it asks first and needs a terminal; or pass --yes')
        ask = input
    return ask(f'{text}\nType yes to write it: ').strip().lower() == 'yes'


def cmd_motor(a, ask=None, say=print) -> int:
    from .safety import ArmsBusy, ArmsLock
    from . import parse_spec
    tool = tool_for(a.bus, getattr(a, 'baud', None))
    address = parse_spec(a.bus)[1]
    try:
        with ArmsLock(f'roborun motor {a.action}', [address]), tool:
            if a.action == 'list':
                ids = parse_ids(a.ids or DEFAULT_IDS[tool.family])
                rows = tool.scan(ids)
                if a.json:
                    say(json.dumps(dict(bus=a.bus, family=tool.family, motors=rows), indent=1))
                else:
                    say(f'{len(rows)} {tool.family} motor(s) answer on {address} (ids {ids[0]}-{ids[-1]} asked; nothing '
                        'was written):')
                    for row in rows:
                        say('  ' + ', '.join(f'{k} {v}' for k, v in row.items()))
                return 0
            try:
                if a.action == 'set-id':
                    plan = tool.plan_set_id(a.id, a.new_id)
                else:
                    plan = tool.plan_set_zero(a.id)
            except Refused as exc:
                raise SystemExit(f'{a.bus}: {exc}') from None
            if not confirm(plan['text'], a.yes, ask):
                say('Nothing was written.')
                return 1
            if a.action == 'set-id':
                result = tool.set_id(a.id, a.new_id, plan, save=not getattr(a, 'no_save', False))
            else:
                result = tool.set_zero(a.id, plan)
            say(json.dumps(dict(done=a.action, **result), default=str))
            return 0
    except ArmsBusy as exc:
        raise SystemExit(str(exc)) from None
    except KeyboardInterrupt:
        say('Stopped. Any write that had started was finished and read back first; run roborun hw motor list to see the bus.')
        return 130


def add_parser(sub):
    m = sub.add_parser('motor', help='motors on a bus: list them (read-only), change an id, set a zero (each asks first)')
    acts = m.add_subparsers(dest='action', required=True)
    for name, text in (('list', 'every motor that answers, read-only'), ('set-id', 'give a motor another id'),
                       ('set-zero', "make a motor's present position its zero")):
        p = acts.add_parser(name, help=text)
        p.add_argument('bus', help='FAMILY@ADDRESS (damiao, robstride, feetech, dynamixel) or ARM@ADDRESS')
        p.add_argument('--baud', type=int, help='serial servos: the bus speed (default 1000000)')
        if name == 'list':
            p.add_argument('--ids', help='ids to ask, e.g. 1-16 or 1,2,7 (default depends on the family)')
            p.add_argument('--json', action='store_true')
        else:
            p.add_argument('id', type=lambda v: int(v, 0))
            if name == 'set-id':
                p.add_argument('new_id', type=lambda v: int(v, 0))
                p.add_argument('--no-save', action='store_true', help="Damiao: do not save to the motor's flash (the id "
                                                                      'returns at power-off)')
            p.add_argument('--yes', action='store_true', help='do not ask (scripts)')
    m.set_defaults(func=cmd_motor)


# -- an arm's motors in the wrong control mode: shown, asked, written, read back ---------------------------------------
def offer_repairs(arm, yes, say=print, save=True):
    """For `roborun hw check --repair` and the guided setup. `yes(question, default)` answers; True when repaired."""
    if not hasattr(arm, 'plan_repairs'):
        say(f'{getattr(arm, "arm_id", "this arm")}: nothing here that Roborun can repair')
        return False
    repairs, other = arm.plan_repairs()
    if other:
        say('Not offering a repair while the arm has other problems (a bus or motor in doubt is not one to write to):')
        for text in other:
            say(f'  - {text}')
        return False
    if not repairs:
        say('Every motor is already in the mode Roborun drives it in: nothing to repair.')
        return False
    from .damiao import mode_name
    say('These motors are in a control mode Roborun does not drive them in:')
    for item in repairs:
        say(f'  - {item["motor"]} (id {item["id"]:#x}): {item["register"]} {mode_name(item["now"])} -> {mode_name(item["to"])}')
    say('Every motor is read first and must have no torque; nothing is enabled or moved. Each value is read back with a '
        'separate read' + (", then saved to the motor's flash so it lasts across power-off (Damiao's save stores all of "
                           'the motor\'s settings; the others were just read and match this arm\'s table).' if save else
                           ', and not saved: it returns at the next power-off.'))
    if not yes('Write it?', False):
        say('Nothing was written.')
        return False
    arm.apply_repairs(repairs, save=save, say=say)
    return True
