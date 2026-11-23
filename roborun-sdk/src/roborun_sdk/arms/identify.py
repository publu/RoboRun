"""Which arm is on this adapter? Asked of the motors without changing anything on them.

Nothing here enables a motor, sends a position, or writes a register. A CAN bus is first listened to; if something else is
already talking on it, nothing is sent at all. The questions are then asked from the safest down:

    Damiao      register reads on 0x7FF, which any Damiao motor answers in any state and no other motor family acts on
    RobStride   parameter reads on 0x300|id, asked only when no Damiao motor answered (on a Damiao bus that id range is
                a command channel). The fault query other code uses is not sent here: to an unknown motor in MIT mode its
                bytes could read as a position target
    Feetech     register reads of the model number
    Dynamixel   register reads of the model number (protocol 2.0)

An answer narrows the list of arms; it never picks one. The person confirms, and the arm's own read-only check follows.
"""
import struct
import time
from dataclasses import dataclass, field

import can

from . import canbus, catalog
from .damiao import PARAM_ID, RID_CTRL_MODE, RID_PMAX, RID_TMAX, RID_VMAX
from .robstride import PARAM_READ, PARAM_RUN_MODE

CAN_IDS = range(1, 17)
FEETECH_IDS = range(1, 13)
DYNAMIXEL_IDS = range(1, 21)
RID_GEAR = 20
LISTEN_S = 0.12
REPLY_S = 0.012
SERIAL_REPLY_S = 0.02
TOLERANCE = 0.02
DAMIAO_NAMES = {7: 'DM3507', 9: 'DM8009', 10: 'DM4310', 40: 'DM4340', 48: 'DM6248'}
FAMILY_LABEL = dict(damiao='Damiao', robstride='RobStride', feetech='Feetech', dynamixel='Dynamixel')
DRIVER_FAMILY = dict(YamArm='damiao', DamiaoArm='damiao', RobStrideArm='robstride', FeetechArm='feetech', DynamixelArm='dynamixel')


@dataclass
class Probe:
    address: str
    bus: str                            # can | serial
    family: str | None = None
    motors: dict = field(default_factory=dict)      # {id: {kind, ...what was read}}
    busy: bool = False                  # something else is talking on this bus: nothing was sent
    error: str | None = None
    notes: list = field(default_factory=list)

    def summary(self):
        if self.error:
            return self.error
        if self.busy:
            return 'another program or controller is already talking on this bus, so nothing was sent to it'
        if not self.motors:
            return 'no motor answered'
        kinds = {}
        for row in self.motors.values():
            kinds[row.get('kind') or '?'] = kinds.get(row.get('kind') or '?', 0) + 1
        ids = ', '.join(str(i) for i in sorted(self.motors))
        what = ', '.join(f'{count} x {kind}' for kind, count in sorted(kinds.items()))
        return f'{len(self.motors)} {FAMILY_LABEL[self.family]} motors answered (ids {ids}): {what}'


# -- CAN ---------------------------------------------------------------------------------------------------------------
def _send(bus, arbitration, data):
    bus.send(can.Message(arbitration_id=arbitration, data=bytes(data), is_extended_id=False))


def _damiao_register(bus, motor, rid, window=REPLY_S):
    _send(bus, PARAM_ID, [motor & 0xff, motor >> 8, 0x33, rid, 0, 0, 0, 0])
    end = time.monotonic() + window
    while (left := end - time.monotonic()) > 0:
        msg = bus.recv(timeout=left)
        if msg is None:
            return None
        d = bytes(msg.data)
        if len(d) == 8 and d[0] == motor & 0xff and d[1] == motor >> 8 and d[2] == 0x33 and d[3] == rid:
            return d[4:8], msg.arbitration_id
    return None


def _robstride_parameter(bus, motor, window=REPLY_S):
    _send(bus, PARAM_READ | motor, struct.pack('<H6x', PARAM_RUN_MODE))
    end = time.monotonic() + window
    while (left := end - time.monotonic()) > 0:
        msg = bus.recv(timeout=left)
        if msg is None:
            return None
        d = bytes(msg.data)
        if msg.arbitration_id == PARAM_READ | motor and len(d) == 8 and struct.unpack('<H', d[:2])[0] == PARAM_RUN_MODE:
            return d[4]
    return None


def _quiet(bus):
    end = time.monotonic() + LISTEN_S
    while (left := end - time.monotonic()) > 0:
        if bus.recv(timeout=left) is not None:
            return False
    return True


def _float(raw):
    return None if raw is None else round(struct.unpack('<f', raw[0])[0], 3)


def _ask_damiao(bus, ids):
    found = {}
    for _ in range(2):                                     # a second pass for a reply lost to a busy adapter
        for motor in ids:
            if motor in found:
                continue
            first = _damiao_register(bus, motor, RID_PMAX)
            if first is not None:
                found[motor] = dict(pmax=_float(first), recv=first[1])
    for motor, row in found.items():
        for name, rid in (('vmax', RID_VMAX), ('tmax', RID_TMAX), ('gear', RID_GEAR)):
            row[name] = _float(_damiao_register(bus, motor, rid, REPLY_S * 3))
        mode = _damiao_register(bus, motor, RID_CTRL_MODE, REPLY_S * 3)
        row['control_mode'] = None if mode is None else struct.unpack('<I', mode[0])[0]
        gear = row.get('gear')
        row['kind'] = DAMIAO_NAMES.get(round(gear)) if gear else None
        row['kind'] = row['kind'] or f"Damiao ({row['pmax']:g}/{row['vmax'] or 0:g}/{row['tmax'] or 0:g})"
    return found


def probe_can(address, ids=CAN_IDS):
    out = Probe(address, 'can')
    bus = canbus.open_bus(address)
    try:
        if not _quiet(bus):
            out.busy = True
            return out
        try:
            motors = _ask_damiao(bus, ids)
            if motors:
                out.family, out.motors = 'damiao', motors
                return out
            found = {}
            for _ in range(2):
                for motor in ids:
                    if motor not in found:
                        mode = _robstride_parameter(bus, motor)
                        if mode is not None:
                            found[motor] = dict(kind='RobStride', run_mode=mode)
            if found:
                out.family, out.motors = 'robstride', found
        except can.CanError as exc:
            out.notes.append(f'the adapter could not send ({exc}): nothing acknowledged the frames, which usually means the '
                             'motors have no power or the CAN wires are not connected')
    finally:
        bus.shutdown()
    return out


# -- serial ------------------------------------------------------------------------------------------------------------
def _bare(kind, port):
    arm = kind.__new__(kind)
    arm.port, arm._buffer, arm.reply_timeout_s = port, bytearray(), SERIAL_REPLY_S
    return arm


def _ask_servos(kind, port, ids, names):
    arm, found = _bare(kind, port), {}
    for _ in range(2):
        for motor in ids:
            if motor in found:
                continue
            data = arm._read_register(motor, arm.MODEL, 2)
            if data is not None:
                number = int.from_bytes(data, 'little')
                torque = arm._read_register(motor, arm.TORQUE, 1)
                found[motor] = dict(model_number=number, kind=names.get(number, f'model {number}'),
                                    torque_on=None if torque is None else bool(torque[0]))
    return found


def probe_serial(address):
    from .dynamixel import MODELS, DynamixelArm
    from .feetech import MODEL_STS3215, FeetechArm
    from .serial_servo import open_port
    out = Probe(address, 'serial')
    port = open_port(address, 1_000_000)
    try:
        for family, kind, ids, names in (('feetech', FeetechArm, FEETECH_IDS, {MODEL_STS3215: 'STS3215'}),
                                         ('dynamixel', DynamixelArm, DYNAMIXEL_IDS, {v: k.upper() for k, v in MODELS.items()})):
            motors = _ask_servos(kind, port, ids, names)
            if motors:
                out.family, out.motors = family, motors
                break
    finally:
        port.close()
    return out


def probe(row):
    """What answers on one row of `roborun hw detect`. Failures come back in the Probe: this never raises for hardware."""
    address, bus = row['address'], 'serial' if row['bus'] == 'serial' else 'can'
    try:
        return probe_serial(address) if bus == 'serial' else probe_can(address)
    except Exception as exc:
        from .connect import explain
        return Probe(address, bus, error=explain(exc, address))


# -- what each catalog arm should look like ------------------------------------------------------------------------------
def family_of(row):
    return DRIVER_FAMILY.get(str(row.get('driver') or '').rpartition(':')[2])


def _yam_motors(row):
    from .yam import ArmType, GripperType, _load_arm_config, _yam_motor_list
    from i2rt.motor_drivers.utils import MotorType
    options = row.get('options') or {}
    arm, gripper = ArmType.from_string_name(options.get('arm_type', 'yam')), GripperType.from_string_name(options.get('gripper_type', 'linear_4310'))
    _load_arm_config(arm)
    out = {}
    for motor, kind in _yam_motor_list(arm, gripper):
        c = MotorType.get_motor_constants(kind)
        out[int(motor)] = dict(kind=kind, pmax=c.POSITION_MAX, vmax=c.VELOCITY_MAX, tmax=c.TORQUE_MAX, gear=MotorType.get_gear_ratio(kind))
    return out


def expected_motors(row):
    """{motor id: what it should read as} for a catalog arm this module can recognise, else None."""
    family, options = family_of(row), row.get('options') or {}
    if family is None:
        return None
    if str(row['driver']).endswith(':YamArm'):
        return _yam_motors(row)
    rows = [r for r in options.get('joints', ()) if 'fixed' not in r] + [options['gripper']] + list(options.get('held', ()))
    out = {}
    for r in rows:
        if family == 'damiao':
            out[int(r['id'])] = dict(kind=r.get('model'), pmax=r['pmax'], vmax=r['vmax'], tmax=r['tmax'], recv=int(r['recv']))
        elif family == 'robstride':
            out[int(r['id'])] = dict(kind=r.get('model'))
        elif family == 'feetech':
            from .feetech import MODEL_STS3215
            out[int(r['id'])] = dict(kind=r.get('model'), model_number=MODEL_STS3215)
        else:
            from .dynamixel import MODELS
            out[int(r['id'])] = dict(kind=r.get('model'), model_number=MODELS[r['model']])
    if family == 'dynamixel':
        for s in options.get('shadows', ()):
            out[int(s['id'])] = dict(kind=s.get('model'), model_number=MODELS[s['model']])
    return out


def _same(seen, wanted):
    for key in ('pmax', 'vmax', 'tmax', 'gear'):
        if seen.get(key) is not None and wanted.get(key) is not None and abs(seen[key] - wanted[key]) > TOLERANCE * abs(wanted[key]):
            return False
    if 'recv' in wanted and seen.get('recv') is not None and seen['recv'] != wanted['recv']:
        return False
    return 'model_number' not in wanted or seen.get('model_number') == wanted['model_number']


@dataclass
class Match:
    arm: str
    level: str                          # exact | extra | partial | family
    why: str

    ORDER = dict(exact=0, extra=1, partial=2, family=3)


def rank(found: Probe, rows=None):
    """Catalog arms this probe could be, best first. Empty when nothing was read."""
    rows = rows if rows is not None else catalog()
    if not found.motors:
        return []
    out = []
    for arm, row in rows.items():
        if family_of(row) != found.family:
            continue
        try:
            wanted = expected_motors(row)
        except Exception:
            continue
        if not wanted:
            continue
        missing = sorted(set(wanted) - set(found.motors))
        wrong = sorted(i for i in set(wanted) & set(found.motors) if not _same(found.motors[i], wanted[i]))
        extra = sorted(set(found.motors) - set(wanted))
        if wrong:
            out.append(Match(arm, 'family', f'same motor family, but motors {_ids(wrong)} are not what this arm uses'))
        elif not missing and not extra:
            out.append(Match(arm, 'exact', 'every motor matches'))
        elif not missing:
            out.append(Match(arm, 'extra', f'every motor matches; ids {_ids(extra)} are extra on this bus'))
        elif len(missing) <= len(wanted) / 2:
            out.append(Match(arm, 'partial', f'motors {_ids(missing)} did not answer (no power, a loose cable, or another arm)'))
        else:
            out.append(Match(arm, 'family', f'same motor family, but {len(missing)} of its {len(wanted)} motors did not answer'))
    return sorted(out, key=lambda m: (Match.ORDER[m.level], m.arm))


def _ids(values):
    return ', '.join(str(v) for v in values)
