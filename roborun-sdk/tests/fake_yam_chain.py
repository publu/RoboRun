"""A simulated YAM on a Damiao motor chain (python-can's virtual bus), plus simulated USB adapters in front of it.

It speaks the wire protocol the real motors speak (its codec is written here from the protocol, not borrowed from the
driver under test): enable / disable / clear-error frames, MIT commands with a feedback reply on id + 0x10, register
reads and writes on 0x7FF, the loss-of-comms timeout. Behind the wire is a joint-level plant: gravity from the arm's
own model, inertia, friction, jaw stops. An enabled arm whose gravity torque is missing really sags here.
"""
import collections
import math
import os
import queue
import struct
import threading
import time
import tty
from types import SimpleNamespace

import can
import numpy as np
import usb.core

from roborun_sdk.arms import yam as _yam  # noqa: F401  (puts the vendored i2rt on sys.path)
from i2rt.robots.utils import ArmType, GripperType, _load_arm_config, combine_arm_and_gripper_xml  # noqa: E402
from i2rt.utils.mujoco_utils import MuJoCoKDL  # noqa: E402

BROADCAST = 0x7FF
FEEDBACK_OFFSET = 0x10
SCALE = {'DM4310': (12.5, 30.0, 10.0, 10.0), 'DM4340': (12.5, 10.0, 28.0, 40.0), 'DM6248': (12.5, 20.0, 120.0, 48.0),
         'DM3507': (12.5, 50.0, 5.0, 7.0)}         # PMAX, VMAX, TMAX, gear ratio
REGISTER_NAMES = {7: 'MST_ID', 8: 'ESC_ID', 9: 'TIMEOUT', 10: 'CTRL_MODE', 13: 'hw_ver', 14: 'sw_ver', 20: 'Gr',
                  21: 'PMAX', 22: 'VMAX', 23: 'TMAX', 80: 'p_m', 81: 'xout'}
INTEGER_REGISTERS = {'MST_ID', 'ESC_ID', 'TIMEOUT', 'CTRL_MODE', 'hw_ver', 'sw_ver'}
ST_DISABLED, ST_ENABLED, ST_LOST_COMMS = 0x0, 0x1, 0xD
JOINT_INERTIA = (0.12, 0.12, 0.06, 0.012, 0.012, 0.004)
JAW_INERTIA = 0.0008
JAW_STROKE = 6.57
JAW_CLOSED = 3.0


def _to_uint(x, span, bits):
    x = max(-span, min(span, x))
    return int((x + span) * ((1 << bits) - 1) / (2 * span))


def _to_float(u, span, bits):
    return u * 2 * span / ((1 << bits) - 1) - span


def wrap_to_turn(position):
    return position - 2 * math.pi * round(position / (2 * math.pi))


class Motor:
    def __init__(self, motor_id, kind, direction, inertia, friction):
        self.id, self.kind, self.direction, self.inertia, self.friction = motor_id, kind, direction, inertia, friction
        self.reported_kind = kind
        self.p = self.v = self.tau = 0.0                # motor frame, continuous
        self.boot_offset = 0.0                          # what the encoder adds to `p` after this power-up
        self.state = ST_DISABLED
        self.cmd = (0.0, 0.0, 0.0, 0.0, 0.0)            # p, v, kp, kd, torque
        self.heard_at = None
        self.silent = False
        self.was_enabled = False                        # until then the arm rests on whatever it was left on
        self.temp = 35
        self.registers = {'MST_ID': motor_id + FEEDBACK_OFFSET, 'ESC_ID': motor_id, 'TIMEOUT': 400, 'CTRL_MODE': 1,
                          'hw_ver': 3, 'sw_ver': 5014}


class FakeYamChain:
    def __init__(self, name, q=(0.4, 1.3, 1.1, 0.2, -0.3, 0.2), jaw=0.0, arm='yam', gripper='linear_4310', gravity=True):
        self.name, self.address = name, f'virtual://{name}'
        arm_type, gripper_type = ArmType(arm), GripperType(gripper)
        config = _load_arm_config(arm_type)
        self.factors = np.asarray(config.gravity_comp_factor, dtype=float)
        self.kdl = MuJoCoKDL(combine_arm_and_gripper_xml(arm_type, gripper_type)) if gravity else None
        self.motors = {}
        for i, (motor_id, kind) in enumerate(config.motor_list):
            self.motors[int(motor_id)] = Motor(int(motor_id), kind, config.directions[i], JOINT_INERTIA[i],
                                               float(config.coulomb_friction[i]))
            self.motors[int(motor_id)].p = q[i] * config.directions[i]
        self.arm_motors = list(self.motors.values())
        self.has_jaw = gripper_type != GripperType.NO_GRIPPER and not gripper_type.is_teaching_handle
        # the teaching handle's passive encoder: report request 0x02 on 0x50e, report on 0x50f (!B h h B: device,
        # position and velocity in 1/4096 turn, the two buttons as bits). It has no motor and takes no other request here.
        self.handle = dict(counts=0, velocity=0, buttons=0, silent=False, requests=[]) if gripper_type.is_teaching_handle else None
        self.answers_status_request = True
        self.jaw = None
        if self.has_jaw:
            self.jaw = Motor(int(config.gripper_can_id), gripper_type.get_motor_type(arm_type),
                             gripper_type.get_motor_direction(arm_type), JAW_INERTIA, 0.05)
            self.jaw.p = jaw
            self.motors[self.jaw.id] = self.jaw
        for motor in self.motors.values():              # an encoder only knows its position within one turn at power-up
            motor.boot_offset = wrap_to_turn(motor.p) - motor.p
        self.frames = collections.deque(maxlen=200_000)  # (t, arbitration id, data bytes): everything heard on the wire
        self.mit = collections.deque(maxlen=200_000)     # (t, motor, p, v, kp, kd, torque)
        self.trace = collections.deque(maxlen=50_000)    # (t, [true joint positions..., jaw])
        self.lock = threading.Lock()
        self.bus = can.Bus(interface='virtual', channel=name, receive_own_messages=False)
        self._stop = threading.Event()
        self._at = time.monotonic()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f'fake-yam-{name}')
        self._thread.start()

    # -- test side ------------------------------------------------------------------------------------------------
    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self.bus.shutdown()

    def positions(self):
        with self.lock:
            return [m.p * m.direction for m in self.arm_motors]

    def jaw_position(self):
        with self.lock:
            return self.jaw.p

    def jaw_limits(self):
        return [JAW_CLOSED, JAW_CLOSED - JAW_STROKE]

    def drop(self, motor_id):
        self.motors[motor_id].silent = True

    def fault(self, motor_id, code=0xC):
        with self.lock:
            self.motors[motor_id].state = code

    def heard(self, since=0.0):
        return [f for f in list(self.frames) if f[0] >= since]

    def special_frames(self, last_byte, since=0.0):
        return [f for f in self.heard(since) if f[1] in self.motors and f[2] == bytes([0xFF] * 7 + [last_byte])]

    def register_commands(self, since=0.0):
        return [f[2][2] for f in self.heard(since) if f[1] == BROADCAST]

    # -- the wire -------------------------------------------------------------------------------------------------
    def _run(self):
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=0.0005)
            reply = None
            with self.lock:
                self._advance(time.monotonic())
                if msg is not None:
                    reply = self._handle(msg)
            if reply is not None:
                self.bus.send(reply)

    def _handle(self, msg):
        data = bytes(msg.data)
        self.frames.append((time.monotonic(), msg.arbitration_id, data))
        if msg.arbitration_id == BROADCAST:
            return self._register(data)
        if msg.arbitration_id == 0x50E and self.handle is not None:
            self.handle['requests'].append(data)
            if self.handle['silent'] or data[1:2] != b'\x02':
                return None
            report = struct.pack('!BhhB', 1, self.handle['counts'], self.handle['velocity'], self.handle['buttons'])
            return can.Message(arbitration_id=0x50F, data=report, is_extended_id=False)
        motor = self.motors.get(msg.arbitration_id)
        if motor is None or motor.silent:
            return None
        motor.heard_at = time.monotonic()
        if data[:7] == b'\xff' * 7:
            if data[7] == 0xFC:
                if motor.state in (ST_DISABLED, ST_ENABLED, ST_LOST_COMMS):
                    motor.state = ST_ENABLED
                    motor.was_enabled = True
                    motor.cmd = (0.0, 0.0, 0.0, 0.0, 0.0)
            elif data[7] == 0xFD:
                motor.state = ST_DISABLED
            elif data[7] == 0xFB:
                motor.state = ST_DISABLED
                return None
            return self._feedback(motor)
        return self._mit(motor, data)

    def _mit(self, motor, data):
        pmax, vmax, tmax, _ = SCALE[motor.kind]
        p = _to_float((data[0] << 8) | data[1], pmax, 16)
        v = _to_float((data[2] << 4) | (data[3] >> 4), vmax, 12)
        kp = ((data[3] & 0xF) << 8 | data[4]) * 500.0 / 4095
        kd = (data[5] << 4 | data[6] >> 4) * 5.0 / 4095
        torque = _to_float(((data[6] & 0xF) << 8) | data[7], tmax, 12)
        if motor.state == ST_ENABLED:
            motor.cmd = (p - motor.boot_offset, v, kp, kd, torque)
        self.mit.append((time.monotonic(), motor.id, p, v, kp, kd, torque))
        return self._feedback(motor)

    def _feedback(self, motor):
        pmax, vmax, tmax, _ = SCALE[motor.kind]
        p = _to_uint(motor.p + motor.boot_offset, pmax, 16)
        v = _to_uint(motor.v, vmax, 12)
        t = _to_uint(motor.tau, tmax, 12)
        data = bytes([(motor.state << 4) | (motor.id & 0xF), p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (t >> 8), t & 0xFF, 31, motor.temp])
        return can.Message(arbitration_id=motor.id + FEEDBACK_OFFSET, data=data, is_extended_id=False)

    def _register(self, data):
        motor = self.motors.get(data[0] | data[1] << 8)
        if motor is None or motor.silent:
            return None
        command, address = data[2], data[3]
        name = REGISTER_NAMES.get(address)
        head = bytes(data[:4])
        reply_id = motor.id + FEEDBACK_OFFSET
        if command == 0xCC:                             # status refresh: a feedback frame, in any motor state
            return self._feedback(motor) if self.answers_status_request else None
        if command == 0xAA:
            return can.Message(arbitration_id=reply_id, data=head, is_extended_id=False)
        if command == 0x55 and name:
            value = struct.unpack('<I' if name in INTEGER_REGISTERS else '<f', data[4:8])[0]
            motor.registers[name] = value
        value = self._read(motor, name)
        raw = struct.pack('<I', int(value)) if name in INTEGER_REGISTERS else struct.pack('<f', float(value))
        return can.Message(arbitration_id=reply_id, data=head + raw, is_extended_id=False)

    def _read(self, motor, name):
        pmax, vmax, tmax, gear = SCALE[motor.reported_kind]
        fixed = dict(Gr=gear, PMAX=pmax, VMAX=vmax, TMAX=tmax, p_m=(motor.p + motor.boot_offset) * gear,
                     xout=motor.p + motor.boot_offset)
        return fixed.get(name, motor.registers.get(name, 0))

    # -- the plant ------------------------------------------------------------------------------------------------
    def _advance(self, now):
        elapsed = min(now - self._at, 0.03)
        if elapsed < 0.0005:
            return
        self._at = now
        q = np.array([m.p * m.direction for m in self.arm_motors])
        gravity = np.zeros(len(q))
        if self.kdl is not None:
            gravity = self.kdl.compute_inverse_dynamics(q, np.zeros_like(q), np.zeros_like(q)) * self.factors
        steps = max(1, math.ceil(elapsed / 0.001))
        dt = elapsed / steps
        for i, motor in enumerate(self.arm_motors):
            load = -gravity[i] * motor.direction
            for _ in range(steps):
                self._step(motor, load, dt, now)
        if self.jaw is not None:
            for _ in range(steps):
                self._step(self.jaw, 0.0, dt, now)
            low, high = JAW_CLOSED - JAW_STROKE, JAW_CLOSED
            if self.jaw.p < low or self.jaw.p > high:
                self.jaw.p, self.jaw.v = max(low, min(high, self.jaw.p)), 0.0
        self.trace.append((now, [m.p * m.direction for m in self.arm_motors] + ([self.jaw.p] if self.jaw else [])))

    def _step(self, m, load, dt, now):
        if not m.was_enabled:
            return
        timeout = m.registers['TIMEOUT'] / 1000.0
        if m.state == ST_ENABLED and timeout and m.heard_at is not None and now - m.heard_at > timeout:
            m.state = ST_LOST_COMMS                     # the firmware's failsafe: damping only
        tmax = SCALE[m.kind][2]
        p_cmd, v_cmd, kp, kd, feedforward = m.cmd
        if m.state == ST_LOST_COMMS:
            p_cmd, v_cmd, kp, kd, feedforward = 0.0, 0.0, 0.0, 2.0, 0.0
        elif m.state != ST_ENABLED:
            kp, kd, feedforward = 0.0, 0.0, 0.0
        drive = kp * (p_cmd - m.p) + kd * (v_cmd - m.v) + feedforward
        clipped = max(-tmax, min(tmax, drive))
        m.tau = clipped
        net = clipped + load
        if abs(m.v) < 1e-3 and abs(net) <= m.friction:  # stiction holds
            m.v = 0.0
            return
        slip = math.copysign(m.friction, m.v if abs(m.v) >= 1e-3 else net)
        if clipped == drive:                            # implicit in the PD terms: stable at any stiffness
            rhs = m.v + dt / m.inertia * (kp * (p_cmd - m.p) + kd * v_cmd + feedforward + load - slip)
            m.v = rhs / (1 + dt / m.inertia * (kd + 0.05) + dt * dt / m.inertia * kp)
        else:
            m.v = (m.v + dt / m.inertia * (net - slip)) / (1 + dt / m.inertia * 0.05)
        m.p += m.v * dt


# -- USB adapters in front of the same wire ------------------------------------------------------------------------
class SimSlcanAdapter:
    """A CANable on a pseudo-terminal: the slcan text protocol on one side, the virtual CAN wire on the other."""
    serial = 'SIMSLCAN1'

    def __init__(self, wire):
        self.master, slave = os.openpty()
        tty.setraw(self.master)
        tty.setraw(slave)
        self.port = os.ttyname(slave)
        self._slave = slave
        self.bus = can.Bus(interface='virtual', channel=wire, receive_own_messages=False)
        self.open = False
        self.commands = []
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._from_host, daemon=True), threading.Thread(target=self._from_wire, daemon=True)]
        for t in self._threads:
            t.start()

    def port_row(self):
        return SimpleNamespace(vid=0x16d0, pid=0x117e, serial_number=self.serial, device=self.port, product='sim slcan',
                               location=getattr(self, 'location', None))

    def _from_host(self):
        buffer = b''
        while not self._stop.is_set():
            try:
                buffer += os.read(self.master, 256)
            except OSError:
                return
            while b'\r' in buffer:
                line, buffer = buffer.split(b'\r', 1)
                self._line(line.decode())

    def _line(self, line):
        if line[:1] == 't' and self.open:
            self.bus.send(can.Message(arbitration_id=int(line[1:4], 16), data=bytes.fromhex(line[5:5 + 2 * int(line[4])]),
                                      is_extended_id=False))
            return
        self.commands.append(line)
        if line == 'O':
            self.open = True
        elif line == 'C':
            self.open = False
        os.write(self.master, b'\r')

    def _from_wire(self):
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=0.01)
            if msg is not None and self.open:
                text = f't{msg.arbitration_id:03X}{len(msg.data)}{bytes(msg.data).hex().upper()}\r'
                os.write(self.master, text.encode())

    def close(self):
        self._stop.set()
        os.close(self._slave)
        os.close(self.master)
        self.bus.shutdown()


class SimGsUsbDevice:
    """A candleLight adapter as pyusb sees it: vendor control requests and two bulk endpoints."""
    idVendor, idProduct = 0x1d50, 0x606f
    product = 'sim candleLight'
    BT_CONST = (0, 48_000_000, 1, 16, 1, 8, 4, 1, 1024, 1)

    def __init__(self, wire, serial='SIMGSUSB1', bus=1, port_numbers=(2, 3)):
        self.serial_number = serial
        self.bus, self.port_numbers = bus, port_numbers
        self.wire = can.Bus(interface='virtual', channel=wire, receive_own_messages=False)
        self.inbox = queue.Queue()
        self.started = False
        self.host_format = self.bit_timing = None
        self.resets = 0
        self.claimed = False
        self._ctx = SimpleNamespace(managed_claim_interface=self._claim, managed_release_interface=self._release,
                                    dispose=lambda *_: None)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._from_wire, daemon=True)
        self._thread.start()

    def _claim(self, _dev, _interface):
        assert not self.claimed
        self.claimed = True

    def _release(self, _dev, _interface):
        self.claimed = False

    def is_kernel_driver_active(self, _interface):
        return False

    def get_active_configuration(self):
        return object()

    def reset(self):
        self.resets += 1

    def ctrl_transfer(self, request_type, request, value, index, payload, timeout=None):
        if request_type == 0xC1 and request == 4:
            return bytearray(struct.pack('<10I', *self.BT_CONST))
        assert request_type == 0x41
        if request == 0:
            self.host_format = struct.unpack('<I', bytes(payload))[0]
        elif request == 1:
            self.bit_timing = struct.unpack('<5I', bytes(payload))
        elif request == 2:
            mode, _flags = struct.unpack('<II', bytes(payload))
            self.started = mode == 1 and self.rate() == 1_000_000 and self.host_format == 0xBEEF if mode == 1 else False
        return len(payload)

    def rate(self):
        prop, seg1, seg2, _sjw, brp = self.bit_timing
        return self.BT_CONST[1] / (brp * (1 + prop + seg1 + seg2))

    def write(self, endpoint, data, timeout=None):
        assert endpoint == 0x02 and self.started
        _echo, can_id, dlc, _channel, _flags, _reserved, payload = struct.unpack('<IIBBBB8s', bytes(data))
        self.wire.send(can.Message(arbitration_id=can_id & 0x7FF, data=payload[:dlc], is_extended_id=False))
        self.inbox.put(bytes(data))                      # the adapter reports each sent frame back
        return len(data)

    def read(self, endpoint, size, timeout=None):
        assert endpoint == 0x81
        try:
            return bytearray(self.inbox.get(timeout=(timeout or 100) / 1000))
        except queue.Empty:
            raise usb.core.USBTimeoutError('timeout', 110, 110) from None

    def _from_wire(self):
        while not self._stop.is_set():
            msg = self.wire.recv(timeout=0.01)
            if msg is not None and self.started:
                self.inbox.put(struct.pack('<IIBBBB8s', 0xFFFFFFFF, msg.arbitration_id, len(msg.data), 0, 0, 0,
                                           bytes(msg.data).ljust(8, b'\0')))

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)
        self.wire.shutdown()
