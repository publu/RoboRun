"""A simulated Damiao CAN bus on python-can's virtual interface, speaking the motors' real frames.

Each motor: MIT frames on its id, enable/disable (FF.. FC/FD), the 0x7FF refresh and register-read service, state frames
on its master id. Load: a motor that is off, or on with neither stiffness nor damping, falls once it has been lifted; damping alone sinks slowly.
"""
import struct
import threading
import time

import can


def to_uint(x, lo, hi, bits): return round((max(lo, min(hi, x)) - lo) * ((1 << bits) - 1) / (hi - lo))
def to_float(u, lo, hi, bits): return u * (hi - lo) / ((1 << bits) - 1) + lo


class Motor:
    def __init__(self, row, pos):
        self.name, self.id, self.recv, self.pos = row['name'], row['id'], row['recv'], pos
        self.pmax, self.vmax, self.tmax = row['pmax'], row['vmax'], row['tmax']
        self.enabled, self.err, self.mode, self.mute = False, 0, 1, False
        self.target, self.kp, self.kd = pos, 0.0, 0.0
        self.vel, self.tau, self.armed = 0.0, 0.0, False
        self.temp, self.blocked = 31, False
        self.flash = dict(mode=1, id=self.id, recv=self.recv)       # what a power cycle comes back with
        self.zero_offset = 0.0
        self.refuse_writes = False                                  # a motor that answers a write but keeps its value


class FakeDamiaoBus:
    fall_speed, sink_speed, follow_speed = 2.0, 0.01, 4.0

    def __init__(self, channel, rows, start, load=True):
        self.motors = {r['id']: Motor(r, start[r['name']]) for r in rows}
        self.by_name = {m.name: m for m in self.motors.values()}
        self.load, self.frames, self.lock = load, [], threading.Lock()
        self.bus = can.Bus(interface='virtual', channel=channel, receive_own_messages=False)
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self):
        self._stop.set(); self.thread.join(2); self.bus.shutdown()

    # what a test looks at
    def pos(self, name): return self.by_name[name].pos
    def move_by_hand(self, name, pos): self.by_name[name].pos = pos
    def enabled(self): return [m.name for m in self.motors.values() if m.enabled]
    def enable_frames(self): return [f for f in self.frames if f[1] == b'\xff' * 7 + b'\xfc']
    def fault(self, name, code=0xa): m = self.by_name[name]; m.err, m.enabled = code, False
    def silence(self, name): self.by_name[name].mute = True
    def wrong_motor(self, name): self.by_name[name].tmax *= 2
    def remove(self, name): self.silence(name)
    def target(self, name): return self.by_name[name].target
    def set_temp(self, name, value): self.by_name[name].temp = int(value)
    def block(self, name): self.by_name[name].blocked = True
    def restore(self, name): self.by_name[name].mute = False
    def make_heavy(self): self.sink_speed = 0.3
    def set_load(self, value): self.load = value
    def power_on(self, name):                            # left holding by an earlier session
        m = self.by_name[name]; m.enabled = m.armed = True; m.target, m.kp, m.kd = m.pos, 50.0, 1.0

    def writes(self): return [f for f in self.frames if f[0] == 0x7ff and f[1][2] in (0x55, 0xaa)]
    def saves(self): return [f for f in self.frames if f[0] == 0x7ff and f[1][2] == 0xaa]
    def pos_force_frames(self): return [f for f in self.frames if f[0] & 0x700 == 0x300]
    def power_cycle(self):
        for m in list(self.motors.values()):
            m.mode, m.enabled = m.flash['mode'], False
            if m.id != m.flash['id'] or m.recv != m.flash['recv']:
                del self.motors[m.id]; m.id, m.recv = m.flash['id'], m.flash['recv']; self.motors[m.id] = m

    def _send(self, arbitration, data):
        self.bus.send(can.Message(arbitration_id=arbitration, data=bytes(data), is_extended_id=False))

    def _state(self, m):
        status = m.err or (1 if m.enabled else 0)
        q = to_uint(m.pos - m.zero_offset, -m.pmax, m.pmax, 16); v = to_uint(m.vel, -m.vmax, m.vmax, 12); t = to_uint(m.tau, -m.tmax, m.tmax, 12)
        self._send(m.recv, [status << 4 | m.id & 0xf, q >> 8, q & 0xff, v >> 4, (v & 0xf) << 4 | t >> 8, t & 0xff, m.temp, 33])

    def _register(self, m, rid, command=0x33):
        values = {21: struct.pack('<f', m.pmax), 22: struct.pack('<f', m.vmax), 23: struct.pack('<f', m.tmax),
                  10: struct.pack('<I', m.mode), 9: struct.pack('<I', 0), 7: struct.pack('<I', m.recv), 8: struct.pack('<I', m.id)}
        if rid in values:
            self._send(m.recv, bytes([m.id & 0xff, m.id >> 8, command, rid]) + values[rid])

    def _write(self, m, rid, value):
        if m.enabled or m.refuse_writes:                 # parameters only change on a disabled motor
            return self._register(m, rid, 0x55)
        number = struct.unpack('<I', value)[0]
        if rid == 10: m.mode = number
        elif rid == 7: m.recv = number
        elif rid == 8:
            del self.motors[m.id]; m.id = number; self.motors[m.id] = m
        self._register(m, rid, 0x55)

    def _handle(self, msg):
        d, arbitration = bytes(msg.data), msg.arbitration_id
        self.frames.append((arbitration, d))
        if arbitration == 0x7ff:
            m = self.motors.get(d[0] | d[1] << 8)
            if m is None or m.mute: return
            if d[2] == 0xcc: self._state(m)
            elif d[2] == 0x33: self._register(m, d[3])
            elif d[2] == 0x55: self._write(m, d[3], d[4:8])
            elif d[2] == 0xaa and not m.enabled: m.flash = dict(mode=m.mode, id=m.id, recv=m.recv)
            return
        if arbitration & 0x700 == 0x300:                 # position-force: position f32, speed x100, current x10000
            m = self.motors.get(arbitration & 0xff)
            if m is None or m.mute or m.mode != 4: return
            pos, speed, current = struct.unpack('<fHH', d)
            m.target, m.kp, m.kd = pos, (50.0 if current else 0.0), 0.0
            m.current = current / 10000
            if m.enabled: self._state(m)
            return
        m = self.motors.get(arbitration)
        if m is None or m.mute: return
        if d[:7] == b'\xff' * 7 and d[7] == 0xfe:
            if not m.enabled: m.zero_offset = m.pos
            self._state(m)
            return
        if d[:7] == b'\xff' * 7 and d[7] in (0xfc, 0xfd):
            if d[7] == 0xfc and m.err == 0: m.enabled = m.armed = True
            if d[7] == 0xfd: m.enabled = False
            self._state(m)
            return
        if m.mode != 1: return                           # an MIT frame is not answered in another mode
        q = d[0] << 8 | d[1]; k = (d[3] & 0xf) << 8 | d[4]; c = d[5] << 4 | d[6] >> 4
        m.target, m.kp, m.kd = to_float(q, -m.pmax, m.pmax, 16), to_float(k, 0, 500, 12), to_float(c, 0, 5, 12)
        if m.enabled: self._state(m)

    def _physics(self, dt):
        for m in self.motors.values():
            before = m.pos
            if m.blocked:
                m.tau = m.tmax if m.enabled and m.kp else 0.0
            elif m.enabled and m.kp > 0:
                step = self.follow_speed * dt
                m.tau = max(-m.tmax, min(m.tmax, m.kp * (m.target - m.pos)))
                m.pos += max(-step, min(step, m.target - m.pos))
            elif self.load and m.armed:                  # a resting arm stays put until it has once been lifted
                m.pos -= (self.sink_speed if m.enabled and m.kd > 0 else self.fall_speed) * dt
            m.vel = (m.pos - before) / dt if dt else 0.0

    def _run(self):
        last = time.monotonic()
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=.001)
            if msg is not None:
                with self.lock: self._handle(msg)
            now = time.monotonic()
            if now - last >= .002:
                self._physics(now - last); last = now
