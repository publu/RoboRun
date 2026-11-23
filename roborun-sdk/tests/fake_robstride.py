"""A simulated RobStride bus (MIT protocol, standard ids) on python-can's virtual interface."""
import struct
import threading
import time

import can

from fake_damiao import to_float, to_uint

HOST = 0xfd


class Motor:
    def __init__(self, row, pos):
        self.name, self.id, self.pos, self.pmax, self.vmax, self.tmax = row['name'], row['id'], pos, row['pmax'], row['vmax'], row['tmax']
        self.enabled, self.fault_word, self.run_mode, self.mute, self.answer_as = False, 0, 0, False, row['id']
        self.target, self.kp, self.kd, self.vel, self.tau, self.armed = pos, 0.0, 0.0, 0.0, 0.0, False
        self.temp, self.blocked = 34.5, False


class FakeRobStrideBus:
    fall_speed, sink_speed, follow_speed = 2.0, 0.01, 4.0

    def __init__(self, channel, rows, start, load=True, parameters=True):
        self.motors = {r['id']: Motor(r, start[r['name']]) for r in rows}
        self.by_name = {m.name: m for m in self.motors.values()}
        self.load, self.parameters, self.frames = load, parameters, []
        self.bus = can.Bus(interface='virtual', channel=channel, receive_own_messages=False)
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self):
        self._stop.set(); self.thread.join(2); self.bus.shutdown()

    def pos(self, name): return self.by_name[name].pos
    def move_by_hand(self, name, pos): self.by_name[name].pos = pos
    def enabled(self): return [m.name for m in self.motors.values() if m.enabled]
    def enable_frames(self): return [f for f in self.frames if f[1][:6] == b'\xff' * 6 and f[1][7] == 0xfc]
    def fault(self, name, word=1 << 14): self.by_name[name].fault_word = word
    def silence(self, name): self.by_name[name].mute = True
    def wrong_motor(self, name): self.by_name[name].answer_as = 99
    def remove(self, name): self.silence(name)
    def target(self, name): return self.by_name[name].target
    def set_temp(self, name, value): self.by_name[name].temp = float(value)
    def block(self, name): self.by_name[name].blocked = True
    def restore(self, name): self.by_name[name].mute = False
    def make_heavy(self): self.sink_speed = 0.3
    def set_load(self, value): self.load = value
    def power_on(self, name):
        m = self.by_name[name]; m.enabled = m.armed = True; m.target, m.kp, m.kd = m.pos, 50.0, 1.0

    def _send(self, arbitration, data):
        self.bus.send(can.Message(arbitration_id=arbitration, data=bytes(data), is_extended_id=False))

    def _state(self, m):
        q = to_uint(m.pos, -m.pmax, m.pmax, 16); v = to_uint(m.vel, -m.vmax, m.vmax, 12); t = to_uint(m.tau, -m.tmax, m.tmax, 12)
        temp = round(m.temp * 10)
        flags = (2 if m.enabled else 0) << 6 | (0x20 if m.fault_word else 0) | temp >> 8
        self._send(HOST, [m.answer_as, q >> 8, q & 0xff, v >> 4, (v & 0xf) << 4 | t >> 8, t & 0xff, flags, temp & 0xff])

    def _handle(self, msg):
        d, arbitration = bytes(msg.data), msg.arbitration_id
        self.frames.append((arbitration, d))
        if arbitration & 0x700 == 0x300:
            m = self.motors.get(arbitration & 0xff)
            if m and not m.mute and self.parameters and struct.unpack('<H', d[:2])[0] == 0x7005:
                self._send(arbitration, d[:2] + bytes(2) + struct.pack('<B3x', m.run_mode))
            return
        m = self.motors.get(arbitration)
        if m is None or m.mute: return
        if d[:6] == b'\xff' * 6 and d[7] == 0xfe:
            if not m.enabled: m.pos = 0.0                # the present position becomes zero
            self._state(m); return
        if d[:6] == b'\xff' * 6 and d[7] in (0xfb, 0xfc, 0xfd):
            if d[7] == 0xfb and d[6] == 0:
                self._send(HOST, bytes([m.answer_as]) + struct.pack('<I', m.fault_word) + bytes(3)); return
            if d[7] == 0xfb: m.fault_word = 0
            if d[7] == 0xfc and not m.fault_word: m.enabled = m.armed = True
            if d[7] == 0xfd: m.enabled = False
            self._state(m); return
        q = d[0] << 8 | d[1]; k = (d[3] & 0xf) << 8 | d[4]; c = d[5] << 4 | d[6] >> 4
        m.target, m.kp, m.kd = to_float(q, -m.pmax, m.pmax, 16), to_float(k, 0, 500, 12), to_float(c, 0, 5, 12)
        self._state(m)

    def _physics(self, dt):
        for m in self.motors.values():
            before = m.pos
            if m.blocked:
                pass
            elif m.enabled and m.kp > 0:
                step = self.follow_speed * dt
                m.tau = max(-m.tmax, min(m.tmax, m.kp * (m.target - m.pos)))
                m.pos += max(-step, min(step, m.target - m.pos))
            elif self.load and m.armed:
                m.pos -= (self.sink_speed if m.enabled and m.kd > 0 else self.fall_speed) * dt
            m.vel = (m.pos - before) / dt if dt else 0.0

    def _run(self):
        last = time.monotonic()
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=.001)
            if msg is not None: self._handle(msg)
            now = time.monotonic()
            if now - last >= .002:
                self._physics(now - last); last = now
