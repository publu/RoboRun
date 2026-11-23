"""Simulated X-series servos on a pseudo-terminal, speaking Dynamixel protocol 2.0, with shadow motors."""
import math

from fake_serial_bus import FakeSerialBus
from roborun_sdk.arms.dynamixel import crc16, stuff, unstuff

COUNTS = 4096


class Servo:
    def __init__(self, sid, counts, model):
        self.id, self.actual, self.mem, self.alert, self.mute = sid, float(counts), bytearray(256), 0, False
        self.mem[0:2] = model.to_bytes(2, 'little'); self.mem[7] = sid; self.mem[8] = 3; self.mem[9] = 250
        self.mem[11] = 3; self.mem[12] = 255; self.mem[100:102] = (885).to_bytes(2, 'little')
        self.velocity, self.armed = 0.0, False
        self.temp, self.blocked = 37, False
        self.mem[116:120] = int(counts).to_bytes(4, 'little', signed=True)

    def goal(self): return int.from_bytes(self.mem[116:120], 'little', signed=True)
    def pwm(self): return int.from_bytes(self.mem[100:102], 'little')
    def refresh(self):
        self.mem[126:128] = (40).to_bytes(2, 'little', signed=True)
        self.mem[128:132] = round(self.velocity / (0.229 * COUNTS / 60)).to_bytes(4, 'little', signed=True)
        homing = int.from_bytes(self.mem[20:24], 'little', signed=True)          # present = actual + homing offset
        self.mem[132:136] = round(self.actual + homing).to_bytes(4, 'little', signed=True)
        self.mem[144:146] = (120).to_bytes(2, 'little'); self.mem[146] = self.temp


class FakeDynamixelBus(FakeSerialBus):
    fall, sag, follow = 1200.0, 8.0, 2600.0
    hold_needs = 100

    def __init__(self, rows, load=True):
        """rows: {id: (counts, model number, master id or None)}"""
        super().__init__()
        self.servos = {i: Servo(i, c, m) for i, (c, m, _) in rows.items()}
        for i, (_, _, master) in rows.items():
            if master is not None:
                self.servos[i].mem[12] = master; self.servos[i].mem[10] = 1
        self.load = load

    def status(self, s, data=b''):
        error = 0x80 if s.alert else 0
        body = bytes([error]) + stuff(data)
        head = b'\xff\xff\xfd\x00' + bytes([s.id]) + (len(body) + 3).to_bytes(2, 'little') + b'\x55' + body
        self.reply(head + crc16(head).to_bytes(2, 'little'))

    def consume(self):
        buf = self.rx
        while True:
            at = buf.find(b'\xff\xff\xfd\x00')
            if at < 0: del buf[:max(0, len(buf) - 3)]; return
            del buf[:at]
            if len(buf) < 7: return
            total = 7 + int.from_bytes(buf[5:7], 'little')
            if len(buf) < total: return
            p = bytes(buf[:total]); del buf[:total]
            if crc16(p[:-2]) == int.from_bytes(p[-2:], 'little'):
                self.packets.append(p); self.handle(p[4], p[7], unstuff(p[8:-2]))

    def targets(self, sid):
        """The servo itself, and any shadow following it through Secondary ID."""
        return [s for s in self.servos.values() if s.id == sid or s.mem[12] == sid]

    def store(self, s, address, data):
        if address < 64 and s.mem[64]: return 7          # EEPROM is locked while torque is on (access error)
        s.mem[address:address + len(data)] = data
        if address == 7 and data[0] != s.id:
            del self.servos[s.id]; s.id = data[0]; self.servos[s.id] = s
        if address == 64 and data[0]: s.armed = True
        return 0

    def handle(self, sid, inst, p):
        u16 = lambda a: int.from_bytes(p[a:a + 2], 'little')
        if inst == 0x83:
            address, length = u16(0), u16(2)
            for i in range(4, len(p), length + 1):
                for s in self.targets(p[i]): self.store(s, address, p[i + 1:i + 1 + length])
            return
        if inst == 0x82:
            address, length = u16(0), u16(2)
            for i in p[4:]:
                s = self.servos.get(i)
                if s and not s.mute:
                    s.refresh(); self.status(s, s.mem[address:address + length])
            return
        s = self.servos.get(sid)
        if s is None or s.mute: return
        if inst == 2:
            s.refresh(); self.status(s, s.mem[u16(0):u16(0) + u16(2)])
        elif inst == 3:
            results = [self.store(t, u16(0), p[2:]) for t in self.targets(sid)]
            self.status(s) if not results[0] else self.reply_error(s, results[0])

    def reply_error(self, s, code):
        body = bytes([code]); head = b'\xff\xff\xfd\x00' + bytes([s.id]) + (4).to_bytes(2, 'little') + b'\x55' + body
        self.reply(head + crc16(head).to_bytes(2, 'little'))

    def physics(self, dt):
        for s in self.servos.values():
            before = s.actual
            if s.blocked:
                pass
            elif s.mem[64]:
                if s.pwm() >= self.hold_needs:
                    s.actual += max(-self.follow * dt, min(self.follow * dt, s.goal() - s.actual))
                elif self.load:
                    s.actual -= self.sag * dt
            elif self.load and s.armed:
                s.actual -= self.fall * dt
            s.velocity = (s.actual - before) / dt if dt else 0.0

    def rad(self, sid): return self.servos[sid].actual * 2 * math.pi / COUNTS
    def torque_writes_on(self):
        out = []
        for p in self.packets:
            params = unstuff(p[8:-2])
            if p[7] == 3 and int.from_bytes(params[:2], 'little') == 64 and params[2]: out.append(p)
            if p[7] == 0x83 and int.from_bytes(params[:2], 'little') == 64 and any(params[5::2]): out.append(p)
        return out
