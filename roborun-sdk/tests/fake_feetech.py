"""Simulated STS3215 servos on a pseudo-terminal, speaking Feetech's packet protocol and register map."""
import math

from fake_serial_bus import FakeSerialBus

COUNTS = 4096


def sm(value, bit): return abs(int(value)) | 1 << bit if value < 0 else int(value)
def unsm(raw, bit): return -(raw & ((1 << bit) - 1)) if raw >> bit & 1 else raw & ((1 << bit) - 1)


class Servo:
    def __init__(self, sid, counts, model=777):
        self.id, self.actual, self.mem, self.error, self.mute = sid, float(counts), bytearray(256), 0, False
        self.mem[3:5] = model.to_bytes(2, 'little'); self.mem[5] = sid
        self.mem[11:13] = (COUNTS - 1).to_bytes(2, 'little'); self.mem[48:50] = (1000).to_bytes(2, 'little'); self.mem[55] = 1
        self.speed, self.armed = 0.0, False; self.temp, self.blocked = 36, False; self.sync_goal()

    def u16(self, a): return int.from_bytes(self.mem[a:a + 2], 'little')
    def homing(self): return unsm(self.u16(31), 11)
    def present(self): return (self.actual - self.homing()) % COUNTS
    def sync_goal(self): self.mem[42:44] = int(self.present()).to_bytes(2, 'little')
    def refresh(self):
        self.mem[56:58] = (round(self.present()) % COUNTS).to_bytes(2, 'little')
        self.mem[58:60] = sm(round(self.speed), 15).to_bytes(2, 'little')
        self.mem[60:62] = sm(120, 10).to_bytes(2, 'little'); self.mem[62] = 74; self.mem[63] = self.temp


class FakeFeetechBus(FakeSerialBus):
    fall, sag, follow = 1200.0, 8.0, 2600.0            # counts per second
    hold_needs = 100                                   # torque limit a loaded joint needs

    def __init__(self, ids_counts, load=True, loaded=()):
        super().__init__()
        self.servos = {i: Servo(i, c) for i, c in ids_counts.items()}
        self.load, self.loaded = load, set(loaded) or set(ids_counts)

    def checksum(self, body): return ~sum(body) & 0xff
    def status(self, s, data=b''):
        body = bytes([s.id, len(data) + 2, s.error]) + bytes(data)
        self.reply(b'\xff\xff' + body + bytes([self.checksum(body)]))

    def consume(self):
        buf = self.rx
        while True:
            at = buf.find(b'\xff\xff')
            if at < 0: buf.clear(); return
            del buf[:at]
            if len(buf) < 4: return
            total = buf[3] + 4
            if len(buf) < total: return
            p = bytes(buf[:total]); del buf[:total]
            if self.checksum(p[2:-1]) == p[-1]:
                self.packets.append(p); self.handle(p[2], p[4], p[5:-1])

    def write(self, s, address, data):
        s.mem[address:address + len(data)] = data
        if address == 5 and data[0] != s.id:
            del self.servos[s.id]; s.id = data[0]; self.servos[s.id] = s
        if address == 40 and data[0]: s.armed = True

    def handle(self, sid, inst, params):
        if inst == 0x83:
            address, length = params[0], params[1]
            for i in range(2, len(params), length + 1):
                s = self.servos.get(params[i])
                if s: self.write(s, address, params[i + 1:i + 1 + length])
            return
        if inst == 0x82:
            address, length = params[0], params[1]
            for i in params[2:]:
                s = self.servos.get(i)
                if s and not s.mute:
                    s.refresh(); self.status(s, s.mem[address:address + length])
            return
        s = self.servos.get(sid)
        if s is None or s.mute: return
        if inst == 2:
            s.refresh(); self.status(s, s.mem[params[0]:params[0] + params[1]])
        elif inst == 3:
            self.write(s, params[0], params[1:]); self.status(s)

    def physics(self, dt):
        for s in self.servos.values():
            before = s.actual
            on = s.mem[40] == 1
            if s.blocked:
                pass
            elif on:
                goal = unsm(s.u16(42), 15) + s.homing()
                strong = s.u16(48) >= self.hold_needs
                if strong:
                    s.actual += max(-self.follow * dt, min(self.follow * dt, goal - s.actual))
                elif self.load and s.id in self.loaded:
                    s.actual -= self.sag * dt
            elif self.load and s.armed and s.id in self.loaded:
                s.actual -= self.fall * dt
            s.speed = (s.actual - before) / dt if dt else 0.0

    # what a test looks at
    def counts(self, sid): return self.servos[sid].present()
    def torque_writes_on(self):
        return [p for p in self.packets if p[4] in (3, 0x83) and p[5] == 40 and (p[6] == 1 or (p[4] == 0x83 and p[8] == 1))]
    def rad(self, sid): return self.counts(sid) * 2 * math.pi / COUNTS
