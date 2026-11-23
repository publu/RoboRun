"""The simulated robot hardware, in its own process. Arms and adapters from a JSON spec; commands one per line on stdin.

    {"arms": [{"name": "A", "arm": "so101", "via": "serial"}, {"name": "B", "arm": "yam", "via": "vcan"|"slcan"}], "cameras": {"camA": 120}}

Replies (JSON, one per line): state / modes / unplug NAME / replug NAME / sweep NAME / wiggle NAME CAM SECONDS / still NAME / frames /
cover CAM / uncover CAM / quit
"""
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import can

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent)]
_real_bus = can.Bus


def _bus(*args, **kw):
    channel = kw.get('channel', '')
    if str(channel).startswith('vcan'):
        return _real_bus(interface='socketcan', channel=channel, receive_own_messages=False)
    return _real_bus(*args, **kw)


can.Bus = _bus

from roborun_sdk.arms import catalog, identify  # noqa: E402

TO_COUNTS = 4096 / (2 * 3.141592653589793)


class Arm:
    def __init__(self, spec, index):
        self.spec, self.name, self.arm_id, self.via = spec, spec['name'], spec['arm'], spec['via']
        self.serial = spec.get('serial') or f'SIM{self.name}'
        self.row = catalog()[self.arm_id]
        self.adapter, self.vcan, self.kind, self.plugged = None, None, identify.family_of(self.row), spec.get('plugged', True)
        wire = f'vcan{index}' if self.via == 'vcan' else f'wire{index}'
        self.wire = wire
        if self.via == 'vcan':
            subprocess.run(['ip', 'link', 'del', wire], capture_output=True)
            subprocess.run(['ip', 'link', 'add', wire, 'type', 'vcan'], check=True)
            subprocess.run(['ip', 'link', 'set', wire, 'up'], check=True)
            self.vcan = wire
            if not self.plugged:
                subprocess.run(['ip', 'link', 'del', wire], check=True)
        options = self.row.get('options') or {}
        if self.arm_id.startswith('yam') or self.arm_id == 'big_yam':
            from fake_yam_chain import FakeYamChain
            self.fake = FakeYamChain(wire, arm=options.get('arm_type', 'yam'), gripper=options.get('gripper_type', 'linear_4310'))
            threading.Thread(target=self._rests_when_off, daemon=True).start()
        elif self.kind == 'damiao':
            from fake_damiao import FakeDamiaoBus
            rows = [r for r in options['joints'] if 'fixed' not in r] + [options['gripper']] + list(options.get('held', ()))
            start = {r['name']: (options['gripper'].get('closed', 0.5) + 0.2 if r is options['gripper'] else 0.0) for r in rows}
            for r, q, (lo, hi) in zip(options['joints'], self.row['home'], self.row['limits']):
                if 'fixed' not in r:
                    start[r['name']] = float(r.get('sign', 1)) * max(lo + .1, min(hi - .1, q)) + r.get('zero', 0.0)
            self.fake = FakeDamiaoBus(wire, rows, start, load=False)
            for motor, mode in (spec.get('modes') or {}).items():          # a motor left in another control mode
                self.fake.by_name[motor].mode = self.fake.by_name[motor].flash['mode'] = mode
        elif self.kind == 'robstride':
            from fake_robstride import FakeRobStrideBus
            rows = [r for r in options['joints'] if 'fixed' not in r] + [options['gripper']] + list(options.get('held', ()))
            start = {r['name']: (options['gripper'].get('closed', 0.0) + 0.1 if r is options['gripper'] else 0.0) for r in rows}
            for r, q, (lo, hi) in zip(options['joints'], self.row['home'], self.row['limits']):
                if 'fixed' not in r:
                    start[r['name']] = float(r.get('sign', 1)) * max(lo + .1, min(hi - .1, q)) + r.get('zero', 0.0)
            self.fake = FakeRobStrideBus(wire, rows, start, load=False)
        elif self.kind == 'feetech':
            from fake_feetech import FakeFeetechBus
            self.fake = FakeFeetechBus({i: 2048 for i in identify.expected_motors(self.row)}, load=False).start()
        else:
            from fake_dynamixel import FakeDynamixelBus
            masters = {int(s['id']): int(s['master']) for s in options.get('shadows', ())}
            rows = {i: (2048, w['model_number'], masters.get(i)) for i, w in identify.expected_motors(self.row).items()}
            self.fake = FakeDynamixelBus(rows, load=False).start()
        if self.via == 'slcan':
            from fake_yam_chain import SimSlcanAdapter
            SimSlcanAdapter.serial = self.serial
            self.adapter = SimSlcanAdapter(wire)

    def _rests_when_off(self):
        """Someone holds the arm, as the setup asks: with its motors off it stays where it is."""
        while True:
            for m in self.fake.motors.values():
                if m.state == 0:
                    m.was_enabled, m.v = False, 0.0
            time.sleep(.002)

    def port_row(self):
        if not self.plugged:
            return None
        if self.via == 'slcan':
            return dict(vid=0x16d0, pid=0x117e, serial_number=self.serial, device=self.adapter.port, product='sim slcan', location=None)
        if self.via == 'serial':
            return dict(vid=0x1a86, pid=0x7523, serial_number=self.serial, device=self.fake.device, product='sim serial', location=None)
        return None

    def enabled(self):
        if self.kind in ('feetech', 'dynamixel'):
            reg = 40 if self.kind == 'feetech' else 64
            return [i for i, s in self.fake.servos.items() if s.mem[reg]]
        if hasattr(self.fake, 'arm_motors'):                                   # the YAM chain
            return [m.id for m in self.fake.motors.values() if m.state == 1]
        return self.fake.enabled()

    def enable_commands(self):
        if self.kind == 'feetech':
            return len(self.fake.torque_writes_on())
        if self.kind == 'dynamixel':
            return sum(1 for p in self.fake.packets if p[7] in (3, 0x83) and int.from_bytes(p[8:10], 'little') == 64)
        if hasattr(self.fake, 'arm_motors'):
            return len(self.fake.special_frames(0xFC))
        return len(self.fake.enable_frames())

    def direct_frames(self):
        """CAN frames addressed to a motor itself: anything but the 0x7FF register service."""
        frames = getattr(self.fake, 'frames', None)
        if frames is None or self.kind not in ('damiao', 'robstride'):
            return 0
        rows = [f[1:] if len(f) == 3 else f for f in list(frames)]
        if self.kind == 'damiao':
            return sum(1 for arbitration, _ in rows if arbitration != 0x7ff)
        return sum(1 for arbitration, _ in rows if arbitration != 0x7ff and arbitration & 0x700 != 0x300)

    def silence(self):
        motors = self.fake.servos if self.kind in ('feetech', 'dynamixel') else self.fake.motors
        for m in motors.values():
            if hasattr(m, 'silent'):
                m.silent = True
            else:
                m.mute = True

    def wiggle(self, seconds, publish_level):
        """A person moving this arm by hand; its wrist camera sees the room change. Runs `seconds`, or until `still NAME`
        when seconds is 0. The joints sweep back and forth in legs of ten steps, so any two reads a third of a second apart
        see the arm in a different place, and the picture takes a different brightness on every step."""
        ids = sorted(identify.expected_motors(self.row))[:2]
        self.moving = stop = threading.Event()
        end, step = time.monotonic() + seconds if seconds else float('inf'), 0
        while time.monotonic() < end and stop.is_set() is False:
            sign = 1 if step // 10 % 2 == 0 else -1
            for i in ids:
                self.nudge(i, 20 * sign)
            step += 1
            publish_level(40 + step * 37 % 160)
            time.sleep(.12)

    def still(self):
        if getattr(self, 'moving', None) is not None:
            self.moving.set()

    def nudge(self, motor_id, counts):
        if self.kind in ('feetech', 'dynamixel'):
            self.fake.servos[motor_id].actual += counts
        else:
            m = self.fake.motors[motor_id]
            if hasattr(m, 'p'):
                m.p += counts / TO_COUNTS
            else:
                m.pos += counts / TO_COUNTS

    def sweep(self):
        """A person moving every joint through its range and opening the jaw."""
        ids = sorted(identify.expected_motors(self.row))
        options = self.row.get('options') or {}
        jaw = int(options['gripper']['id']) if options.get('gripper') else ids[-1]
        for sign in (1, -2, 1):
            for i in ids:
                if i != jaw:
                    self.nudge(i, 800 * sign)
            time.sleep(.25)
        self.nudge(jaw, 500)
        time.sleep(.25)


def main():
    spec = json.load(open(sys.argv[1]))
    root = Path(sys.argv[2])
    arms = {row['name']: Arm(row, i) for i, row in enumerate(spec.get('arms', ()))}
    cameras = dict(spec.get('cameras') or {})
    saved = dict(cameras)

    published = threading.Lock()                     # a wiggle thread publishes too

    def publish():
        with published:
            _publish()

    def _publish():
        ports = [a.port_row() for a in arms.values()]
        (root / 'ports.tmp').write_text(json.dumps([p for p in ports if p]))
        (root / 'ports.tmp').replace(root / 'ports.json')
        (root / 'cams.tmp').write_text(json.dumps(cameras))
        (root / 'cams.tmp').replace(root / 'cameras.json')

    publish()
    print(json.dumps(dict(ready=True, addresses={n: address(a) for n, a in arms.items()})), flush=True)
    for raw in sys.stdin:
        words = raw.split()
        if not words:
            continue
        cmd, arg = words[0], words[1] if len(words) > 1 else None
        out = dict(ok=True)
        try:
            if cmd == 'quit':
                break
            if cmd == 'state':
                out = dict(enabled={n: a.enabled() for n, a in arms.items()}, enables={n: a.enable_commands() for n, a in arms.items()},
                           direct={n: a.direct_frames() for n, a in arms.items()})
            elif cmd == 'modes':                                               # Damiao control modes: now, and saved
                out = {n: {m.name: [m.mode, m.flash['mode']] for m in a.fake.motors.values()}
                       for n, a in arms.items() if a.kind == 'damiao'}
            elif cmd == 'silence':
                arms[arg].silence()
            elif cmd == 'still':                                               # still ARM: stop a wiggle
                arms[arg].still()
            elif cmd == 'frames':                                              # frames read so far, per camera
                out = dict(frames=json.loads((root / 'frames.json').read_text()) if (root / 'frames.json').exists() else {})
            elif cmd == 'wiggle':                                              # wiggle ARM CAMERA SECONDS (0: until `still ARM`)
                camera, seconds = words[2], float(words[3])
                def level(value):
                    cameras[camera] = value
                    publish()
                threading.Thread(target=arms[arg].wiggle, args=(seconds, level), daemon=True).start()
            elif cmd == 'unplug':
                a = arms[arg]; a.plugged = False
                if a.vcan:
                    subprocess.run(['ip', 'link', 'del', a.vcan], check=True)
            elif cmd == 'replug':
                a = arms[arg]; a.plugged = True
                if a.vcan:
                    subprocess.run(['ip', 'link', 'add', a.vcan, 'type', 'vcan'], check=True)
                    subprocess.run(['ip', 'link', 'set', a.vcan, 'up'], check=True)
            elif cmd == 'sweep':
                arms[arg].sweep()
            elif cmd == 'cover':
                cameras[arg] = 5
            elif cmd == 'uncover':
                cameras[arg] = saved[arg]
            publish()
        except Exception as exc:
            out = dict(ok=False, error=repr(exc))
        print(json.dumps(out), flush=True)


def address(a):
    if a.via == 'vcan':
        return a.vcan
    return f"{'slcan' if a.via == 'slcan' else 'serial'}://{a.serial}"


if __name__ == '__main__':
    main()
