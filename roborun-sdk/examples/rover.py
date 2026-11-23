"""Your own robot body: a diff-drive rover with one fixed camera, from your own files (my_rover.yaml, my_rover.xml, this file).

    python rover.py                  offline: drives the rover through Roborun's player (50 Hz, the profile's limits, Stop),
                                     then prints the request a move would send to the motion service (nothing is sent)
    python rover.py --mujoco         the same in Roborun's MuJoCo simulation of my_rover.xml, in the room scene
    python rover.py install          copies my_rover.yaml and my_rover.xml to ~/.config/roborun/bodies, then:

    roborun up --profile my_rover --sim room                       the rover simulated, its local API on 127.0.0.1:8120
    roborun up --profile my_rover --body /path/to/rover.py:Rover   this class, on your robot

`Rover` below is simulated (it integrates its own odometry and draws a picture): replace the marked lines with your motor
controller and camera. Run it with Roborun's Python: roborun python rover.py
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import threading
import time
from pathlib import Path

from roborun_sdk.body import BaseAdapter, HeadAdapter

HERE = Path(__file__).resolve().parent
PROFILE = HERE / 'my_rover.yaml'


class RoverBase(BaseAdapter):
    """A diff-drive base: forward speed and turning rate (no sideways), stopped when commands stop coming (the deadman)."""
    kind = 'wheeled'

    def __init__(self, limits, deadman_s=0.3):
        self.limits, self.deadman_s = limits, deadman_s
        self.lock = threading.Lock()
        self.x = self.y = self.yaw = 0.
        self.v = self.w = 0.
        self.commanded = dict(vx=0., vy=0., vyaw=0.)
        self.at = self.sent = time.monotonic()

    def _advance(self):
        now = time.monotonic()
        if now - self.sent > self.deadman_s:            # no fresh command: the base stops by itself
            self.v = self.w = 0.
        dt, self.at = now - self.at, now
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

    def command(self, vx, vy, vyaw):
        clamp = lambda v, lo_hi: min(lo_hi[1], max(lo_hi[0], float(v)))
        with self.lock:
            self._advance()
            self.v, self.w = clamp(vx, self.limits['vx_m_s']), clamp(vyaw, self.limits['vyaw_rad_s'])
            self.commanded = dict(vx=self.v, vy=0., vyaw=self.w)
            self.sent = time.monotonic()
            # your robot: send (self.v, self.w) to the motor controller here

    def stop(self):
        with self.lock:
            self._advance()
            self.v = self.w = 0.
            self.commanded = dict(vx=0., vy=0., vyaw=0.)
            # your robot: brake (zero velocity; never a power-off)

    def state(self):
        with self.lock:
            self._advance()
            # your robot: odometry from the wheel encoders
            return dict(x=self.x, y=self.y, yaw=self.yaw, vx=self.v, vy=0., vyaw=self.w, commanded=dict(self.commanded),
                        t=time.time())


class FixedCamera(HeadAdapter):
    """The one camera, fixed on a mast (dof 0). frame() needs only the JPEG: its size, intrinsics, height and tilt come from
    the profile's camera section."""
    dof = 0

    def __init__(self, base, camera):
        self.base, self.camera = base, camera

    def command(self, pitch, yaw):
        pass                                             # a fixed camera does not move

    def state(self):
        return dict(pitch=float(self.camera.get('pitch_rad', 0.)), yaw=0.)

    def frame(self):
        # your robot: grab a frame from the camera and return its JPEG bytes
        import numpy as np
        from PIL import Image
        import io
        w, h = self.camera['size_px']
        yaw = self.base.state()['yaw']
        img = np.zeros((h, w, 3), np.uint8)
        horizon = int(h * 0.42)
        img[:horizon] = (205, 215, 225)                  # wall
        img[horizon:] = (150, 140, 125)                  # floor
        door = int((0.5 - yaw / math.radians(90)) * w)   # a door straight ahead at the start, sliding across as the rover turns
        if -60 < door < w + 60:
            img[int(h * 0.15):horizon, max(0, door - 60):max(0, min(w, door + 60))] = (120, 80, 50)
        out = io.BytesIO()
        Image.fromarray(img).save(out, 'JPEG', quality=85)
        return dict(jpeg=out.getvalue())


class Rover:
    """What `roborun up --body rover.py:Rover` makes: an object with .base and .head. `profile` is the body's profile (a dict),
    given because the constructor names it; any other key of robot.yaml's `body:` section would be a keyword argument."""
    profile = 'my_rover'                                 # the profile it runs when roborun up names none

    def __init__(self, profile):
        self.base = RoverBase(profile['robot']['limits'], profile['motion']['deadman_s'])
        self.head = FixedCamera(self.base, profile['camera'])

    def close(self):
        self.base.stop()


# ---------------------------------------------------------------------------------------------------- offline demo
DRIVE = ([dict(t=i / 10, vx=0.4, vy=0., vyaw=0.) for i in range(15)]           # forward 1.5 s
         + [dict(t=1.5 + i / 10, vx=0.2, vy=0., vyaw=0.8) for i in range(10)])  # then a left arc


def demo(use_mujoco=False):
    from roborun_sdk.body import profiles
    from roborun_sdk.body.jobs import build_request
    from roborun_sdk.body.player import Watch, play
    pid = profiles.add_file(PROFILE)                     # roborun up reads ~/.config/roborun/bodies; this demo, the file here
    profile = profiles.load(pid)
    if use_mujoco:
        from roborun_sdk.body.sim import BodySim
        sim = BodySim(pid, scene='room')
        base, head, close = sim.base, sim.head, sim.close
        print(f'{profile["label"]}: MuJoCo simulation of {profile["sim"]["model"]}, room scene')
    else:
        rover = Rover(profile)
        base, head, close = rover.base, rover.head, rover.close
        print(f'{profile["label"]}: this file\'s simulated rover')
    try:
        frame = head.frame()
        request = build_request(pid, frame, 'drive to the door and stop in front of it', duration_s=3)
        print('a move would send the motion service (not sent: this demo is offline):')
        for key, value in request.items():
            print(f'  {key}: {json.dumps(value)}')
        before = base.state()
        watch = Watch(pid, base, head).start()           # the obstacle guard (without depth: slow, and at most 0.4 m per move)
        try:
            stop = threading.Event()
            played = play(DRIVE, base, head, pid, stop=stop, watch=watch)
        finally:
            watch.close()
        for _ in range(40):                              # a braking base comes to rest within its stop deceleration
            after = base.state()
            if abs(after['vx']) < 0.01 and abs(after['vyaw']) < 0.02:
                break
            time.sleep(0.05)
        moved = math.hypot(after['x'] - before['x'], after['y'] - before['y'])
        print(f"played {played['played_s']:.2f} s{' (stopped: ' + played['guard_stop']['event'] + ')' if played.get('guard_stop') else ''}: "
              f"moved {moved:.2f} m, turned {math.degrees(after['yaw'] - before['yaw']):.0f} deg, "
              f"now at rest: {abs(after['vx']) < 0.01 and abs(after['vyaw']) < 0.02}")
    finally:
        close()
    return 0


def install():
    home = Path(os.environ.get('ROBORUN_HOME') or Path.home() / '.config' / 'roborun') / 'bodies'
    home.mkdir(parents=True, exist_ok=True)
    for f in (PROFILE, HERE / 'my_rover.xml'):
        shutil.copy2(f, home / f.name)
    from roborun_sdk.body import profiles
    profiles.load('my_rover')                            # checked, as roborun up will
    print(f'installed my_rover in {home}')
    print('simulate it:      roborun up --profile my_rover --sim room')
    print(f'your own robot:   roborun up --profile my_rover --body {Path(__file__).resolve()}:Rover')
    return 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('action', nargs='?', choices=('demo', 'install'), default='demo')
    ap.add_argument('--mujoco', action='store_true', help='the demo in the MuJoCo simulation of my_rover.xml')
    a = ap.parse_args()
    sys.exit(install() if a.action == 'install' else demo(a.mujoco))
