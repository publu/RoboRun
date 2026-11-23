"""The smallest arm driver Roborun accepts: a class with read() and command().

    roborun up --left /path/to/minimal_arm.py:MinimalArm          # or in robot.yaml:  arm: /path/to/minimal_arm.py:MinimalArm
    roborun station --adapter minimal_arm:MinimalArm               # just the station, from this folder

Roborun calls command() at its 30 Hz tick with a target that is already smooth and step-limited, and read() every tick.
A plain adapter like this one does not get the 100 Hz shaping, the guards or your rules file: build on StreamedArm for
those (see `roborun new` and docs/BUILDING.md). This one moves software joints at a bounded speed, so it runs anywhere.

    python minimal_arm.py        # runs it on a Roborun station for two seconds and prints the station's view of it
"""
import time

from roborun_sdk.embodiment import ArmAdapter


class MinimalArm(ArmAdapter):
    home = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]           # rad, one per joint
    limits = [(-2.6, 2.6)] * 6

    def __init__(self, bus=None):                     # bus: the arm's address from robot.yaml or --left ARM@ADDRESS
        self.bus = bus
        self.q, self.jaw = list(self.home), 1.0
        self.target, self.at = (list(self.home), 1.0), time.monotonic()

    def command(self, positions_rad, gripper):        # replace with: send these to your motors
        self.target = (list(positions_rad), float(gripper))

    def read(self):                                   # replace with: read your motors
        now = time.monotonic(); step = self.joint_speed_rad_s * min(0.1, now - self.at); self.at = now
        self.q = [a + max(-step, min(step, b - a)) for a, b in zip(self.q, self.target[0])]
        self.jaw += max(-step, min(step, self.target[1] - self.jaw))
        return dict(positions_rad=list(self.q), gripper=self.jaw)


if __name__ == '__main__':
    import json
    import urllib.request

    from roborun_sdk.embodiment import run_station

    arm = MinimalArm()
    arm.connect()
    station, server = run_station(arm, port=0)          # port 0: any free port, so it never meets a running robot
    try:
        time.sleep(2)
        with urllib.request.urlopen(server.url + '/supervisor/pose', timeout=3) as response:
            pose = json.load(response)
        print(json.dumps(dict(station=server.url, left=pose.get('left') or pose), default=str)[:600])
    finally:
        server.close()
