"""Your own policy on Roborun: a provider that sways the wrist by 0.1 rad around where the arm is.

    roborun up --sim cup                                   # two simulated arms (or your own arms)
    roborun policy check --provider my_policy.py:Sway --arms left
    roborun policy run --provider my_policy.py:Sway --arms left --seconds 20 --prompt "sway the wrist"
    roborun policy run --provider my_policy.py:Sway --option amplitude=0.05 --arms left --prompt "sway less"
    roborun python my_policy.py                            # the same from Python

A provider gets observations (measured joints, the target the station last played, camera frames) and returns chunks: per arm,
rows of absolute targets [q_1..q_n, gripper] at `hz`, in radians, with the gripper as an open fraction (0 closed, 1 open).
Roborun's station plays them through every limit (the per-tick clip, the seam check, the joint and tool speed limits, your
rules file), holds the arm when a chunk is late, and Stop ends the run at once. Every run is a task: its record, its
recording and a dataset episode are on this computer like any other task's.
"""
import math

from roborun_sdk.policy import Chunk, Provider, plain_needs


class Sway(Provider):
    name = 'sway'

    def __init__(self, amplitude=0.1, period=4.0, hz=30.0):
        if not 0 < float(amplitude) <= 0.3:
            raise ValueError('amplitude: 0 .. 0.3 rad')
        self.amplitude, self.period, self.hz = float(amplitude), float(period), float(hz)
        self.start_t = None

    def needs(self, spec):
        """Offline: which arms it drives and what it wants to see. Here: every arm of the run, no cameras."""
        self.arms = {arm.side: arm for arm in spec.arms}
        return plain_needs(tuple(self.arms), frame_hz=2.0, version='sway example 1')

    def act(self, obs):
        """48 rows (1.6 s at 30 Hz) from the observation's time on: the last joint swings around where it started."""
        if self.start_t is None:
            self.start_t = obs.t
            self.center = {side: list(state.commanded_rad) for side, state in obs.arms.items()}
        rows = {}
        for side, arm in self.arms.items():
            out = []
            for i in range(48):
                t = obs.t - self.start_t + i / self.hz
                q = list(self.center[side])
                q[-1] += self.amplitude * math.sin(2 * math.pi * t / self.period)
                out.append(q + ([1.0] if arm.gripper else []))           # the gripper stays open
            rows[side] = out
        return Chunk(rows, hz=self.hz)


if __name__ == '__main__':
    from roborun_sdk import policy
    result = policy.run(Sway(), prompt='sway the wrist', arms='left', seconds=20)
    print(result.task_id, result.status, result.message)
