"""The built-in providers: `hold` (a self-test that moves nothing) and `replay` (plays back a recorded episode's actions).

    roborun policy run --provider hold --arms left --seconds 5 --prompt "self-test"
    roborun policy run --provider replay --option path=episode.npz --option relative=true --prompt "replay"

replay reads either an .npz with `action` (T, D) and optionally `names` (D,) and `hz`, or one episode of a LeRobot dataset
(`path=<dataset root>`, `episode=N`; needs the dataset extra for parquet). Columns are matched by name: <side>_joint<i> and
<side>_gripper, as Roborun's datasets write them. Dataset grippers are in the station's units and are converted to the open
fraction with the arm's jaw range; an .npz is taken as already in fractions unless `gripper_units=station`. With
`relative=true` the rows are shifted so that the first one starts where the arm is (joints only). After the last row the arm
holds its last target. Standard library only, except for reading the files.
"""
from __future__ import annotations

import json
from pathlib import Path

from .provider import Chunk, PolicyRefused, Provider, plain_needs

HORIZON = 48
HZ = 30.0


def _sides(spec, arms):
    have = [a.side for a in spec.arms]
    if arms in (None, '', 'all', 'both'):
        return tuple(have)
    wanted = tuple(arms) if isinstance(arms, (list, tuple)) else (str(arms),)
    missing = [s for s in wanted if s not in have]
    if missing:
        raise PolicyRefused(f'this run has no {missing[0]} arm')
    return wanted


class HoldProvider(Provider):
    """Every chunk is the target the station is already playing: the arms stay where they are."""
    name = 'hold'

    def __init__(self, arms=None, horizon=30, hz=HZ):
        self.arms, self.horizon, self.hz = arms, int(horizon), float(hz)

    def needs(self, spec):
        self.sides = _sides(spec, self.arms)
        self.gripper = {a.side: a.gripper for a in spec.arms}
        return plain_needs(self.sides, frame_hz=2.0, version='hold (built-in)')

    def act(self, obs):
        rows = {}
        for side in self.sides:
            state = obs.arms[side]
            row = list(state.commanded_rad) + ([state.commanded_gripper] if self.gripper[side] else [])
            rows[side] = [row] * self.horizon
        return Chunk(rows, hz=self.hz)


class ReplayProvider(Provider):
    """Plays back recorded actions, timed from the first act() (row i at i / hz)."""
    name = 'replay'

    def __init__(self, path, episode=0, hz=None, relative=False, gripper_units=None, horizon=HORIZON):
        self.path, self.episode, self.hz_given = Path(str(path)).expanduser(), int(episode), hz
        self.relative, self.units, self.horizon = bool(relative), gripper_units, int(horizon)
        self.t0 = None
        self.offset = {}

    # ---- reading the episode
    def _load(self):
        if self.path.suffix == '.npz':
            try:
                import numpy as np
            except ImportError as exc:
                raise PolicyRefused("replay of an .npz needs numpy: pip install 'roborun-sdk[robot]'") from exc
            with np.load(self.path, allow_pickle=False) as z:
                actions = [list(map(float, r)) for r in z['action']]
                names = [str(n) for n in z['names']] if 'names' in z.files else None
                hz = float(z['hz']) if 'hz' in z.files else None
            return actions, names, hz, self.units or 'fraction'
        info = json.loads((self.path / 'meta' / 'info.json').read_text())
        names = (info.get('features', {}).get('action') or {}).get('names')
        chunk = self.episode // int(info.get('chunks_size') or 1000)
        data = self.path / info['data_path'].format(episode_chunk=chunk, episode_index=self.episode)
        try:
            import pyarrow.parquet as pq
        except ImportError as exc:
            raise PolicyRefused("replay of a dataset episode needs the dataset tools: pip install 'roborun-sdk[dataset]'") from exc
        column = pq.read_table(data, columns=['action']).column('action').to_pylist()
        return [list(map(float, r)) for r in column], names, float(info['fps']), self.units or 'station'

    def needs(self, spec):
        if not self.path.exists():
            raise PolicyRefused(f'replay: there is no {self.path}')
        actions, names, hz, units = self._load()
        if not actions:
            raise PolicyRefused('replay: the episode has no action rows')
        self.hz = float(self.hz_given or hz or HZ)
        arms = {a.side: a for a in spec.arms}
        if names is None:                               # unnamed columns: left arm, then right, each joints (+ gripper)
            names, at = [], 0
            for a in spec.arms:
                names += [f'{a.side}_joint{i + 1}' for i in range(a.joints)] + ([f'{a.side}_gripper'] if a.gripper else [])
            if len(names) != len(actions[0]):
                raise PolicyRefused(f'replay: {len(actions[0])} action columns, but this run\'s arms take {len(names)}')
        cols = {}
        for side, a in arms.items():
            want = [f'{side}_joint{i + 1}' for i in range(a.joints)] + ([f'{side}_gripper'] if a.gripper else [])
            if all(n in names for n in want):
                cols[side] = [names.index(n) for n in want]
        if not cols:
            raise PolicyRefused('replay: the episode has no columns for this run\'s arms '
                                f'(it has {", ".join(names[:4])}...)')
        self.sides = tuple(s for s in ('left', 'right') if s in cols)
        self.rows = {}
        for side in self.sides:
            a = arms[side]
            table = [[row[i] for i in cols[side]] for row in actions]
            if a.gripper and units == 'station':
                lo, hi = a.jaw_range
                table = [r[:-1] + [(r[-1] - lo) / (hi - lo)] for r in table]
            self.rows[side] = table
        self.gripper = {s: arms[s].gripper for s in self.sides}
        return plain_needs(self.sides, frame_hz=2.0, version='replay (built-in)')

    def act(self, obs):
        if self.t0 is None:
            self.t0 = obs.t
            if self.relative:
                for side in self.sides:
                    n = len(obs.arms[side].commanded_rad)
                    self.offset[side] = [c - r for c, r in zip(obs.arms[side].commanded_rad, self.rows[side][0][:n])]
        start = max(0, int((obs.t - self.t0) * self.hz))
        out = {}
        for side in self.sides:
            table = self.rows[side]
            off = self.offset.get(side)
            chunk = []
            for i in range(start, start + self.horizon):
                row = list(table[min(i, len(table) - 1)])
                if off:
                    row = [v + d for v, d in zip(row, off)] + row[len(off):]
                chunk.append(row)
            out[side] = chunk
        return Chunk(out, hz=self.hz, meta=dict(row=start, of=len(self.rows[self.sides[0]])))
