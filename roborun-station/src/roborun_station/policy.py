"""The station's `policy` lease: timed chunks of joint + gripper rows pushed by a policy runner, played on the station's tick.

A policy run (roborun_sdk.policy) pushes chunks: per leased arm, rows of [q_1..q_n, gripper open fraction] at `hz`, whose first
row was computed from an observation `skip_s` seconds before the push. The station checks every chunk before it touches a
target (`check_chunk`: the same function the runner's shadow mode uses), converts the gripper fraction to its own jaw units
(`jaw_from_fraction`), and plays the rows on its own tick (`PolicyBuffer`): linear interpolation at the tick time, then a
per-tick clip of the step from the last target. An empty buffer keeps the last target, so the arm holds; nothing ever jumps.
Everything after the station (the driver's jerk shaping, joint clamp, tool speed cap and rules file) applies unchanged.

The limits are module constants; the lease's start reply echoes them (`limits()`), so the runner and its records use the
values that were enforced. Standard library only.
"""
from __future__ import annotations

import math

POLICY_MAX_S = 600.0               # one lease at most this long; the heartbeat keeps it alive
POLICY_HEARTBEAT_S = 1.0           # no heartbeat for this long: client_disconnected, every leased arm holds
POLICY_MAX_ROWS = 512              # rows per arm in one push (48-row chunks are the norm)
POLICY_MAX_HORIZON_S = 10.0        # no chunk plans further ahead than this
POLICY_MAX_AGE_S = 3.0             # the oldest observation a chunk may come from (its skip_s)
POLICY_SEAM_RAD = 0.5              # the first row still due vs the current target, on any joint; more is refused
POLICY_DRY_END_S = 10.0            # an empty buffer this long ends the lease (policy_error)
POLICY_TOOL_SPEED_M_S = 0.25       # the default tool speed cap of a policy run (a request can lower it, never raise it)
POLICY_REFUSALS_END = 5            # this many refused chunks in a row end the lease (policy_error)
POLICY_MIN_HZ, POLICY_MAX_HZ = 5.0, 200.0
GRIPPER_LO, GRIPPER_HI = -0.05, 1.05   # an open fraction outside this is refused (checked before conversion)
JOINT_STEP = 0.04                  # rad per tick: the clip (the station's per-answer bound, used here as a clip)
JAW_STEP = 0.081                   # station jaw units per tick


def limits() -> dict:
    """The limits this station enforces for policy leases."""
    return dict(max_s=POLICY_MAX_S, heartbeat_s=POLICY_HEARTBEAT_S, max_rows=POLICY_MAX_ROWS,
                max_horizon_s=POLICY_MAX_HORIZON_S, max_age_s=POLICY_MAX_AGE_S, seam_rad=POLICY_SEAM_RAD,
                dry_end_s=POLICY_DRY_END_S, tool_speed_m_s=POLICY_TOOL_SPEED_M_S, refusals_end=POLICY_REFUSALS_END,
                min_hz=POLICY_MIN_HZ, max_hz=POLICY_MAX_HZ, gripper_range=[GRIPPER_LO, GRIPPER_HI],
                joint_step_rad=JOINT_STEP, jaw_step=JAW_STEP)


# ---- the gripper: station jaw units <-> an open fraction 0 (closed) .. 1 (open) -------------------------------------------
def jaw_range_usable(jaw_range) -> bool:
    try:
        closed, opened = (float(v) for v in jaw_range)
    except (TypeError, ValueError):
        return False
    return math.isfinite(closed) and math.isfinite(opened) and abs(opened - closed) > 1e-9


def fraction_from_jaw(jaw, jaw_range) -> float:
    """A station jaw reading as an open fraction (0 closed, 1 open)."""
    closed, opened = (float(v) for v in jaw_range)
    return (float(jaw) - closed) / (opened - closed)


def jaw_from_fraction(fraction, jaw_range) -> float:
    """An open fraction as a station jaw target, clamped to the jaw's range."""
    closed, opened = (float(v) for v in jaw_range)
    jaw = closed + float(fraction) * (opened - closed)
    lo, hi = sorted((closed, opened))
    return max(lo, min(hi, jaw))


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


class ChunkRefused(ValueError):
    """A chunk the station will not play; the lease continues (the buffer plays on or holds)."""

    def __init__(self, why: str, code: str):
        super().__init__(why)
        self.code = code
        self.reply = {"refused": why, "code": code}


def check_chunk(chunk: dict, *, sides, joints: dict, grippers: dict, jaw_ranges: dict, commanded: dict, arrival: float):
    """The chunk's rows still to play, per side, in station units: {side: [(due, q, jaw or None), ...]}, plus how many rows
    were already due. Raises ChunkRefused with the reason. `commanded`: side -> (q, jaw) the station targets now; `arrival`:
    the station clock when the chunk arrived. Row i is due at arrival - skip_s + i / hz."""
    rows, hz, skip = chunk.get("rows"), chunk.get("hz"), chunk.get("skip_s", 0.0)
    if not isinstance(rows, dict):
        raise ChunkRefused("rows must be {side: [[q.., gripper], ...]}", "malformed")
    extra = sorted(set(rows) - set(sides))
    if extra:
        raise ChunkRefused(f"the {extra[0]} arm is not in this lease", "wrong_side")
    missing = sorted(set(sides) - set(rows))
    if missing:
        raise ChunkRefused(f"no rows for the {missing[0]} arm, which this lease drives", "missing_side")
    if not _finite(hz) or not POLICY_MIN_HZ <= hz <= POLICY_MAX_HZ:
        raise ChunkRefused(f"hz must be between {POLICY_MIN_HZ:g} and {POLICY_MAX_HZ:g}", "hz")
    if not _finite(skip) or skip < 0:
        raise ChunkRefused("skip_s must be a number >= 0", "malformed")
    lengths = set()
    for side in sides:
        table = rows[side]
        if not isinstance(table, list) or not table:
            raise ChunkRefused(f"the {side} arm has no rows", "rows")
        if len(table) > POLICY_MAX_ROWS:
            raise ChunkRefused(f"{len(table)} rows for the {side} arm; at most {POLICY_MAX_ROWS}", "rows")
        lengths.add(len(table))
        width = joints[side] + (1 if grippers[side] else 0)
        for row in table:
            if not isinstance(row, list) or len(row) != width:
                raise ChunkRefused(f"each {side} row must have {width} values ({joints[side]} joints"
                                   + (" and the gripper)" if grippers[side] else ")"), "length")
            if not all(_finite(v) for v in row):
                raise ChunkRefused(f"a {side} row has a value that is not a finite number", "not_finite")
            if grippers[side] and not GRIPPER_LO <= row[-1] <= GRIPPER_HI:
                raise ChunkRefused(f"a {side} gripper value {row[-1]:g} is outside 0 (closed) .. 1 (open)", "gripper")
    if len(lengths) != 1:
        raise ChunkRefused("every arm needs the same number of rows", "rows")
    n = lengths.pop()
    if (n - 1) / hz > POLICY_MAX_HORIZON_S:
        raise ChunkRefused(f"the rows cover {(n - 1) / hz:.1f} s; at most {POLICY_MAX_HORIZON_S:g} s", "horizon")
    if skip > POLICY_MAX_AGE_S:
        raise ChunkRefused(f"the chunk is {skip:.2f} s old; at most {POLICY_MAX_AGE_S:g} s", "late")
    start = arrival - skip
    first = 0
    while first < n and start + first / hz < arrival - 1e-9:
        first += 1
    if first >= n:
        raise ChunkRefused(f"the chunk is {skip:.2f} s old and every row of it is already due", "late")
    out = {}
    for side in sides:
        q_now, jaw_now = commanded[side]
        nj = joints[side]
        row = rows[side][first]
        seam = max(abs(float(a) - float(b)) for a, b in zip(row[:nj], q_now))
        if seam > POLICY_SEAM_RAD:
            raise ChunkRefused(f"the {side} arm's first row is {seam:.2f} rad from where it is now; at most "
                               f"{POLICY_SEAM_RAD:g} rad", "seam")
        played = []
        for i in range(first, n):
            r = rows[side][i]
            jaw = jaw_from_fraction(r[nj], jaw_ranges[side]) if grippers[side] else None
            played.append((start + i / hz, [float(v) for v in r[:nj]], jaw))
        out[side] = played
    return out, first


class PolicyBuffer:
    """One arm's rows still to play: (due, q, jaw) on the station clock. `target(now, q_last, jaw_last, t_last)` is the
    interpolated target at `now` (None when nothing is left: the arm keeps its last target)."""

    def __init__(self):
        self.rows: list = []

    def replace(self, rows):
        self.rows = list(rows)                      # the rows still to come replace what was left (overlay)

    def clear(self):
        self.rows = []

    def seconds_left(self, now) -> float:
        return max(0.0, self.rows[-1][0] - now) if self.rows else 0.0

    def target(self, now, q_last, jaw_last, t_last):
        rows = self.rows
        while len(rows) >= 2 and rows[1][0] <= now:
            rows.pop(0)                             # keep the newest row already due as the start of the segment
        if not rows:
            return None
        if rows[0][0] <= now:
            if len(rows) == 1:                      # the last row: play it, then the buffer is empty
                due, q, jaw = rows.pop(0)
                return q, jaw
            (t0, q0, j0), (t1, q1, j1) = rows[0], rows[1]
        else:                                       # the first row is still ahead: blend from where the target is now
            t0, q0, j0 = t_last, q_last, jaw_last
            t1, q1, j1 = rows[0]
        span = t1 - t0
        u = 1.0 if span <= 1e-9 else max(0.0, min(1.0, (now - t0) / span))
        q = [a + u * (b - a) for a, b in zip(q0, q1)]
        jaw = None if j1 is None else (j1 if j0 is None else j0 + u * (j1 - j0))
        return q, jaw


def clip_step(q_last, jaw_last, q, jaw):
    """The step from the last target toward (q, jaw), at most JOINT_STEP per joint and JAW_STEP on the jaw.
    Returns (q, jaw, clipped)."""
    clipped = False
    out = []
    for a, b in zip(q_last, q):
        d = b - a
        if abs(d) > JOINT_STEP:
            d = math.copysign(JOINT_STEP, d)
            clipped = True
        out.append(a + d)
    if jaw is not None and jaw_last is not None:
        d = jaw - jaw_last
        if abs(d) > JAW_STEP:
            jaw = jaw_last + math.copysign(JAW_STEP, d)
            clipped = True
    return out, jaw, clipped
