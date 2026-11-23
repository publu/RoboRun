"""Per-arm sample history and the rest test.

The rest test is a position window, not a velocity threshold: velocity is quantised in 0.00244 rad/s steps and
a YAM at rest reads 0.0073-0.0659 rad/s, so |v| <= 1e-4 refuses every start. Rule:
<= 1.5 mrad of joint motion (and <= 0.003 of jaw) over >= 0.3 s, the window video_capture already uses.
"""
from __future__ import annotations

from collections import deque

import numpy as np

from .datatypes import ArmSample


class History:
    def __init__(self, horizon_s: float = 3.0, maxlen: int = 512):
        self.horizon_s = horizon_s
        self.rows: deque = deque(maxlen=maxlen)     # (t, q ndarray, jaw float|nan, sample_id)
        self.last_id = None

    def add(self, s: ArmSample) -> bool:
        """Record a new driver sample; returns False for a repeat or an out-of-order stamp."""
        if s.sample_id == self.last_id or (self.rows and s.t <= self.rows[-1][0]):
            return False
        self.last_id = s.sample_id
        jaw = float("nan") if s.jaw is None else float(s.jaw)
        self.rows.append((s.t, np.asarray(s.q, float), jaw, s.sample_id))
        while self.rows and self.rows[-1][0] - self.rows[0][0] > self.horizon_s:
            self.rows.popleft()
        return True

    def latest(self):
        return self.rows[-1] if self.rows else None

    def window(self, span_s: float) -> list:
        """The newest samples covering ``span_s``: includes the last sample at or before (newest - span)."""
        if not self.rows:
            return []
        end = self.rows[-1][0]
        out = []
        for row in reversed(self.rows):
            out.append(row)
            if row[0] <= end - span_s:
                break
        out.reverse()
        return out

    def at_rest(self, now: float, *, window_s: float, range_rad: float, jaw_range: float, max_age_s: float) -> tuple[bool, dict]:
        rows = self.window(window_s)
        if not rows:
            return False, {"why": "no_samples"}
        newest = rows[-1][0]
        if now - newest > max_age_s:
            return False, {"why": "stale", "age_s": round(now - newest, 4)}
        span = newest - rows[0][0]
        if span + 1e-9 < window_s:
            return False, {"why": "window_not_covered", "span_s": round(span, 4)}
        q = np.stack([r[1] for r in rows])
        joint = float(np.max(np.ptp(q, axis=0)))
        jaws = np.array([r[2] for r in rows])
        jaw = float(np.ptp(jaws)) if np.isfinite(jaws).all() else 0.0
        ok = joint <= range_rad + 1e-12 and jaw <= jaw_range + 1e-12
        return ok, {"why": None if ok else "moving", "span_s": round(span, 4), "joint_range_rad": round(joint, 6),
                    "jaw_range": round(jaw, 6), "samples": len(rows)}

    def range_over(self, span_s: float, column: int | None = None) -> float | None:
        """ptp of a joint (column) or of the jaw (column None) over the newest ``span_s``."""
        rows = self.window(span_s)
        if len(rows) < 2 or rows[-1][0] - rows[0][0] + 1e-9 < span_s:
            return None
        if column is None:
            v = np.array([r[2] for r in rows])
        else:
            v = np.array([r[1][column] for r in rows])
        return float(np.ptp(v)) if np.isfinite(v).all() else None

    def delta_over(self, span_s: float, column: int | None = None) -> float | None:
        rows = self.window(span_s)
        if len(rows) < 2 or rows[-1][0] - rows[0][0] + 1e-9 < span_s:
            return None
        pick = (lambda r: r[2]) if column is None else (lambda r: r[1][column])
        return float(pick(rows[-1]) - pick(rows[0]))
