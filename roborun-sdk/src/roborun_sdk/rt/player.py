"""Segment player on executed time.

Rows advance only while they are commanded; a hold never skips ahead (the vendor ``_install_chunk`` rule: after a
hold the plan resumes where the arm stopped). Each segment starts with a 0.2 s ramp from the measured joints and from
the jaw reference (the owner's present jaw command, which may squeeze past the measured width), never from the
measured jaw: ramping from a squeezed object's measured width would release the squeeze.

- ``rows30.v1``: tau advances by dt per commanded tick; the target is the rows interpolated at tau.
- ``path-rows.v1``: arc length s (max-norm over the joints) advances by ``step`` per 1/15 s while the target leads
  the measured pose by less than ``track_limit``; otherwise s holds (a tracking pause, as in the stream7d.v1 player).

The player only proposes targets. The runtime runs every proposal through K3/K4 before a connector sees it, and
only advances the player when the write actually happened -- and only by the fraction of the step K3 let through
(``commit(scale=)``): a segment the kernel slows follows its rows at the kernel's speed and still reaches the last
row (advancing the full step left a clamped reach 0.566 rad short, ``not_arrived``). Once the
rows are done the end gate first "settles" the command: it keeps writing the last row through K3/K4 until the command
holds it, or no bound lets it get closer by more than sensor noise for one rest window (``settle_idle_s``), and only
then times the stillness window.
"""
from __future__ import annotations

import math

import numpy as np

from .segment import Segment

RAMP_S = 0.2
TICK_15HZ = 1 / 15


class Playback:
    def __init__(self, seg: Segment, *, start_q7, n_joints: int, now: float, rate_hz: float = 30.0,
                 start_jaw: float | None = None, jaw_ref: float | None = None, jaw_allow: float | None = None,
                 arrival_tol=None, start_signals: dict | None = None):
        self.seg = seg
        self.start_jaw = start_jaw                # measured at acceptance (evidence)
        self.jaw_ref = jaw_ref                    # the most-closed jaw a write may hold (acceptance.jaw_reference)
        self.jaw_allow = jaw_allow                # noise allowance past it (never an opening)
        self.arrival_tol = None if arrival_tol is None else np.asarray(arrival_tol, float)
        self.start_signals = dict(start_signals or {})
        self.gate_evidence: dict = {}
        self.last_proximity = None
        self.n = n_joints
        self.dt = 1.0 / rate_hz
        self.started_t = now
        self.tau = 0.0                    # executed time (rows30) or arc length (path-rows)
        self.state = "running"            # running | paused | gate | ended
        self.outcome: str | None = None
        self.pause_until: float | None = None
        self.pause_trigger = None
        self.fresh_hold_used_s = 0.0
        self.tracking_pause_since: float | None = None
        self.gate_since: float | None = None      # the command holds the last row from here (stillness is timed)
        self.gate_opened: float | None = None     # the rows were done (the settle phase starts)
        self.settled = False                      # the command holds the last row (or K3 lets it get no closer)
        self._settle_ref: tuple[float, float, float] | None = None   # settle phase: (joint gap, jaw gap, since)
        self.last_target: np.ndarray | None = None
        self.clamps = 0
        self.writes = 0
        start = np.asarray(start_q7, float).copy()
        rows = seg.rows
        if not math.isfinite(start[-1]):
            start[-1] = rows[0, -1]
        self.offset0 = start - rows[0]   # blended away over the ramp
        if seg.format == "path-rows.v1":
            seg_len = np.max(np.abs(np.diff(rows[:, :n_joints], axis=0)), axis=1) if len(rows) > 1 else np.zeros(0)
            self.knots = np.concatenate([[0.0], np.cumsum(seg_len)])
            self.length = float(self.knots[-1])
            self.step_per_tick = float(seg.player["step"]) * self.dt / TICK_15HZ
            self.track_limit = float(seg.player["track_limit"])
            self.window_s = float(seg.player.get("window_s", 12.0))
        else:
            self.length = (len(rows) - 1) * seg.dt

    # ------------------------------------------------------------------ targets
    def _row_at(self, tau: float) -> np.ndarray:
        rows = self.seg.rows
        if self.seg.format == "rows30.v1":
            x = min(max(tau / self.seg.dt, 0.0), len(rows) - 1)
        else:
            if self.length <= 0:
                return rows[-1].copy()
            s = min(max(tau, 0.0), self.length)
            k = int(np.searchsorted(self.knots, s, side="right") - 1)
            k = min(max(k, 0), len(rows) - 2)
            span = self.knots[k + 1] - self.knots[k]
            x = k + (0.0 if span <= 0 else (s - self.knots[k]) / span)
        i = int(math.floor(x))
        if i >= len(rows) - 1:
            return rows[-1].copy()
        f = x - i
        return rows[i] * (1 - f) + rows[i + 1] * f

    def _ramp(self, tau: float) -> float:
        if self.seg.format == "rows30.v1":
            return max(0.0, 1.0 - tau / RAMP_S)
        elapsed = self.writes * self.dt
        return max(0.0, 1.0 - elapsed / RAMP_S)

    def proposal(self, measured_q=None) -> tuple[np.ndarray, float]:
        """Next target and the tau it corresponds to. Does not advance."""
        if self.seg.format == "rows30.v1":
            nxt = min(self.tau + self.dt, self.length)
        else:
            nxt = self.tau
            lead_ok = True
            if measured_q is not None and self.last_target is not None:
                lead = float(np.max(np.abs(self.last_target[:self.n] - np.asarray(measured_q, float)[:self.n])))
                lead_ok = lead <= self.track_limit
            if lead_ok:
                nxt = min(self.tau + self.step_per_tick, self.length)
        target = self._row_at(nxt) + self.offset0 * self._ramp(nxt)
        return target, nxt

    def commit(self, target: np.ndarray, tau: float, *, clamped: bool, scale: float = 1.0) -> None:
        """The runtime wrote ``target``: advance executed time by the fraction of the proposed step that was written
        (``scale``, K3's speed-clamp factor; 1 when nothing slowed it). The command then sits where the rows are at the
        new tau, so the next proposal continues along the rows and the last row is reached, never skipped."""
        self.last_target = np.asarray(target, float).copy()
        frac = min(max(float(scale), 0.0), 1.0) if math.isfinite(scale) else 0.0
        self.tau = tau if frac >= 1.0 or tau <= self.tau else self.tau + (tau - self.tau) * frac
        self.writes += 1
        if clamped:
            self.clamps += 1

    def rows_done(self) -> bool:
        return self.tau >= self.length - 1e-9

    def end(self, outcome: str) -> None:
        if self.state != "ended":
            self.state, self.outcome = "ended", outcome

    def end_target(self) -> np.ndarray:
        return self.seg.rows[-1].copy()

    def command_at_end(self, tol: float = 1e-9) -> bool:
        """The last write holds the last row (joints and jaw)."""
        return self.last_target is not None and float(np.max(np.abs(self.last_target - self.seg.rows[-1]))) <= tol

    def settle_idle_s(self, target, now: float, *, joint_eps: float, jaw_eps: float, moving: bool = False) -> float:
        """Settle phase: seconds since the command last got closer to the last row by more than the sensor-noise band
        (``joint_eps`` rad on the joints' max-norm gap, ``jaw_eps`` on the jaw gap; the runtime passes the pack's rest
        window, 1.5 mrad / 0.003). The gap is measured against the best gap so far, so slow steady progress still
        counts once it adds up, while a bound that follows a jittering reading (K3's jaw lead bound rides the measured
        jaw) does not look like progress (an exact-equality test kept a blocked close settling for the
        whole complete.timeout_s when the jaw reading jittered by 1e-4). ``moving``: K3's speed clamps cut this step,
        so the joints are still travelling at the cap -- progress however small the step."""
        d = np.abs(np.asarray(target, float) - self.seg.rows[-1])
        gap_j = float(np.max(d[:self.n])) if self.n else 0.0
        gap_w = float(d[self.n]) if d.shape[0] > self.n else 0.0
        ref = self._settle_ref
        if ref is None:
            self._settle_ref = (gap_j, gap_w, now)
            return 0.0
        if moving or gap_j < ref[0] - joint_eps or gap_w < ref[1] - jaw_eps:
            self._settle_ref = (min(gap_j, ref[0]), min(gap_w, ref[1]), now)
            return 0.0
        return now - ref[2]

    def public(self) -> dict:
        return {"segment": self.seg.id, "state": self.state, "tau": round(self.tau, 4), "length": round(self.length, 4),
                "writes": self.writes, "clamps": self.clamps, "outcome": self.outcome, "settled": self.settled}
