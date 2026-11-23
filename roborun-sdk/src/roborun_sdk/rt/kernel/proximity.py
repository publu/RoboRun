"""K4 proximity on fresh samples.

At every write the moving arm is swept from its *measured* pose to its *commanded* target and checked against the
other arm's freshest measured pose *and* its commanded pose, the table, the obstacles and the keep-in box.
Rule: inter-arm >= c + age_charge x oldest sample age (20 mm + 0.45 m/s x age); a sample older
than 150 ms is a typed fresh-pose hold (the runtime spends a 0.5 s budget per segment, then ends the segment).
An arm without a fresh sample is an obstacle of unknown pose: nothing moves near it.

Two checkers:
- ``MixedSafetyProximity``: the robot runtime's mixed-arm check (MuJoCo convex hulls of the real meshes, the Metal's
  70 mm envelopes), loaded read-only by file path from the runtime folder for a YAM/Metal pair.
- ``CapsuleProximity``: the pack's capsules (generic; sim and any arm).
"""
from __future__ import annotations

import importlib.util
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..datatypes import ArmSample
from ..geometry import clearances
from ..pack import Pack


@dataclass
class ProximityResult:
    ok: bool
    reason: str | None = None          # stale | collision | other_arm_unknown | keep_in | checker_error
    message: str = ""
    min_gap_m: float | None = None
    required_m: float | None = None
    table_m: float | None = None
    obstacle_m: float | None = None
    ages_ms: dict = field(default_factory=dict)
    checker: str = ""
    duration_ms: float | None = None

    def public(self) -> dict:
        out = {"ok": self.ok, "checker": self.checker}
        for key in ("reason", "min_gap_m", "required_m", "table_m", "obstacle_m", "duration_ms"):
            v = getattr(self, key)
            if v is not None:
                out[key] = round(v, 5) if isinstance(v, float) else v
        if self.message:
            out["message"] = self.message[:300]
        if self.ages_ms:
            out["ages_ms"] = self.ages_ms
        return out


def ages(samples: dict, now: float) -> dict:
    return {arm: now - s.t + s.clock_uncertainty_s for arm, s in samples.items() if s is not None}


def _densify(q_from, q_to, step_rad: float) -> np.ndarray:
    q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
    n = max(1, int(math.ceil(float(np.max(np.abs(q_to - q_from))) / step_rad)))
    return np.linspace(q_from, q_to, n + 1)


def densify_path(rows, step_rad: float) -> np.ndarray:
    rows = np.asarray(rows, float)
    out = [rows[:1]]
    for a, b in zip(rows[:-1], rows[1:]):
        out.append(_densify(a, b, step_rad)[1:])
    return np.vstack(out)


class CapsuleProximity:
    name = "capsule"

    def __init__(self, pack: Pack, *, step_rad: float = 0.01):
        self.pack = pack
        self.step_rad = step_rad

    def _setup(self, arm: str, samples: dict, now: float):
        prox = self.pack.proximity
        a = ages(samples, now)
        missing = [name for name in self.pack.arms if samples.get(name) is None]
        if missing:
            return None, ProximityResult(False, "other_arm_unknown", f"no sample for {', '.join(missing)}",
                                         checker=self.name)
        stale = {k: v for k, v in a.items() if v > prox["max_sample_age_s"]}
        if stale:
            return None, ProximityResult(False, "stale", "samples older than the freshness bound",
                                         ages_ms={k: round(v * 1000, 1) for k, v in a.items()}, checker=self.name)
        geom = self.pack.arms[arm].geometry
        if geom is None or self.pack.scene is None:
            return None, ProximityResult(False, "checker_error", "pack has no geometry for the capsule check",
                                         checker=self.name)
        others = []
        for name, other in self.pack.arms.items():
            if name == arm:
                continue
            if other.geometry is None:
                return None, ProximityResult(False, "other_arm_unknown", f"{name} has no geometry", checker=self.name)
            s = samples[name]
            others.append((name, other.geometry, np.asarray(s.q, float), None if s.q_cmd is None else np.asarray(s.q_cmd, float)))
        required = prox["c_m"] + prox["age_charge_m_s"] * max(a.values())
        return (geom, others, required, a), None

    def _sweep(self, geom, poses, others, required, a, started) -> ProximityResult:
        scene = self.pack.scene
        worst = {"inter_arm_m": math.inf, "table_m": math.inf, "obstacle_m": math.inf, "keep_in_m": math.inf,
                 "obstacle_margin_m": math.inf}
        variants = []
        for name, g, q_meas, q_cmd in others:
            variants.append((g, q_meas))
            if q_cmd is not None and np.max(np.abs(q_cmd - q_meas)) > 1e-6:
                variants.append((g, q_cmd))
        for q in poses:
            for g, qo in variants:
                c = clearances(geom, q, [(g, qo)], scene)
                for k in worst:
                    worst[k] = min(worst[k], c[k])
            if not variants:
                c = clearances(geom, q, [], scene)
                for k in worst:
                    worst[k] = min(worst[k], c[k])
        res = ProximityResult(True, min_gap_m=worst["inter_arm_m"] if math.isfinite(worst["inter_arm_m"]) else None,
                              required_m=required, table_m=worst["table_m"] if math.isfinite(worst["table_m"]) else None,
                              obstacle_m=worst["obstacle_m"] if math.isfinite(worst["obstacle_m"]) else None,
                              ages_ms={k: round(v * 1000, 1) for k, v in a.items()}, checker=self.name,
                              duration_ms=round((time.perf_counter() - started) * 1000, 3))
        if worst["inter_arm_m"] < self.pack.proximity["c_m"]:
            res.ok, res.reason, res.message = False, "collision", "inter-arm clearance below the minimum gap"
        elif worst["table_m"] < scene.table_gap_m:
            res.ok, res.reason, res.message = False, "collision", "table clearance below the minimum gap"
        elif worst["obstacle_margin_m"] < 0:
            res.ok, res.reason, res.message = False, "collision", "obstacle clearance below its minimum gap"
        elif worst["keep_in_m"] < 0:
            res.ok, res.reason, res.message = False, "keep_in", "a capsule leaves the keep-in box"
        elif worst["inter_arm_m"] < required:
            # nominal geometry is clear; only the sample-age charge refuses: hold and refresh
            res.ok, res.reason, res.message = False, "stale", "needs a fresher opposite-arm pose"
        return res

    def check(self, arm: str, q_from, q_to, samples: dict, now: float) -> ProximityResult:
        started = time.perf_counter()
        setup, refusal = self._setup(arm, samples, now)
        if refusal is not None:
            return refusal
        geom, others, required, a = setup
        n = self.pack.arms[arm].n
        poses = _densify(np.asarray(q_from, float)[:n], np.asarray(q_to, float)[:n], self.step_rad)
        return self._sweep(geom, poses, others, required, a, started)

    def check_path(self, arm: str, rows, samples: dict, now: float) -> ProximityResult:
        started = time.perf_counter()
        setup, refusal = self._setup(arm, samples, now)
        if refusal is not None:
            return refusal
        geom, others, required, a = setup
        n = self.pack.arms[arm].n
        start = np.asarray(samples[arm].q, float)[None, :n]
        poses = densify_path(np.vstack([start, np.asarray(rows, float)[:, :n]]), self.step_rad)
        return self._sweep(geom, poses, others, required, a, started)


# ---------------------------------------------------------------- the robot runtime's mesh check, wrapped

def _load_by_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module


class MixedSafetyProximity:
    """Runs the robot runtime's mesh check (left YAM + right Metal) exactly as the owners do. Loaded read-only by file path: only
    its kinematics and mesh-check modules enter sys.modules (MuJoCo + numpy), never the runtime's other modules.

    Per write: ``validate_path`` (public) from the measured pose to the target, once against the other arm's measured
    pose and once against its commanded pose. Whole-path acceptance sweeps use ``_dense``/``_limits``/``_geometry``
    (private, the same code validate_path runs) because ``validate_path`` caps a path at a 5 degree manual jog.
    """
    name = "mixed_safety"

    def __init__(self, dreamapi_root: str | Path):
        root = Path(dreamapi_root)
        if not (root / "mixed_safety.py").is_file() or not (root / "kin.py").is_file():
            raise FileNotFoundError(f"mixed_safety.py / kin.py not found under {root}")
        self.root = root
        self.kin = sys.modules.get("kin") or _load_by_path("kin", root / "kin.py")
        self.ms = sys.modules.get("mixed_safety") or _load_by_path("mixed_safety", root / "mixed_safety.py")
        for needed in ("validate_path", "_measured", "_dense", "_limits", "_geometry", "_rig", "FreshnessOnlyError"):
            if not hasattr(self.ms, needed):
                raise ImportError(f"mixed_safety has no {needed}; the wrapper needs re-review")

    def warm(self) -> None:
        """Compile the MuJoCo scene once (seconds) before the first write needs it."""
        self.ms._rig()

    @staticmethod
    def _rows(samples: dict) -> dict:
        rows = {}
        for arm, s in samples.items():
            if s is None or s.raw is None:
                raise ValueError(f"{arm} sample has no owner row for mixed_safety")
            rows[arm] = dict(s.raw)
        return rows

    def _map(self, exc: Exception, started: float) -> ProximityResult:
        stale = isinstance(exc, self.ms.FreshnessOnlyError)
        return ProximityResult(False, "stale" if stale else "collision", str(exc), checker=self.name,
                               duration_ms=round((time.perf_counter() - started) * 1000, 3))

    def check(self, arm: str, q_from, q_to, samples: dict, now: float) -> ProximityResult:
        started = time.perf_counter()
        try:
            rows = self._rows(samples)
        except ValueError as exc:
            return ProximityResult(False, "other_arm_unknown", str(exc), checker=self.name)
        q_from = [float(v) for v in list(q_from)[:6]]
        q_to = [float(v) for v in list(q_to)[:6]]
        results = []
        variants = [rows]
        for other, row in rows.items():
            cmd = row.get("commanded_rad")
            if other != arm and isinstance(cmd, list) and len(cmd) == 6 and \
                    max(abs(a - b) for a, b in zip(cmd, row["positions_rad"])) > 1e-6:
                alt = {k: dict(v) for k, v in rows.items()}
                alt[other]["positions_rad"] = list(cmd)
                variants.append(alt)
        try:
            for variant in variants:
                results.append(self.ms.validate_path(arm, [q_from, q_to], q_from, variant, now=now)["safety"])
        except Exception as exc:
            if isinstance(exc, ValueError):
                return self._map(exc, started)
            return ProximityResult(False, "checker_error", f"{type(exc).__name__}: {exc}", checker=self.name)
        return self._result(results, started)

    def _result(self, metas: list, started: float) -> ProximityResult:
        gaps = [m["minimum_clearance_mm"] for m in metas]
        def worst(key):
            vals = [g.get(key) for g in gaps if g.get(key) is not None]
            return min(vals) / 1000 if vals else None
        return ProximityResult(True, min_gap_m=worst("inter_arm_m"),
                               required_m=max(m["inter_arm_required_mm"] for m in metas) / 1000,
                               table_m=worst("table_m"), obstacle_m=worst("obstacle_m"),
                               ages_ms=metas[0].get("source_ages_ms", {}), checker=self.name,
                               duration_ms=round((time.perf_counter() - started) * 1000, 3))

    def check_path(self, arm: str, rows, samples: dict, now: float) -> ProximityResult:
        started = time.perf_counter()
        ms = self.ms
        try:
            raw = self._rows(samples)
        except ValueError as exc:
            return ProximityResult(False, "other_arm_unknown", str(exc), checker=self.name)
        try:
            rig = ms._rig()
            measured, a = ms._measured(raw, now)
            path = np.asarray(rows, float)[:, :6]
            # the same 0.25 degree densification as the runtime's own check, without its 1000-row manual-jog cap
            dense = densify_path(np.vstack([measured[arm][None], path]), float(getattr(ms, "STEP_RAD", math.radians(.25))))
            ms._limits(rig, arm, dense, measured[arm])
            others = [measured]
            for other, row in raw.items():
                cmd = row.get("commanded_rad")
                if other != arm and isinstance(cmd, list) and len(cmd) == 6:
                    alt_measured = dict(measured)
                    alt_measured[other] = np.asarray(cmd, float)
                    others.append(alt_measured)
            metas = []
            for variant in others:
                for start in range(0, len(dense), 400):
                    metas.append(ms._geometry(rig, arm, dense[start:start + 400], variant, a, raw))
        except Exception as exc:
            if isinstance(exc, ValueError):
                return self._map(exc, started)
            return ProximityResult(False, "checker_error", f"{type(exc).__name__}: {exc}", checker=self.name)
        return self._result(metas, started)
