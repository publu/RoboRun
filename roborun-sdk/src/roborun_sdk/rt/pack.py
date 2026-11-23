"""The safety pack: the only arm-specific knowledge in the SDK.

Signed by the API's root key (pinned at enrollment), content-addressed, never edited in place. The kernel computes
for itself whether a new pack is tighter than the active one, field by field; it never trusts a label. Anything not
provably tighter is a loosening: staged, and activated only after a local operator confirmation at rest. Activation
of any pack happens only while every arm is holding and no grant is open.

Where a pack omits a value the kernel enforces a default, and the parser writes that default into the parsed pack
(``tcp_m_s``, ``thermal``), so the loosening rule always compares the values the kernel actually enforces: a new pack
that *adds* a loose value is a loosening, never "tighter than nothing".
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import signing
from .canonical import pack_digest
from .datatypes import OperatorConfirm, Refused
from .geometry import ArmGeometry, Capsule, Chain, Obstacle, SceneGeometry, box_as_capsule

SCHEMA = "manifest.safety-pack/1"
SIG_KIND = "safety-pack"

# Kernel tuning defaults: the design's numbers, used only where a pack omits the section (sim packs).
DEFAULT_PROXIMITY = {"c_m": 0.020, "age_charge_m_s": 0.45, "max_sample_age_s": 0.150, "fresh_hold_budget_s": 0.5}
DEFAULT_REST = {"window_s": 0.3, "range_rad": 0.0015, "jaw_range": 0.003}
DEFAULT_LINK = {"heartbeat_hz": 10, "lost_after_s": {"quic": 0.5, "wss": 1.0, "local": 1.0},
                "late_after_s": {"quic": 0.25, "wss": 0.5, "local": 0.5}, "brake_above_m_s": 0.10, "brake_max_s": 0.15}
DEFAULT_TCP_BASE_M_S = 0.04                 # K3: 0.04 m/s base; free_max / loaded_max default to the base when absent
DEFAULT_THERMAL_C = {"refuse_start": 70.0, "stop_work": (75.0, 80.0)}   # K6
HOME_SLACK_RAD = math.radians(5)            # the robot runtime's start-up slack: how far a Home may sit past a hard limit


def _num(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise Refused("pack_malformed", f"{name} must be a finite number")
    return float(value)


def _vec(value, n: int | None, name: str) -> np.ndarray:
    if not isinstance(value, (list, tuple)) or (n is not None and len(value) != n):
        raise Refused("pack_malformed", f"{name} must be a list of {n} numbers")
    return np.array([_num(v, name) for v in value], float)


@dataclass(frozen=True)
class JawSpec:
    unit: str
    open: float
    closed: float
    step_per_15hz: float
    lead_max: float
    force_cap: float | None

    @property
    def lo(self) -> float:
        return min(self.open, self.closed)

    @property
    def hi(self) -> float:
        return max(self.open, self.closed)

    def opening_direction(self) -> float:
        """+1 when larger values open the jaw."""
        return 1.0 if self.open >= self.closed else -1.0


@dataclass(frozen=True)
class ArmPack:
    name: str
    semantics: str
    lower: np.ndarray
    upper: np.ndarray
    margin_rad: float
    step_rad_per_15hz: float
    vel_rad_s: float
    point_step_m_per_15hz: float
    point_reach_m: np.ndarray | None
    tcp_m_s: dict
    jaw: JawSpec
    tracking_pause_rad: np.ndarray | None
    geometry: ArmGeometry | None
    z_m: float
    home_q: np.ndarray | None
    thermal_c: dict                 # as the pack wrote it
    connector: dict
    raw: dict = field(repr=False, compare=False, default_factory=dict)
    thermal: dict = field(default_factory=dict)          # effective K6 thresholds (defaults and native holds applied)
    arrival_rad: np.ndarray | None = None                # per-unit arrival tolerance (pack home.arrival_rad)

    @property
    def n(self) -> int:
        return len(self.lower)

    def soft_lower(self) -> np.ndarray:
        return self.lower + self.margin_rad

    def soft_upper(self) -> np.ndarray:
        return self.upper - self.margin_rad


@dataclass(frozen=True)
class Pack:
    raw: dict
    digest: str
    id: str
    rig: str
    supersedes: str | None
    origin: str
    arms: dict
    scene: SceneGeometry | None
    proximity: dict
    rest: dict
    link: dict
    trusted_keys: dict
    signed: bool
    key_not_after: dict = field(default_factory=dict)     # kid -> last valid day (checked again at use time)

    def arm(self, name: str) -> ArmPack:
        try:
            return self.arms[name]
        except KeyError:
            raise Refused("unknown_arm", f"arm {name!r} is not in pack {self.id}") from None


def _chain_and_capsules(arm: dict, name: str):
    chain_spec = arm.get("chain")
    caps = arm.get("capsules")
    if not isinstance(chain_spec, list) or not isinstance(caps, list):
        return None
    T = arm.get("base_T_table", arm.get("T_table_base"))   # the base's pose in the table frame (rig.json T_table_base)
    T = np.asarray(T if T is not None else np.eye(4), float)
    try:
        chain = Chain.from_pack(chain_spec, T)
    except (KeyError, ValueError, TypeError) as exc:
        raise Refused("pack_malformed", f"{name} chain: {exc}") from None
    capsules = []
    for c in caps:
        try:
            link = chain.link_index(c["link"])
        except ValueError as exc:
            raise Refused("pack_malformed", f"{name}: {exc}") from None
        if not -1 <= link < chain.n:
            raise Refused("pack_malformed", f"{name} capsule link {link} outside the chain")
        moving = bool(c.get("table", not chain.height_invariant(link)))
        capsules.append(Capsule(link, _vec(c["p0"], 3, "capsule p0"), _vec(c["p1"], 3, "capsule p1"),
                                _num(c["r"], "capsule r"), str(c.get("name") or ""), moving))
    if not capsules:
        raise Refused("pack_malformed", f"{name} has a chain but no capsules")
    return ArmGeometry(chain, capsules, _num(arm.get("z_m", 0.0), "z_m"))


def _tcp(arm: dict) -> dict:
    """Effective EE speed caps (K3): base 0.04 m/s; free_max and loaded_max default to the base when the pack omits
    them, which is what the kernel enforces without them."""
    given = {k: _num(v, f"tcp {k}") for k, v in (arm.get("tcp_m_s") or {}).items()}
    base = given.get("base", DEFAULT_TCP_BASE_M_S)
    return {"base": base, "free_max": given.get("free_max", base), "loaded_max": given.get("loaded_max", base)}


def _thermal(name: str, th: dict) -> dict:
    """Effective K6 thresholds, joints and jaw motor separately. Defaults: refuse starts at 70 C, stop work at 75 C.
    A native hold (``joints_hold`` / ``jaw_hold``: the Metal owner holds at 65 C on a joint and 55 C on the jaw)
    caps both, so the kernel never plans work the owner would interrupt."""
    def num(key, default):
        return _num(th[key], f"{name} thermal_c.{key}") if th.get(key) is not None else default
    refuse = num("refuse_start", DEFAULT_THERMAL_C["refuse_start"])
    stop = th.get("stop_work")
    stop_at = (min(_num(v, f"{name} thermal_c.stop_work") for v in stop) if isinstance(stop, (list, tuple)) and stop
               else num("stop_work", min(DEFAULT_THERMAL_C["stop_work"])))
    joint_stop = min(stop_at, num("joints_hold", stop_at))
    joint_refuse = min(refuse, joint_stop)
    jaw_stop = min(joint_stop, num("jaw_hold", joint_stop))
    jaw_refuse = min(joint_refuse, jaw_stop)
    return {"refuse_start": joint_refuse, "stop_work": joint_stop, "jaw_refuse_start": jaw_refuse,
            "jaw_stop_work": jaw_stop}


def _arm(name: str, arm: dict) -> ArmPack:
    if not isinstance(arm, dict):
        raise Refused("pack_malformed", f"arm {name} must be an object")
    sem = arm.get("semantics")
    if not isinstance(sem, str) or not sem:
        raise Refused("pack_malformed", f"arm {name} has no semantics id")
    limits = arm.get("limits") or {}
    if "lower_rad" in limits:
        lower = _vec(limits.get("lower_rad"), None, f"{name} lower_rad")
        upper = _vec(limits.get("upper_rad"), len(lower), f"{name} upper_rad")
    else:                                                   # the wire carries joint limits on the chain
        chain = arm.get("chain") or []
        lower = _vec([j.get("lower") for j in chain], None, f"{name} chain lower")
        upper = _vec([j.get("upper") for j in chain], len(lower), f"{name} chain upper")
    if len(lower) < 1 or np.any(lower >= upper):
        raise Refused("pack_malformed", f"arm {name} limits must have lower < upper")
    jaw = arm.get("jaw") or {}
    # K5 force cap: ``force_cap`` (YAM 0.5); else the native trip (Metal ``trip_nm`` 0.45 Nm). ``effort_nm`` (Metal
    # 0.30 Nm) is the owner's own closing-effort limit, native only: a held object squeezes at about that effort.
    cap = jaw.get("force_cap") if jaw.get("force_cap") is not None else jaw.get("trip_nm")
    jaw_spec = JawSpec(str(jaw.get("unit", "")), _num(jaw.get("open"), "jaw open"), _num(jaw.get("closed"), "jaw closed"),
                       _num(jaw.get("step", 0.04), "jaw step"), _num(jaw.get("lead_max", 0.03), "jaw lead_max"),
                       None if cap is None else _num(cap, "force_cap"))
    if jaw_spec.unit not in ("open_fraction", "motor_rad"):
        raise Refused("pack_malformed", f"arm {name} jaw unit must be open_fraction or motor_rad")
    reach = arm.get("point_reach_m")
    pause = arm.get("tracking_pause_rad")
    home = (arm.get("home") or {}).get("q")
    arrival = (arm.get("home") or {}).get("arrival_rad")
    home_q = None if home is None else _vec(home, len(lower), "home q")
    if home_q is not None and np.any((home_q < lower - HOME_SLACK_RAD) | (home_q > upper + HOME_SLACK_RAD)):
        # the kernel lets a joint rest at Home past a soft limit (K3), so a Home may sit only within the startup slack
        raise Refused("pack_malformed", f"arm {name} home is more than 5 degrees outside its joint limits")
    tcp = _tcp(arm)
    geometry = _chain_and_capsules(arm, name)
    if geometry is not None and geometry.chain.n != len(lower):
        raise Refused("pack_malformed", f"arm {name} chain has {geometry.chain.n} joints, limits {len(lower)}")
    thermal_c = dict(arm.get("thermal_c") or {})
    return ArmPack(
        name=name, semantics=sem, lower=lower, upper=upper,
        margin_rad=_num(limits.get("margin_rad", 0.01), "margin_rad"),
        step_rad_per_15hz=_num(limits.get("step_rad_per_15hz", 0.04), "step_rad_per_15hz"),
        vel_rad_s=_num(limits.get("vel_rad_s", 0.6), "vel_rad_s"),
        point_step_m_per_15hz=_num(arm.get("point_step_m_per_15hz", 0.02), "point_step_m_per_15hz"),
        point_reach_m=None if reach is None else _vec(reach, len(lower), "point_reach_m"),
        tcp_m_s=tcp, jaw=jaw_spec,
        tracking_pause_rad=None if pause is None else _vec(pause, len(lower), "tracking_pause_rad"),
        geometry=geometry, z_m=_num(arm.get("z_m", 0.0), "z_m"), home_q=home_q,
        thermal_c=thermal_c, connector=dict(arm.get("connector") or {}), raw=arm, thermal=_thermal(name, thermal_c),
        arrival_rad=None if arrival is None else _vec(arrival, len(lower), "home arrival_rad"))


def _scene(scene: dict | None) -> SceneGeometry | None:
    if not scene:
        return None
    table = scene.get("table") or {}
    obstacles = []
    for ob in scene.get("obstacles") or ():
        gap = _num(ob.get("min_gap_m", 0.02), "obstacle min_gap_m")
        if "capsule" in ob:
            c = ob["capsule"]
            obstacles.append(Obstacle(str(ob.get("name", "")), _vec(c["p0"], 3, "p0"), _vec(c["p1"], 3, "p1"),
                                      _num(c["r"], "r"), gap))
        elif "box" in ob:
            b = ob["box"]
            obstacles.append(box_as_capsule(str(ob.get("name", "")), _vec(b["pos"], 3, "pos"), _vec(b["half"], 3, "half"), gap))
        else:
            raise Refused("pack_malformed", "obstacles are capsules or boxes")
    keep = scene.get("keep_in")
    return SceneGeometry(table_z_m=_num(table.get("z_m", 0.0), "table z_m"),
                         table_gap_m=_num(table.get("min_gap_m", 0.02), "table min_gap_m"),
                         obstacles=tuple(obstacles),
                         keep_in_min=None if not keep else _vec(keep["min"], 3, "keep_in min"),
                         keep_in_max=None if not keep else _vec(keep["max"], 3, "keep_in max"))


def parse_pack(raw: dict, *, signed: bool = False) -> Pack:
    if not isinstance(raw, dict) or raw.get("schema") != SCHEMA:
        raise Refused("pack_malformed", f"expected schema {SCHEMA}")
    arms = raw.get("arms")
    if not isinstance(arms, dict) or not arms:
        raise Refused("pack_malformed", "a pack lists its arms")
    def merged(defaults, value):
        out = dict(defaults)
        out.update(value or {})
        return out
    link = merged(DEFAULT_LINK, raw.get("link"))
    for key in ("lost_after_s", "late_after_s"):
        sub = dict(DEFAULT_LINK[key])
        sub.update((raw.get("link") or {}).get(key) or {})
        link[key] = sub
    keys = signing.trusted_entries(raw.get("trusted_keys"))
    return Pack(raw=raw, digest=pack_digest(raw), id=str(raw.get("pack", "")), rig=str(raw.get("rig", "")),
                supersedes=raw.get("supersedes"), origin=str(raw.get("origin", "api")),
                arms={name: _arm(name, arm) for name, arm in arms.items()}, scene=_scene(raw.get("scene")),
                proximity=merged(DEFAULT_PROXIMITY, raw.get("proximity")), rest=merged(DEFAULT_REST, raw.get("rest")),
                link=link, trusted_keys={kid: pub for kid, (pub, _) in keys.items()}, signed=signed,
                key_not_after={kid: na for kid, (_, na) in keys.items()})


def load_pack(raw: dict, root_keys: dict[str, bytes], *, allow_unsigned: bool = False) -> Pack:
    """Verify the root signature, then parse. ``allow_unsigned`` is for sim/dev packs only."""
    ok, why = signing.verify(raw, SIG_KIND, root_keys)
    if not ok and not allow_unsigned:
        raise Refused("pack_signature", f"safety pack signature check failed ({why})")
    return parse_pack(raw, signed=ok)


# ---------------------------------------------------------------- the loosening rule

@dataclass
class PackDiff:
    tighter: bool = True
    loosened: list = field(default_factory=list)
    tightened: list = field(default_factory=list)
    info: list = field(default_factory=list)

    def loosen(self, what: str):
        self.tighter = False
        self.loosened.append(what)

    def check(self, what: str, ok: bool, changed: bool):
        if not ok:
            self.loosen(what)
        elif changed:
            self.tightened.append(what)


def _le(diff, what, new, old):
    new, old = np.atleast_1d(np.asarray(new, float)), np.atleast_1d(np.asarray(old, float))
    diff.check(what, new.shape == old.shape and bool(np.all(new <= old + 1e-12)), not np.array_equal(new, old))


def _ge(diff, what, new, old):
    new, old = np.atleast_1d(np.asarray(new, float)), np.atleast_1d(np.asarray(old, float))
    diff.check(what, new.shape == old.shape and bool(np.all(new >= old - 1e-12)), not np.array_equal(new, old))


def _contains(new_caps, old) -> bool:
    for c in new_caps:
        if c.link != old.link:
            continue
        for a, b in ((c.p0, c.p1), (c.p1, c.p0)):
            shift = max(float(np.linalg.norm(a - old.p0)), float(np.linalg.norm(b - old.p1)))
            if c.r >= old.r + shift - 1e-12:
                return True
    return False


def _same_chain(a: Chain, b: Chain) -> bool:
    if a.n != b.n or not np.allclose(a.T_table_base, b.T_table_base, atol=1e-12):
        return False
    return all(np.allclose(x.T_parent, y.T_parent, atol=1e-12) and np.allclose(x.axis, y.axis, atol=1e-12)
               for x, y in zip(a.joints, b.joints))


def _home_bounds(arm: ArmPack):
    lo, hi = arm.soft_lower(), arm.soft_upper()
    if arm.home_q is None:
        return lo, hi
    return np.minimum(lo, arm.home_q), np.maximum(hi, arm.home_q)


def compare(old: Pack, new: Pack) -> PackDiff:
    """Is ``new`` provably at least as tight as ``old``? Field by field; unknown changes count as loosening."""
    d = PackDiff()
    if set(new.arms) != set(old.arms):
        d.loosen("arms_changed")
    for name in sorted(set(new.arms) & set(old.arms)):
        a, b = old.arms[name], new.arms[name]
        p = f"arms.{name}."
        if a.semantics != b.semantics:
            d.loosen(p + "semantics")
            continue
        if a.lower.shape != b.lower.shape:
            d.loosen(p + "joint_count")
            continue
        _ge(d, p + "lower_rad", b.lower, a.lower)
        _le(d, p + "upper_rad", b.upper, a.upper)
        _ge(d, p + "margin_rad", b.margin_rad, a.margin_rad)
        _le(d, p + "step_rad_per_15hz", b.step_rad_per_15hz, a.step_rad_per_15hz)
        _le(d, p + "vel_rad_s", b.vel_rad_s, a.vel_rad_s)
        _le(d, p + "point_step_m_per_15hz", b.point_step_m_per_15hz, a.point_step_m_per_15hz)
        if a.point_reach_m is not None:
            if b.point_reach_m is None:
                d.loosen(p + "point_reach_m_removed")
            else:
                _ge(d, p + "point_reach_m", b.point_reach_m, a.point_reach_m)
        for key in ("base", "free_max", "loaded_max"):          # effective values: defaults were materialised
            _le(d, p + f"tcp_m_s.{key}", b.tcp_m_s[key], a.tcp_m_s[key])
        if (a.jaw.unit, a.jaw.open, a.jaw.closed) != (b.jaw.unit, b.jaw.open, b.jaw.closed):
            # a narrower jaw range is tighter; anything else is a loosening
            ok = a.jaw.unit == b.jaw.unit and b.jaw.lo >= a.jaw.lo and b.jaw.hi <= a.jaw.hi
            d.check(p + "jaw.range", ok, True)
        _le(d, p + "jaw.step", b.jaw.step_per_15hz, a.jaw.step_per_15hz)
        _le(d, p + "jaw.lead_max", b.jaw.lead_max, a.jaw.lead_max)
        if a.jaw.force_cap is not None:
            if b.jaw.force_cap is None:
                d.loosen(p + "jaw.force_cap_removed")
            else:
                _le(d, p + "jaw.force_cap", b.jaw.force_cap, a.jaw.force_cap)
        if a.tracking_pause_rad is not None:
            if b.tracking_pause_rad is None:
                d.loosen(p + "tracking_pause_removed")
            else:
                _le(d, p + "tracking_pause_rad", b.tracking_pause_rad, a.tracking_pause_rad)
        _ge(d, p + "z_m", b.z_m, a.z_m)
        for key in ("refuse_start", "stop_work", "jaw_refuse_start", "jaw_stop_work"):   # effective K6 thresholds
            _le(d, p + f"thermal.{key}", b.thermal[key], a.thermal[key])
        # K3 lets a joint rest at Home past a soft limit: a Home further out widens where joints may go
        a_lo, a_hi = _home_bounds(a)
        b_lo, b_hi = _home_bounds(b)
        _ge(d, p + "home_limit_lower", b_lo, a_lo)
        _le(d, p + "home_limit_upper", b_hi, a_hi)
        if a.geometry is not None:
            if b.geometry is None:
                d.loosen(p + "geometry_removed")
            else:
                if not _same_chain(a.geometry.chain, b.geometry.chain):
                    # Base transform or joint zeros changed: only provable with a Z growth sweep (API side).
                    d.loosen(p + "chain_or_base_changed")
                b_caps = [Capsule(c.link, c.p0, c.p1, c.r + b.z_m) for c in b.geometry.capsules]
                for cap in a.geometry.capsules:
                    if not _contains(b_caps, Capsule(cap.link, cap.p0, cap.p1, cap.r + a.z_m)):
                        d.loosen(p + f"capsule_not_contained:{cap.name or cap.link}")
        if (b.home_q is None) != (a.home_q is None) or (a.home_q is not None and not np.allclose(a.home_q, b.home_q)):
            d.info.append(p + "home_changed")
    so, sn = old.scene, new.scene
    if so is not None:
        if sn is None:
            d.loosen("scene_removed")
        else:
            _ge(d, "scene.table.z_m", sn.table_z_m, so.table_z_m)
            _ge(d, "scene.table.min_gap_m", sn.table_gap_m, so.table_gap_m)
            new_obs = [Capsule(-2, o.p0, o.p1, o.r) for o in sn.obstacles]
            for o in so.obstacles:
                if not _contains(new_obs, Capsule(-2, o.p0, o.p1, o.r)):
                    d.loosen(f"scene.obstacle_removed_or_shrunk:{o.name}")
                else:
                    match = [n for n in sn.obstacles if n.name == o.name]
                    if match and match[0].min_gap_m < o.min_gap_m - 1e-12:
                        d.loosen(f"scene.obstacle_gap:{o.name}")
            if so.keep_in_min is not None:
                if sn.keep_in_min is None:
                    d.loosen("scene.keep_in_removed")
                else:
                    _ge(d, "scene.keep_in.min", sn.keep_in_min, so.keep_in_min)
                    _le(d, "scene.keep_in.max", sn.keep_in_max, so.keep_in_max)
    _ge(d, "proximity.c_m", new.proximity["c_m"], old.proximity["c_m"])
    _ge(d, "proximity.age_charge_m_s", new.proximity["age_charge_m_s"], old.proximity["age_charge_m_s"])
    _le(d, "proximity.max_sample_age_s", new.proximity["max_sample_age_s"], old.proximity["max_sample_age_s"])
    _le(d, "proximity.fresh_hold_budget_s", new.proximity["fresh_hold_budget_s"], old.proximity["fresh_hold_budget_s"])
    _ge(d, "rest.window_s", new.rest["window_s"], old.rest["window_s"])
    _le(d, "rest.range_rad", new.rest["range_rad"], old.rest["range_rad"])
    _le(d, "rest.jaw_range", new.rest["jaw_range"], old.rest["jaw_range"])
    for section in ("lost_after_s", "late_after_s"):
        for key in sorted(old.link[section]):
            _le(d, f"link.{section}.{key}", new.link[section].get(key, math.inf), old.link[section][key])
    if set(new.trusted_keys) - set(old.trusted_keys):
        d.info.append("trusted_keys_added")
    return d


class PackStore:
    """Active pack plus at most one staged pack (K2 admission checks segments against ``active``)."""

    def __init__(self, active: Pack):
        self.active = active
        self.staged: Pack | None = None
        self.staged_diff: PackDiff | None = None
        self.history: list[dict] = []

    def offer(self, new: Pack, *, at_rest: bool, now: float) -> dict:
        """A new verified pack arrives. Activates only if provably tighter and the rig is at rest with no grant."""
        if new.digest == self.active.digest:
            return {"result": "current", "pack": new.digest}
        diff = compare(self.active, new)
        if diff.tighter and at_rest:
            return self._activate(new, diff, now, "tighter")
        self.staged, self.staged_diff = new, diff
        why = "loosening_needs_local_confirmation" if not diff.tighter else "waiting_for_rest"
        return {"result": "staged", "pack": new.digest, "why": why, "loosened": diff.loosened}

    def confirm(self, digest: str, confirm: OperatorConfirm, *, at_rest: bool, now: float) -> dict:
        if self.staged is None or self.staged.digest != digest:
            raise Refused("no_staged_pack", "no staged pack with that digest")
        if not isinstance(confirm, OperatorConfirm) or not confirm.operator:
            raise Refused("operator_confirmation_required", "a loosening pack needs a local operator confirmation")
        if not at_rest:
            raise Refused("not_at_rest", "packs activate only while every arm holds and no grant is open")
        new, diff = self.staged, self.staged_diff
        return self._activate(new, diff, now, "operator_confirmed", operator=confirm.operator)

    def maybe_activate_staged(self, *, at_rest: bool, now: float) -> dict | None:
        if self.staged is not None and self.staged_diff is not None and self.staged_diff.tighter and at_rest:
            return self._activate(self.staged, self.staged_diff, now, "tighter")
        return None

    def _activate(self, new: Pack, diff: PackDiff | None, now: float, how: str, **extra) -> dict:
        old = self.active
        self.active = new
        self.staged = self.staged_diff = None
        rec = {"result": "activated", "pack": new.digest, "previous": old.digest, "how": how, "t": now,
               "loosened": [] if diff is None else diff.loosened, **extra}
        self.history.append(rec)
        return rec
