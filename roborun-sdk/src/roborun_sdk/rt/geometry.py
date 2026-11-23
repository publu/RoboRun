"""Capsule kernel: forward kinematics from pack data and capsule distances.

Generic code; every number comes from the safety pack: each joint's fixed transform (xyz + quaternion), its axis
and limits, the arm's base transform in the table frame, and capsules (link, p0, p1, r) attached to link frames.
No meshes, no IK, no MuJoCo. Two arms x 8 capsules check in about 0.2 ms.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def quat_matrix(q) -> np.ndarray:
    """(w, x, y, z) -> 3x3 rotation."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if not n > 0:
        raise ValueError("zero quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def axis_angle(axis, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rpy_matrix(rpy) -> np.ndarray:
    """Fixed-axis XYZ roll/pitch/yaw: R = Rz(yaw) Ry(pitch) Rx(roll), the URDF convention."""
    r, p, y = (float(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _transform(xyz, quat=None, rpy=None) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(rpy) if rpy is not None else quat_matrix(quat if quat is not None else (1.0, 0.0, 0.0, 0.0))
    T[:3, 3] = np.asarray(xyz, float)
    return T


@dataclass(frozen=True)
class Joint:
    name: str
    T_parent: np.ndarray       # fixed transform parent link -> this joint's frame (before rotation)
    axis: tuple[float, float, float]
    lower: float
    upper: float


class Chain:
    """A serial chain of revolute joints. Frame i is the link frame after joint i (index 0..n-1); frame -1 is the
    base. Optional extra fixed frames (e.g. a tool) hang off a named link."""

    def __init__(self, joints: list[Joint], T_table_base: np.ndarray, extra: dict[str, tuple[int, np.ndarray]] | None = None):
        if not joints:
            raise ValueError("empty chain")
        self.joints = joints
        self.T_table_base = np.asarray(T_table_base, float)
        if self.T_table_base.shape != (4, 4):
            raise ValueError("base transform must be 4x4")
        self.extra = extra or {}

    @property
    def n(self) -> int:
        return len(self.joints)

    def frames(self, q) -> list[np.ndarray]:
        q = np.asarray(q, float)
        if q.shape[0] < self.n:
            raise ValueError("joint vector too short")
        T = self.T_table_base
        out = []
        for i, joint in enumerate(self.joints):
            R = np.eye(4)
            R[:3, :3] = axis_angle(joint.axis, float(q[i]))
            T = T @ joint.T_parent @ R
            out.append(T)
        return out

    @classmethod
    def from_pack(cls, spec: list[dict], T_table_base) -> "Chain":
        """Joints as the wire carries them ({xyz, rpy, axis, lower, upper, parent, child}) or with ``quat``.
        A serial chain: each joint's parent must be the previous joint's child (or "base")."""
        joints = []
        previous = "base"
        for i, j in enumerate(spec):
            if "parent" in j and j["parent"] != previous:
                raise ValueError(f"joint {j.get('name')}: parent {j['parent']!r} breaks the serial chain")
            previous = j.get("child", f"link{i + 1}")
            axis = np.asarray(j["axis"], float)
            norm = float(np.linalg.norm(axis))
            if not norm > 0:
                raise ValueError("joint axis must be non-zero")
            axis = tuple(float(v) for v in axis / norm)
            T = _transform(j["xyz"], quat=j.get("quat"), rpy=j.get("rpy"))
            joints.append(Joint(str(j.get("name", f"j{i + 1}")), T, axis, float(j["lower"]), float(j["upper"])))
        chain = cls(joints, np.asarray(T_table_base, float))
        chain.links = ["base"] + [j.get("child", f"link{i + 1}") for i, j in enumerate(spec)]
        return chain

    def link_index(self, link) -> int:
        """'base' -> -1; a joint's child name -> its index; ints pass through."""
        if isinstance(link, bool):
            raise ValueError("bad link")
        if isinstance(link, int):
            return link
        names = getattr(self, "links", None) or ["base"] + [f"link{i + 1}" for i in range(self.n)]
        if link not in names:
            raise ValueError(f"capsule link {link!r} is not in the chain")
        return names.index(link) - 1

    def height_invariant(self, index: int) -> bool:
        """True for the base, and for the first link when joint 1 turns about the table's vertical: such capsules
        cannot change height (or leave a box) by moving, so the table and keep-in checks skip them."""
        if index < 0:
            return True
        if index == 0:
            R = self.T_table_base[:3, :3] @ self.joints[0].T_parent[:3, :3]
            return abs(float((R @ np.asarray(self.joints[0].axis))[2])) > 0.999
        return False


@dataclass(frozen=True)
class Capsule:
    link: int          # -1 = base frame; 0..n-1 = link after joint i
    p0: np.ndarray
    p1: np.ndarray
    r: float
    name: str = ""
    table: bool = True    # False: exempt from the table check (base, or a link whose height a vertical joint 1 cannot change)


class ArmGeometry:
    def __init__(self, chain: Chain, capsules: list[Capsule], z_m: float = 0.0):
        self.chain = chain
        self.capsules = capsules
        self.z_m = float(z_m)
        self._links = np.array([c.link for c in capsules])
        self._p0 = np.array([c.p0 for c in capsules], float).reshape(-1, 3)
        self._p1 = np.array([c.p1 for c in capsules], float).reshape(-1, 3)
        self.radii = np.array([c.r for c in capsules], float) + self.z_m
        self.table_mask = np.array([bool(c.table) for c in capsules])

    def world(self, q) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Capsule endpoints in the table frame (inflated by the arm's model uncertainty Z)."""
        frames = self.chain.frames(q)
        base = self.chain.T_table_base
        P0 = np.empty_like(self._p0)
        P1 = np.empty_like(self._p1)
        for k, link in enumerate(self._links):
            F = base if link < 0 else frames[link]
            P0[k] = F[:3, :3] @ self._p0[k] + F[:3, 3]
            P1[k] = F[:3, :3] @ self._p1[k] + F[:3, 3]
        return P0, P1, self.radii

    def points(self, q) -> np.ndarray:
        P0, P1, _ = self.world(q)
        return np.vstack([P0, P1])

    def max_point_displacement(self, qa, qb) -> float:
        """Largest displacement of any capsule endpoint between two joint vectors (K3 per-point clamp)."""
        return float(np.max(np.linalg.norm(self.points(qa) - self.points(qb), axis=1)))


def seg_seg(p1, q1, p2, q2, eps: float = 1e-12) -> np.ndarray:
    """Closest distance between segment sets (N,3)x(N,3), Ericson's ClosestPtSegmentSegment, vectorised."""
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = np.einsum("ij,ij->i", d1, d1)
    e = np.einsum("ij,ij->i", d2, d2)
    f = np.einsum("ij,ij->i", d2, r)
    c = np.einsum("ij,ij->i", d1, r)
    b = np.einsum("ij,ij->i", d1, d2)
    a_ok = a > eps
    e_ok = e > eps
    safe_a = np.where(a_ok, a, 1.0)
    safe_e = np.where(e_ok, e, 1.0)
    denom = a * e - b * b
    s = np.where(denom > eps, np.clip((b * f - c * e) / np.where(denom > eps, denom, 1.0), 0.0, 1.0), 0.0)
    t = (b * s + f) / safe_e
    # second segment degenerate: t = 0, s = clamp(-c / a)
    s = np.where(e_ok, s, np.clip(-c / safe_a, 0.0, 1.0))
    t = np.where(e_ok, t, 0.0)
    t_clamped = np.clip(t, 0.0, 1.0)
    s = np.where(e_ok & (t_clamped != t), np.clip((b * t_clamped - c) / safe_a, 0.0, 1.0), s)
    t = t_clamped
    # first segment degenerate: s = 0, t = clamp(f / e)
    t = np.where(a_ok, t, np.where(e_ok, np.clip(f / safe_e, 0.0, 1.0), 0.0))
    s = np.where(a_ok, s, 0.0)
    c1 = p1 + d1 * s[:, None]
    c2 = p2 + d2 * t[:, None]
    return np.linalg.norm(c1 - c2, axis=1)


@dataclass(frozen=True)
class Obstacle:
    name: str
    p0: np.ndarray
    p1: np.ndarray
    r: float
    min_gap_m: float


@dataclass(frozen=True)
class SceneGeometry:
    table_z_m: float
    table_gap_m: float
    obstacles: tuple[Obstacle, ...]
    keep_in_min: np.ndarray | None
    keep_in_max: np.ndarray | None


def box_as_capsule(name: str, pos, half, min_gap_m: float) -> Obstacle:
    """A box, conservatively replaced by a capsule containing it (along its longest axis)."""
    pos = np.asarray(pos, float)
    half = np.asarray(half, float)
    k = int(np.argmax(half))
    rest = [half[i] for i in range(3) if i != k]
    r = float(math.hypot(*rest))
    d = np.zeros(3)
    d[k] = half[k]
    return Obstacle(name, pos - d, pos + d, r, float(min_gap_m))


def clearances(moving: ArmGeometry, q_moving, others: list[tuple[ArmGeometry, object]], scene: SceneGeometry) -> dict:
    """Minimum surface distances (m) for one pose of the moving arm: inter-arm, table, obstacles, keep-in."""
    A0, A1, AR = moving.world(q_moving)
    out = {"inter_arm_m": math.inf, "table_m": math.inf, "obstacle_m": math.inf, "keep_in_m": math.inf,
           "obstacle_margin_m": math.inf}
    for geom, q in others:
        B0, B1, BR = geom.world(q)
        I, J = np.meshgrid(np.arange(len(AR)), np.arange(len(BR)), indexing="ij")
        I, J = I.ravel(), J.ravel()
        gaps = seg_seg(A0[I], A1[I], B0[J], B1[J]) - AR[I] - BR[J]
        out["inter_arm_m"] = min(out["inter_arm_m"], float(gaps.min()))
    if moving.table_mask.any():
        lowest = (np.minimum(A0[:, 2], A1[:, 2]) - AR)[moving.table_mask]
        out["table_m"] = float(lowest.min() - scene.table_z_m)
    for ob in scene.obstacles:
        n = len(AR)
        gaps = seg_seg(A0, A1, np.repeat(ob.p0[None], n, 0), np.repeat(ob.p1[None], n, 0)) - AR - ob.r
        g = float(gaps.min())
        out["obstacle_m"] = min(out["obstacle_m"], g)
        out["obstacle_margin_m"] = min(out["obstacle_margin_m"], g - ob.min_gap_m)
    if scene.keep_in_min is not None and scene.keep_in_max is not None and moving.table_mask.any():
        # static / base-height capsules (table-exempt) cannot leave the box by moving: only moving ones are checked
        m = moving.table_mask
        pts = np.vstack([A0[m], A1[m]])
        rad = np.concatenate([AR[m], AR[m]])
        lo = (pts - rad[:, None]) - scene.keep_in_min
        hi = scene.keep_in_max - (pts + rad[:, None])
        out["keep_in_m"] = float(min(lo.min(), hi.min()))
    return out
