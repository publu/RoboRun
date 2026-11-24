"""The YAM arm's tool pose as DreamLabs' Dream-1 reads it: `state.<side>_eef_xyz` and `state.<side>_eef_rot6`.

dream-1-beta reads, besides each arm's joints and gripper, the tool point of each arm in that arm's own base frame:
`eef_xyz` in metres and `eef_rot6`, the first two columns of the tool's rotation matrix (column 0, then column 1). The
frame must be the one DreamLabs' own IK uses, or the model sees poses in another frame. So the chain below is the joint
origins and axes of the `assets/yam.urdf` of DreamLabs' IK module, copied exactly as written there, and the tool point is
that module's `TCP_OFFSET_M` (0.1347 m) along link_6's z axis, with link_6's axes.

Source: the `dreamlabs` 1.0.42 wheel on PyPI, Copyright 2026 Dream Labs, Apache License 2.0 (policy/data/
dreamlabs.LICENSE, and THIRD_PARTY_NOTICES.md); DreamLabs' NOTICE says its YAM description is derived from i2rt's (MIT
License, Copyright (c) I2RT Robotics). At the zero pose it gives xyz (0.245, 0, 0.164) and rot6 (0, 1, 0, 0, 0, 1), the
values DreamLabs' client computes with its own IK (tests/test_dreamlabs_yam.py pins these and other poses).

Standard library only.
"""
from __future__ import annotations

import math

# yam.urdf, joint1..joint6 in chain order (base_link -> link_1 -> ... -> link_6): (origin xyz, origin rpy); every axis is z
CHAIN = (
    ((-3.55271e-18, -1.05495e-18, 0.0631), (0.0, -0.0, 1.5708)),
    ((2.5e-05, -0.02, 0.0409), (1.5708, -1.98004e-28, 1.5708)),
    ((0.264, 0.0, 6.77626e-21), (-3.14159, 1.26218e-29, 7.88861e-31)),
    ((-0.245, -0.0600003, -1.9984e-15), (-5.65471e-31, -1.1507e-13, 2.94252e-43)),
    ((-0.0739968, -0.0395003, 2.44738e-05), (-1.5708, 1.5708, 0.0)),
    ((-4.0742e-16, 0.0353, 0.0395), (-1.5708, 3.81639e-16, 4.68582e-15)),
)
TCP_OFFSET_M = 0.1347          # DreamLabs' TCP_OFFSET_M: the tool point along link_6 z


def _rpy(r, p, y):
    """URDF rpy: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rz(q):
    c, s = math.cos(q), math.sin(q)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


_FIXED = tuple((xyz, _rpy(*rpy)) for xyz, rpy in CHAIN)


def tool_pose(q):
    """(xyz, R) of the tool point in the arm's base frame for the six joint angles q (radians, DreamLabs' YAM convention,
    which is Roborun's: policy/data/dreamlabs_conventions.json)."""
    q = [float(v) for v in q]
    if len(q) != len(CHAIN):
        raise ValueError(f'a YAM arm has {len(CHAIN)} joints; got {len(q)}')
    R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    p = [0.0, 0.0, 0.0]
    for (xyz, fixed), angle in zip(_FIXED, q):
        p = [p[i] + sum(R[i][k] * xyz[k] for k in range(3)) for i in range(3)]
        R = _mul(_mul(R, fixed), _rz(angle))
    p = [p[i] + R[i][2] * TCP_OFFSET_M for i in range(3)]
    return p, R


def eef(q):
    """(eef_xyz [3], eef_rot6 [6]) as Dream-1 reads them: the tool point, then R's column 0 and column 1."""
    p, R = tool_pose(q)
    return p, [R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1]]
