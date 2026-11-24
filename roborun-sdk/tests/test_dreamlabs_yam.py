"""The YAM tool pose Roborun sends to dream-1-beta is the one DreamLabs' own `dream_ik` computes.

The expected values were computed with `dream_ik.solver._tool_fk` on `build_arm('yam', 'right', 'tcp')` and
`dream_ik.decode.mat_to_rot6d`, from DreamLabs' `dreamlabs` 1.0.42 wheel (JAX in float32, so they agree to about 1e-7).
The zero pose is also what DreamLabs' client sent for both arms with `ik: {arm: 'yam', head: 'joint'}`: xyz
(0.245, 0, 0.164), rot6 (0, 1, 0, 0, 0, 1). Standard library only: no DreamLabs package is needed to run this.
"""
import math

import pytest

from roborun_sdk.policy import dreamlabs_yam

# (joint1..joint6 rad, eef_xyz m, eef_rot6) from dream_ik
DREAM_IK = [
    ([0, 0, 0, 0, 0, 0],
     [0.2449969, 2e-06, 0.1640017],
     [-3.7e-06, 0.9999999, -6.3e-06, -7.3e-06, 6.4e-06, 0.9999999]),
    ([0.0, 0.5, 0.5, -0.35, 0.0, 0.0],
     [0.2625227, 2.7e-06, 0.2069039],
     [-5.1e-06, 1.0, -7.3e-06, 0.342891, 8.6e-06, 0.9393752]),
    ([0.3, -0.7, 1.2, 0.4, -0.9, 1.1],
     [-0.4574221, -0.0021074, 0.2803528],
     [-0.4920523, 0.1429234, -0.8587536, -0.6037793, -0.7666616, 0.2183595]),
    ([1.6, 0.2, 0.9, -0.5, 0.6, -1.3],
     [0.0924408, 0.1227795, 0.4027502],
     [-0.2305887, 0.3328739, -0.9143433, -0.8089422, 0.4566456, 0.370253]),
    ([0.0, 0.5, 0.8, 0.0, 0.3, 0.0],
     [0.2304897, -0.0502346, 0.430155],
     [0.2823159, 0.9553385, 0.0873285, -0.295527, 4.1e-06, 0.9553345]),
]


@pytest.mark.parametrize('q,xyz,rot6', DREAM_IK)
def test_the_tool_pose_matches_dream_ik(q, xyz, rot6):
    got_xyz, got_rot6 = dreamlabs_yam.eef(q)
    assert got_xyz == pytest.approx(xyz, abs=1e-6)
    assert got_rot6 == pytest.approx(rot6, abs=1e-6)


def test_the_zero_pose_is_what_dreamlabs_client_sends():
    xyz, rot6 = dreamlabs_yam.eef([0.0] * 6)
    assert xyz == pytest.approx([0.245, 0.0, 0.164], abs=1e-5)
    assert rot6 == pytest.approx([0, 1, 0, 0, 0, 1], abs=1e-5)


def test_rot6_is_the_first_two_columns_of_a_rotation():
    p, R = dreamlabs_yam.tool_pose([0.3, -0.7, 1.2, 0.4, -0.9, 1.1])
    _, rot6 = dreamlabs_yam.eef([0.3, -0.7, 1.2, 0.4, -0.9, 1.1])
    assert rot6 == [R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1]]
    c0, c1 = rot6[:3], rot6[3:]
    assert math.isclose(sum(v * v for v in c0), 1.0, abs_tol=1e-9) and math.isclose(sum(v * v for v in c1), 1.0, abs_tol=1e-9)
    assert abs(sum(a * b for a, b in zip(c0, c1))) < 1e-9


def test_a_wrong_joint_count_is_refused():
    with pytest.raises(ValueError, match='6 joints'):
        dreamlabs_yam.eef([0.0] * 7)
