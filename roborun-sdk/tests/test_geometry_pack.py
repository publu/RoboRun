import copy

import numpy as np
import pytest

from roborun_sdk import dev
from roborun_sdk.rt.datatypes import OperatorConfirm, Refused
from roborun_sdk.rt.geometry import seg_seg
from roborun_sdk.rt.pack import PackStore, compare, load_pack, parse_pack


def _brute(p1, q1, p2, q2, n=400):
    s = np.linspace(0, 1, n)
    a = p1 + (q1 - p1) * s[:, None]
    b = p2 + (q2 - p2) * s[:, None]
    return np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2))


def test_seg_seg_matches_brute_force():
    rng = np.random.default_rng(3)
    P1, Q1, P2, Q2 = (rng.uniform(-1, 1, (60, 3)) for _ in range(4))
    Q2[:5] = P2[:5]                          # degenerate second segments
    Q1[5:10] = P1[5:10]                      # degenerate first segments
    got = seg_seg(P1, Q1, P2, Q2)
    want = np.array([_brute(*v) for v in zip(P1, Q1, P2, Q2)])
    assert np.all(got <= want + 1e-9)
    assert np.allclose(got, want, atol=6e-3)


def test_bundled_pack_is_signed_and_has_both_arms(pack):
    raw = dev.sim_pack_raw()
    load_pack(raw, dev.dev_root_keys())
    with pytest.raises(Refused):
        load_pack(raw, {"someone-else": dev.DEV_SEGMENTS.public})
    assert set(pack.arms) == {"left", "right"}
    right = pack.arms["right"]
    assert right.geometry.capsules[4].r >= 0.10                 # the Metal wrist/gripper carry the 70 mm envelope
    home = pack.arms["left"].home_q
    P0, P1, R = pack.arms["left"].geometry.world(home)
    assert np.isfinite(P0).all() and len(R) == 6
    # base and the first link (joint 1 vertical) cannot change height: exempt from the table check
    assert [c.table for c in pack.arms["left"].geometry.capsules][:2] == [False, True]


def test_every_arm_at_its_own_home_is_inside_the_keep_in_box(pack):
    """The design example's box (y >= -0.30) excluded the Metal's own Home (its link2 capsule
    reaches y = -0.323), so every Metal segment refused keep_in. The box now holds each arm's Home capsules + 50 mm."""
    from roborun_sdk.rt.geometry import clearances
    for name, arm in pack.arms.items():
        others = [(o.geometry, o.home_q) for n, o in pack.arms.items() if n != name]
        c = clearances(arm.geometry, arm.home_q, others, pack.scene)
        assert c["keep_in_m"] >= 0.05 - 1e-9, (name, c)
        assert c["inter_arm_m"] > 0.02, (name, c)
    assert pack.scene.keep_in_min[1] < -0.30          # widened past the design box's y >= -0.30 for the Metal's Home


def _variant(mutate):
    raw = copy.deepcopy(dev.sim_pack_raw())
    raw.pop("sig")
    mutate(raw)
    return parse_pack(dev.sign_pack(raw), signed=True)


def test_compare_tighter_and_loosening(pack):
    slower = _variant(lambda r: r["arms"]["left"]["limits"].__setitem__("vel_rad_s", 0.3))
    assert compare(pack, slower).tighter
    faster = _variant(lambda r: r["arms"]["left"]["limits"].__setitem__("vel_rad_s", 0.9))
    d = compare(pack, faster)
    assert not d.tighter and "arms.left.vel_rad_s" in d.loosened
    wider = _variant(lambda r: r["arms"]["left"]["chain"][0].__setitem__("upper", 3.2))
    assert "arms.left.upper_rad" in compare(pack, wider).loosened
    thinner = _variant(lambda r: r["arms"]["right"]["capsules"][3].__setitem__("r", 0.01))
    assert any(x.startswith("arms.right.capsule_not_contained") for x in compare(pack, thinner).loosened)
    fatter = _variant(lambda r: r["arms"]["right"]["capsules"][3].__setitem__("r", 0.08))
    assert compare(pack, fatter).tighter
    no_pole = _variant(lambda r: r["scene"].__setitem__("obstacles", r["scene"]["obstacles"][1:]))
    assert not compare(pack, no_pole).tighter
    less_age = _variant(lambda r: r["proximity"].__setitem__("max_sample_age_s", 0.1))
    assert compare(pack, less_age).tighter


def test_store_activates_tighter_at_rest_and_stages_loosening(pack):
    store = PackStore(pack)
    slower = _variant(lambda r: r["arms"]["left"]["limits"].__setitem__("vel_rad_s", 0.3))
    assert store.offer(slower, at_rest=False, now=1.0)["result"] == "staged"       # tighter, but not at rest
    assert store.maybe_activate_staged(at_rest=True, now=2.0)["result"] == "activated"
    faster = _variant(lambda r: r["arms"]["left"]["limits"].__setitem__("vel_rad_s", 0.9))
    rec = store.offer(faster, at_rest=True, now=3.0)
    assert rec["result"] == "staged" and rec["why"] == "loosening_needs_local_confirmation"
    assert store.maybe_activate_staged(at_rest=True, now=4.0) is None              # never automatic
    with pytest.raises(Refused):
        store.confirm(faster.digest, OperatorConfirm("op", "approve", 5.0), at_rest=False, now=5.0)
    assert store.confirm(faster.digest, OperatorConfirm("op", "approve", 5.0), at_rest=True, now=5.0)["result"] == "activated"
    assert store.active.digest == faster.digest
