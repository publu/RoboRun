import copy
import json

import pytest

from roborun_sdk import dev
from roborun_sdk.rt import _ed25519, signing
from roborun_sdk.rt.canonical import canonical_bytes, digest, from_jsonable, to_jsonable


def test_rfc8032_vectors_pure_python():
    seed = bytes.fromhex("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60")
    assert _ed25519.public_key(seed).hex() == "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a"
    sig = _ed25519.sign(seed, b"")
    assert sig.hex() == ("e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e065224901555fb8821590a33bacc61e39701cf9b46b"
                         "d25bf5f0595bbe24655141438e7a100b")
    assert _ed25519.verify(_ed25519.public_key(seed), b"", sig)
    seed2 = bytes.fromhex("4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb")
    sig2 = _ed25519.sign(seed2, b"\x72")
    assert sig2.hex().startswith("92a009a9f0d4cab8720e820b5f642540")
    assert not _ed25519.verify(_ed25519.public_key(seed2), b"\x73", sig2)


def test_sign_verify_tamper_and_unknown_kid(pack):
    seg = dev.make_segment([[0.0] * 7, [0.001] * 7], semantics=pack.arm("left").semantics, pack_digest=pack.digest)
    keys = {dev.DEV_SEGMENTS.kid: dev.DEV_SEGMENTS.public}
    assert signing.verify(seg, "segment", keys) == (True, "ok")
    bad = copy.deepcopy(seg)
    bad["limits"]["ee_speed_m_s"] = 0.5
    assert signing.verify(bad, "segment", keys) == (False, "bad_signature")
    assert signing.verify(seg, "segment", {"other": dev.DEV_SEGMENTS.public})[1] == "unknown_kid"
    assert signing.verify(seg, "control", keys) == (False, "wrong_kind")   # a segment is not a control
    as_control = dict(seg, type="control")
    assert signing.verify(as_control, "control", keys)[0] is False   # a segment's signature never fits a control


def test_payload_is_manifest_wire_signing_payload(pack):
    """One convention (manifest_wire.base): the bytes of the message as received, never a model re-dump."""
    from roborun_sdk.rt import wire
    if not wire.available():
        pytest.skip("wire schema package not importable")
    w = wire.module()
    seg = dev.make_segment([[0.0] * 7, [0.001] * 7], semantics=pack.arm("left").semantics, pack_digest=pack.digest)
    assert signing.payload(seg, "segment") == w.signing_payload(seg)
    # a sender holding a model signs what it sends: render(model)
    model = w.parse_message(seg)
    assert w.signing_payload(model) == w.signing_payload(w.render(model))
    raw = dev.sim_pack_raw()
    assert signing.payload(raw, "safety-pack") == w.signing_payload(raw)
    from manifest_wire.pack import SafetyPack
    assert pack.digest == SafetyPack.model_validate(raw).digest_ref()
    # the dev segment spells 10 for a float field (expires_after_s): a model re-dump would sign 10.0 instead
    assert b'"expires_after_s":10,' in signing.payload(seg, "segment")
    assert b'"expires_after_s":10.0' in w.parse_message(seg).canonical()


def test_json_renderings_verify_alike(pack):
    seg = dev.make_segment([[0.0] * 7, [0.001] * 7], semantics=pack.arm("left").semantics, pack_digest=pack.digest)
    keys = {dev.DEV_SEGMENTS.kid: dev.DEV_SEGMENTS.public}
    round_trip = from_jsonable(json.loads(json.dumps(to_jsonable(seg))))          # this SDK's {"$b64"} files
    assert canonical_bytes(round_trip) == canonical_bytes(seg) and digest(round_trip) == digest(seg)
    assert signing.verify(round_trip, "segment", keys)[0]
    from roborun_sdk.rt.canonical import wire_jsonable
    wire_json = json.loads(json.dumps(wire_jsonable(seg)))                       # the wire's base64 strings
    assert signing.verify(wire_json, "segment", keys)[0]


def test_canonical_refuses_non_finite():
    with pytest.raises(ValueError):
        canonical_bytes({"x": float("nan")})
