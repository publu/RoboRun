"""Integrity primitives: canonical hashing, Merkle root, signing, journal chain.

The black-box product surface (seal/verify/tamper of recordings) lives in
recorder.py and is covered by test_recorder.py; this file covers the
primitives those are built on, plus the event bus's live journal chain.
"""
import json

from roborun import integrity


def test_merkle_root_deterministic():
    hashes = [integrity.hash_event({"a": i}) for i in range(7)]
    assert integrity.merkle_root(hashes) == integrity.merkle_root(list(hashes))
    assert integrity.merkle_root(hashes) != integrity.merkle_root(hashes[::-1])


def test_merkle_root_empty_and_single():
    assert integrity.merkle_root([])  # defined, stable
    one = integrity.hash_event({"x": 1})
    assert integrity.merkle_root([one]) == one


def test_canonical_key_order_invariant():
    assert integrity.hash_event({"a": 1, "b": 2}) == integrity.hash_event({"b": 2, "a": 1})


def test_canonical_whitespace_invariant():
    """Hashing operates on parsed events, not raw serialization."""
    evt = {"id": "e1", "detail": {"n": 3}}
    reserialized = json.loads(json.dumps(evt, indent=2))
    assert integrity.hash_event(evt) == integrity.hash_event(reserialized)


def test_sign_and_verify_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setattr(integrity, "_KEY_DIR", tmp_path)
    monkeypatch.setattr(integrity, "_KEY_PATH", tmp_path / "ed25519.key")
    sig = integrity.sign_message(b"hello")
    if sig is None:  # cryptography not installed
        assert integrity.verify_message(None, b"hello") is None
        return
    assert integrity.verify_message(sig, b"hello") is True
    assert integrity.verify_message(sig, b"hellp") is False
    assert integrity.public_key_hex() == sig["public_key"]


# ── Event bus journaling (the live chain the recorder ingests) ───────────

def _verify_chain(lines: list[str]) -> bool:
    prev = integrity.GENESIS
    for ln in lines:
        evt = json.loads(ln)
        if evt.get("prev") != prev:
            return False
        prev = integrity.hash_event(evt)
    return True


def test_emit_journals_chained_events(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    from roborun import events as bus
    bus.close_journal()  # detach any journal owned by other tests
    for i in range(10):
        bus.emit("system", "test", f"hello {i}")
    live = bus.current_run()
    assert live is not None
    run_dir = bus.close_journal()
    assert run_dir is not None and run_dir.parent == tmp_path / "runs"
    lines = (run_dir / "run.jsonl").read_text().splitlines()
    assert len(lines) == 10
    assert _verify_chain(lines)

    # tampering any event breaks the chain at the next link
    evt = json.loads(lines[4])
    evt["title"] = "edited"
    lines[4] = json.dumps(evt)
    assert not _verify_chain(lines)
