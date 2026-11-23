"""Async anchoring (PERF #2): seal returns immediately; anchoring is deferred.

The Ed25519 signature is computed synchronously and is the integrity anchor;
only the RFC 3161 timestamp is deferred. So the run is always verifiable, and
sealing never blocks on TSA HTTP.
"""
from __future__ import annotations

import time

import pytest

from roborun import recorder as R
from roborun.recorder import RunRecorder, verify_mcap_run


def _record(tmp_path):
    rec = RunRecorder(robot_id="t", root=tmp_path, checkpoint_interval=0.01)
    rec.write_pose(1.0, 2.0, 0.5, ts=time.time())
    return rec


def test_close_seals_immediately_signed(tmp_path):
    rec = _record(tmp_path)
    seal = rec.close(do_anchor=True, anchor_async=True)
    # Returned seal is fully signed, with anchoring pending (not blocking).
    assert seal["signature"] is not None
    assert seal["merkle_root"]
    assert seal["anchor"]["status"] == "unanchored"
    # The run already verifies as internally consistent — integrity is anchored.
    v = verify_mcap_run(rec.mcap_path)
    assert v["state"] in ("consistent_unanchored", "verified_anchored")
    assert v["chain_intact"]


def test_close_is_fast(tmp_path, monkeypatch):
    # Simulate a slow TSA: a synchronous stamp would block on this; async must not.
    def slow_stamp(_digest, **kw):
        time.sleep(2.0)
        return None
    monkeypatch.setattr(R.anchor, "stamp_digest", slow_stamp)
    rec = _record(tmp_path)
    t0 = time.time()
    rec.close(do_anchor=True, anchor_async=True)
    assert time.time() - t0 < 1.0  # did not wait on the 2s stamp


def test_sync_path_still_works(tmp_path, monkeypatch):
    # anchor_async=False folds the (here: offline/None) result into the seal.
    monkeypatch.setattr(R.anchor, "stamp_digest", lambda d, **k: None)
    rec = _record(tmp_path)
    seal = rec.close(do_anchor=True, anchor_async=False)
    assert seal["anchor"]["status"] == "unanchored"
    assert "reason" in seal["anchor"]


def test_anchor_thread_folds_proof(tmp_path, monkeypatch):
    # A successful stamp updates the on-disk seal to anchored via the bg path.
    monkeypatch.setattr(R.anchor, "stamp_digest", lambda d, **k: b"\x30\x03fake-tsr")
    monkeypatch.setattr(R.anchor, "status",
                        lambda p, expected_digest=None: {"status": "verified_anchored"})
    rec = _record(tmp_path)
    info = rec._anchor_into_seal(rec.close(do_anchor=False)["merkle_root"])
    assert info["status"] == "verified_anchored"
    import json
    assert json.loads(rec.seal_path.read_text())["anchor"]["status"] == "verified_anchored"
