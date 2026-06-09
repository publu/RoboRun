"""Integrity tests — seal, verify, tamper, detect."""
import json

import pytest

from roborun import integrity


@pytest.fixture
def run_dir(tmp_path):
    events = [
        {"id": f"evt_{i}", "type": "ros", "source": "test",
         "title": f"event {i}", "detail": {"n": i}, "ts": 1749480000 + i * 0.1}
        for i in range(100)
    ]
    d = tmp_path / "run_test"
    integrity.snapshot_run(events, d)
    return d


def test_seal_and_verify(run_dir):
    sealed = integrity.seal_run(run_dir)
    assert sealed["ok"] and sealed["event_count"] == 100
    result = integrity.verify_run(run_dir)
    assert result["verified"]
    assert result["merkle_root"] == sealed["merkle_root"]


def test_tamper_detected_at_exact_event(run_dir):
    integrity.seal_run(run_dir)
    integrity.tamper_run(run_dir, 42)
    result = integrity.verify_run(run_dir)
    assert not result["verified"]
    assert result["failed_event"] == 42
    assert result["expected"] != result["found"]


def test_deleted_event_detected(run_dir):
    integrity.seal_run(run_dir)
    lines = (run_dir / "run.jsonl").read_text().splitlines()
    (run_dir / "run.jsonl").write_text("\n".join(lines[:-1]) + "\n")
    result = integrity.verify_run(run_dir)
    assert not result["verified"]
    assert "count mismatch" in result["reason"]


def test_reordered_events_detected(run_dir):
    integrity.seal_run(run_dir)
    lines = (run_dir / "run.jsonl").read_text().splitlines()
    lines[10], lines[11] = lines[11], lines[10]
    (run_dir / "run.jsonl").write_text("\n".join(lines) + "\n")
    result = integrity.verify_run(run_dir)
    assert not result["verified"]


def test_unsealed_run_fails(run_dir):
    result = integrity.verify_run(run_dir)
    assert not result["verified"]
    assert "not sealed" in result["reason"]


def test_merkle_root_deterministic():
    hashes = [integrity.hash_event({"a": i}) for i in range(7)]
    assert integrity.merkle_root(hashes) == integrity.merkle_root(list(hashes))
    assert integrity.merkle_root(hashes) != integrity.merkle_root(hashes[::-1])


def test_canonical_key_order_invariant():
    assert integrity.hash_event({"a": 1, "b": 2}) == integrity.hash_event({"b": 2, "a": 1})


def test_whitespace_in_jsonl_does_not_matter(run_dir):
    """Hashing operates on parsed events, not raw lines."""
    integrity.seal_run(run_dir)
    lines = (run_dir / "run.jsonl").read_text().splitlines()
    evt = json.loads(lines[0])
    lines[0] = json.dumps(evt, indent=None, separators=(", ", ": "))
    (run_dir / "run.jsonl").write_text("\n".join(lines) + "\n")
    assert integrity.verify_run(run_dir)["verified"]


# ── Hash chain ────────────────────────────────────────────────────────────────

def _chained_events(n=20):
    prev = integrity.GENESIS
    out = []
    for i in range(n):
        evt = {"id": f"evt_{i}", "type": "ros", "source": "test",
               "title": f"event {i}", "detail": {}, "ts": 1749480000 + i * 0.1,
               "prev": prev}
        prev = integrity.hash_event(evt)
        out.append(evt)
    return out


@pytest.fixture
def chained_dir(tmp_path):
    d = tmp_path / "run_chained"
    integrity.snapshot_run(_chained_events(), d)
    return d


def test_chain_intact(chained_dir):
    lines = (chained_dir / "run.jsonl").read_text().splitlines()
    chain = integrity.verify_chain(lines)
    assert chain["chain_checked"] and chain["chain_intact"]


def test_unsealed_chained_run_reports_chain(chained_dir):
    result = integrity.verify_run(chained_dir)
    assert not result["verified"]
    assert result["chain_intact"]
    assert "chain intact" in result["reason"]


def test_chain_break_detected(chained_dir):
    lines = (chained_dir / "run.jsonl").read_text().splitlines()
    evt = json.loads(lines[7])
    evt["title"] = "edited"
    lines[7] = json.dumps(evt)
    (chained_dir / "run.jsonl").write_text("\n".join(lines) + "\n")
    chain = integrity.verify_chain(lines)
    assert chain["chain_checked"] and not chain["chain_intact"]
    assert chain["chain_break"] == 8  # the NEXT event's prev no longer matches


def test_sealed_chained_run_verifies_with_chain(chained_dir):
    integrity.seal_run(chained_dir)
    result = integrity.verify_run(chained_dir)
    assert result["verified"] and result["chain_intact"]


def test_tamper_breaks_seal_and_chain(chained_dir):
    integrity.seal_run(chained_dir)
    integrity.tamper_run(chained_dir, 5)
    result = integrity.verify_run(chained_dir)
    assert not result["verified"]
    assert result["failed_event"] == 5


def test_legacy_unchained_run_skips_chain(run_dir):
    integrity.seal_run(run_dir)
    result = integrity.verify_run(run_dir)
    assert result["verified"]
    assert not result.get("chain_checked")


# ── Event bus journaling ──────────────────────────────────────────────────────

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
    chain = integrity.verify_chain(lines)
    assert chain["chain_intact"]
    sealed = integrity.seal_run(run_dir)
    assert integrity.verify_run(run_dir)["verified"]
    bus.record_sealed(run_dir.name, sealed["merkle_root"])
    bus.emit("system", "test", "next run starts")
    next_dir = bus.close_journal()
    manifest = json.loads((next_dir / "manifest.json").read_text())
    assert manifest["prev_run"]["merkle_root"] == sealed["merkle_root"]
