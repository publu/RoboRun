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
    assert "never sealed" in result["reason"]


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
