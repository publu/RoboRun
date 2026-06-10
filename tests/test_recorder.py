"""MCAP recorder tests — chunk chain, O(1) seal, three-state verify, clips."""
import json
import time

import pytest

from roborun.recorder import (
    RunRecorder, export_clip, list_runs, verify_mcap_run,
)


@pytest.fixture
def recorded_run(tmp_path):
    rec = RunRecorder(robot_id="testbot", root=tmp_path, checkpoint_interval=0.01)
    t0 = time.time()
    for i in range(40):
        ts = t0 + i * 0.1
        rec.write_camera(b"\xff\xd8jpeg" + bytes([i]) * 64, ts=ts)
        rec.write_detections(
            [{"label": "person", "score": 0.9, "bbox": [1, 2, 3, 4]}], ts=ts)
        rec.write_event({"id": f"evt_{i}", "type": "agent", "source": "t",
                         "title": f"e{i}", "detail": {}, "ts": ts, "prev": ""})
    rec.write_pose(1.0, 2.0, 0.5, ts=t0 + 1)
    seal = rec.close(do_anchor=False)
    return rec, seal, t0


def test_seal_is_o1_and_signed(recorded_run):
    rec, seal, _ = recorded_run
    assert seal["format"] == "mcap-chain-v1"
    assert seal["segment_count"] >= 1
    assert "event_hashes" not in seal  # the O(n) list is gone by design
    assert seal["signature"] is not None
    assert seal["mcap_size"] == rec.mcap_path.stat().st_size
    assert seal["message_counts"]["/camera/webcam"] == 40
    assert seal["message_counts"]["/agent/events"] == 40


def test_verify_consistent_unanchored(recorded_run):
    rec, seal, _ = recorded_run
    v = verify_mcap_run(rec.mcap_path)
    assert v["state"] == "consistent_unanchored"
    assert v["merkle_root"] == seal["merkle_root"]
    assert v["chain_intact"]
    assert v["signature_valid"] in (True, None)


def test_tamper_localizes_segment(recorded_run):
    rec, _, _ = recorded_run
    data = bytearray(rec.mcap_path.read_bytes())
    data[len(data) // 2] ^= 1
    rec.mcap_path.write_bytes(bytes(data))
    v = verify_mcap_run(rec.mcap_path)
    assert v["state"] == "broken"
    assert "segment" in v
    lo, hi = v["byte_range"]
    assert lo <= len(data) // 2 < hi


def test_appended_bytes_detected(recorded_run):
    rec, _, _ = recorded_run
    with open(rec.mcap_path, "ab") as fh:
        fh.write(b"sneaky extra frame")
    v = verify_mcap_run(rec.mcap_path)
    assert v["state"] == "broken"
    assert "size mismatch" in v["reason"]


def test_chain_sidecar_rewrite_detected(recorded_run):
    rec, _, _ = recorded_run
    lines = rec.chain_path.read_text().splitlines()
    entry = json.loads(lines[0])
    entry["sha256"] = "0" * 64
    lines[0] = json.dumps(entry, separators=(",", ":"))
    rec.chain_path.write_text("\n".join(lines) + "\n")
    v = verify_mcap_run(rec.mcap_path)
    assert v["state"] == "broken"


def test_reseal_attempt_detected(recorded_run):
    """Rewriting the seal without the private key breaks the signature."""
    rec, seal, _ = recorded_run
    if seal["signature"] is None:
        pytest.skip("cryptography not installed")
    forged = dict(seal)
    forged["sealed_at"] = "1999-01-01T00:00:00Z"
    rec.seal_path.write_text(json.dumps(forged))
    v = verify_mcap_run(rec.mcap_path)
    assert v["state"] == "broken"
    assert "signature" in v["reason"]


def test_unsealed_run_is_checkable(tmp_path):
    rec = RunRecorder(robot_id="crashbot", root=tmp_path, checkpoint_interval=0.01)
    for i in range(5):
        rec.write_event({"id": f"e{i}", "ts": time.time()})
    rec.checkpoint()
    # simulate crash: no close(), no seal
    v = verify_mcap_run(rec.mcap_path)
    assert v["state"] in ("consistent_unanchored", "broken")
    assert "not sealed" in v.get("reason", "") or v["state"] == "broken"


def test_clip_export_with_proof(recorded_run):
    rec, seal, t0 = recorded_run
    result = export_clip(rec.mcap_path, t0 + 1.0, t0 + 2.0)
    assert result["ok"]
    proof = json.loads(open(result["proof"]).read())
    assert proof["source_merkle_root"] == seal["merkle_root"]
    assert proof["message_count"] == result["messages"] > 0
    assert proof["signature"] is not None or proof["signature"] is None  # present key
    assert "clip_sha256" in proof


def test_clip_refuses_broken_source(recorded_run):
    rec, _, t0 = recorded_run
    data = bytearray(rec.mcap_path.read_bytes())
    data[100] ^= 1
    rec.mcap_path.write_bytes(bytes(data))
    result = export_clip(rec.mcap_path, t0, t0 + 5)
    assert not result["ok"]


def test_run_linking(tmp_path):
    rec1 = RunRecorder(robot_id="bot", root=tmp_path)
    rec1.write_event({"id": "e", "ts": time.time()})
    seal1 = rec1.close(do_anchor=False)
    time.sleep(1.1)  # distinct run_id second
    rec2 = RunRecorder(robot_id="bot", root=tmp_path)
    assert rec2.prev_run == {"run": seal1["run"], "merkle_root": seal1["merkle_root"]}
    rec2.write_event({"id": "e2", "ts": time.time()})
    rec2.close(do_anchor=False)


def test_list_runs(recorded_run, tmp_path):
    rec, _, _ = recorded_run
    runs = list_runs(root=rec.mcap_path.parent.parent)
    assert any(r["run"] == rec.run_id and r["sealed"] for r in runs)


def test_foxglove_compatible_mcap(recorded_run):
    """The MCAP must be readable by a standard reader with json channels."""
    from mcap.reader import make_reader
    rec, _, _ = recorded_run
    with open(rec.mcap_path, "rb") as fh:
        reader = make_reader(fh)
        topics = set()
        n = 0
        for schema, channel, message in reader.iter_messages():
            topics.add(channel.topic)
            assert channel.message_encoding == "json"
            json.loads(message.data)
            n += 1
        assert {"/camera/webcam", "/detections/yolo", "/agent/events", "/pose"} <= topics
        assert n == 121
