"""MCAP run recorder — one file per run, tamper-evident at chunk granularity.

Every run is a single MCAP file holding all channels (camera frames,
detections, CLIP embeddings, agent events, pose), replacing the old three-way
split between run.jsonl, spatial_memory.db, and datasets/. Messages are JSON
encoded against Foxglove-compatible schemas, so runs replay natively in
Foxglove Studio.

Integrity (the reseal fix):
  * As the MCAP writer flushes, the bytes that land on disk are hashed into
    segments and chained: chain[i] = sha256(bytes_i), linked to chain[i-1] in a
    sidecar `<run>.chain.jsonl` written live (tamper-evident while recording,
    crash-leaves-a-checkable-prefix). Segments track MCAP's own chunk flushes.
  * Sealing computes a Merkle root over segment hashes — the seal is O(1):
    roots, counts, timestamps, signature, anchor status. No per-event hash list.
  * The root (already 32 bytes) is anchored to an RFC 3161 trusted
    timestamp authority (anchor.py) — synchronous, done at seal time.

Verification is three-state, not binary:
    verified_anchored        — unchanged since an external clock witnessed it
    consistent_unanchored    — chain + Merkle root intact, no external anchor
    broken                   — with the failing segment and byte range

Layout (local mirror of the R2 layout in the architecture spec):
    ~/.roborun/runs/<robot_id>/<run_id>.mcap
    ~/.roborun/runs/<robot_id>/<run_id>.chain.jsonl
    ~/.roborun/runs/<robot_id>/<run_id>.seal
    ~/.roborun/runs/<robot_id>/<run_id>.seal.tsr

CLI:
    python -m roborun.recorder verify  <run.mcap>
    python -m roborun.recorder upgrade <run.mcap>     # anchor a run sealed offline
    python -m roborun.recorder clip    <run.mcap> <start_ts> <end_ts>
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
import queue
import threading
import time
from pathlib import Path
from typing import Any, IO

from roborun import anchor
from roborun.integrity import GENESIS, merkle_root, sign_message, verify_message

SEAL_FORMAT = "mcap-chain-v1"

# Foxglove renders these natively from JSON; roborun.* are our own.
SCHEMAS: dict[str, dict] = {
    "foxglove.CompressedImage": {
        "type": "object",
        "properties": {
            "timestamp": {"type": "object"},
            "frame_id": {"type": "string"},
            "data": {"type": "string", "contentEncoding": "base64"},
            "format": {"type": "string"},
        },
    },
    "foxglove.PoseInFrame": {
        "type": "object",
        "properties": {
            "timestamp": {"type": "object"},
            "frame_id": {"type": "string"},
            "pose": {"type": "object"},
        },
    },
    "roborun.Detections": {
        "type": "object",
        "properties": {
            "timestamp": {"type": "object"},
            "detections": {"type": "array"},
        },
    },
    "roborun.ClipEmbedding": {
        "type": "object",
        "properties": {
            "timestamp": {"type": "object"},
            "dim": {"type": "integer"},
            "vec": {"type": "string", "contentEncoding": "base64"},
            "frame_topic": {"type": "string"},
            "label": {"type": "string"},
        },
    },
    "roborun.AgentEvent": {
        "type": "object",
        "properties": {
            "id": {"type": "string"}, "type": {"type": "string"},
            "source": {"type": "string"}, "title": {"type": "string"},
            "detail": {"type": "object"}, "ts": {"type": "number"},
            "prev": {"type": "string"},
        },
    },
    "roborun.Json": {"type": "object"},
}


def runs_root() -> Path:
    base = os.environ.get("ROBORUN_STATE_DIR")
    root = Path(base) if base else Path.home() / ".roborun"
    return root / "runs"


def _ts_obj(ts: float) -> dict:
    sec = int(ts)
    return {"sec": sec, "nsec": int((ts - sec) * 1e9)}


class _ChainedStream:
    """File wrapper that hashes everything written, in checkpointable segments."""

    def __init__(self, fh: IO[bytes]):
        self._fh = fh
        self.offset = 0
        self._seg_start = 0
        self._seg_hash = hashlib.sha256()

    def write(self, data: bytes) -> int:
        self._fh.write(data)
        self._seg_hash.update(data)
        self.offset += len(data)
        return len(data)

    def tell(self) -> int:
        return self.offset

    def checkpoint(self) -> tuple[int, int, str] | None:
        """Close the current segment: (start_offset, length, sha256) or None if empty."""
        length = self.offset - self._seg_start
        if length == 0:
            return None
        digest = self._seg_hash.hexdigest()
        seg = (self._seg_start, length, digest)
        self._seg_start = self.offset
        self._seg_hash = hashlib.sha256()
        self._fh.flush()
        return seg


class RunRecorder:
    """Records one run into one MCAP file with a live hash chain sidecar."""

    def __init__(self, robot_id: str = "local", root: Path | None = None,
                 chunk_size: int = 256 * 1024,
                 checkpoint_interval: float = 5.0) -> None:
        from mcap.writer import Writer

        self.robot_id = robot_id
        self._root = (root or runs_root()) / robot_id
        self._root.mkdir(parents=True, exist_ok=True)

        run_id = time.strftime("run_%Y%m%d_%H%M%S", time.gmtime())
        n = 1
        while (self._root / f"{run_id}.mcap").exists():
            run_id = time.strftime("run_%Y%m%d_%H%M%S", time.gmtime()) + f"_{n}"
            n += 1
        self.run_id = run_id
        self.mcap_path = self._root / f"{run_id}.mcap"
        self.chain_path = self._root / f"{run_id}.chain.jsonl"
        self.seal_path = self._root / f"{run_id}.seal"

        self._lock = threading.RLock()
        self._fh = open(self.mcap_path, "wb")
        self._stream = _ChainedStream(self._fh)
        self._writer = Writer(self._stream, chunk_size=chunk_size)
        self._writer.start(profile="", library="roborun")

        self._chain_fh = open(self.chain_path, "a", buffering=1)
        self._prev_hash = GENESIS
        self._segments: list[str] = []
        self._checkpoint_interval = checkpoint_interval
        self._last_checkpoint = time.monotonic()

        self._schema_ids: dict[str, int] = {}
        self._channel_ids: dict[str, int] = {}
        self._message_counts: dict[str, int] = {}
        self.started_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self.prev_run = _latest_sealed_run(self._root)
        self._closed = False

        self._bus_queue: queue.Queue | None = None
        self._bus_thread: threading.Thread | None = None

        self._writer.add_metadata("roborun", {
            "run_id": run_id, "robot_id": robot_id,
            "started_at": self.started_at,
            "prev_run": json.dumps(self.prev_run) if self.prev_run else "",
        })

    # ── channels ─────────────────────────────────────────────────────────

    def _channel(self, topic: str, schema_name: str) -> int:
        cid = self._channel_ids.get(topic)
        if cid is not None:
            return cid
        sid = self._schema_ids.get(schema_name)
        if sid is None:
            sid = self._writer.register_schema(
                schema_name, "jsonschema",
                json.dumps(SCHEMAS.get(schema_name, SCHEMAS["roborun.Json"])).encode())
            self._schema_ids[schema_name] = sid
        cid = self._writer.register_channel(topic, "json", sid)
        self._channel_ids[topic] = cid
        return cid

    def write_json(self, topic: str, schema_name: str, obj: dict,
                   ts: float | None = None) -> None:
        """Generic write — the tap and all typed helpers funnel through here."""
        ts = ts if ts is not None else time.time()
        with self._lock:
            if self._closed:
                return
            cid = self._channel(topic, schema_name)
            data = json.dumps(obj, separators=(",", ":"), default=str).encode()
            t_ns = int(ts * 1e9)
            self._writer.add_message(cid, t_ns, data, t_ns)
            self._message_counts[topic] = self._message_counts.get(topic, 0) + 1
            if time.monotonic() - self._last_checkpoint >= self._checkpoint_interval:
                self._checkpoint_locked()

    def write_camera(self, jpeg: bytes, name: str = "webcam",
                     ts: float | None = None, frame_id: str = "camera") -> None:
        ts = ts if ts is not None else time.time()
        self.write_json(f"/camera/{name}", "foxglove.CompressedImage", {
            "timestamp": _ts_obj(ts), "frame_id": frame_id,
            "data": base64.b64encode(jpeg).decode(), "format": "jpeg",
        }, ts)

    def write_detections(self, detections: list[dict], name: str = "yolo",
                         ts: float | None = None) -> None:
        ts = ts if ts is not None else time.time()
        self.write_json(f"/detections/{name}", "roborun.Detections", {
            "timestamp": _ts_obj(ts), "detections": detections,
        }, ts)

    def write_clip(self, embedding, frame_topic: str = "/camera/webcam",
                   label: str | None = None, ts: float | None = None) -> None:
        ts = ts if ts is not None else time.time()
        import numpy as np
        vec = np.asarray(embedding, dtype=np.float32)
        self.write_json("/clip/embeddings", "roborun.ClipEmbedding", {
            "timestamp": _ts_obj(ts), "dim": int(vec.size),
            "vec": base64.b64encode(vec.tobytes()).decode(),
            "frame_topic": frame_topic, "label": label or "",
        }, ts)

    def write_pose(self, x: float, y: float, z: float = 0.0,
                   orientation: dict | None = None, frame_id: str = "map",
                   ts: float | None = None) -> None:
        ts = ts if ts is not None else time.time()
        self.write_json("/pose", "foxglove.PoseInFrame", {
            "timestamp": _ts_obj(ts), "frame_id": frame_id,
            "pose": {"position": {"x": x, "y": y, "z": z},
                     "orientation": orientation or {"x": 0, "y": 0, "z": 0, "w": 1}},
        }, ts)

    def write_event(self, event: dict) -> None:
        self.write_json("/agent/events", "roborun.AgentEvent", event,
                        event.get("ts"))

    # ── event bus tap ────────────────────────────────────────────────────

    def attach_event_bus(self) -> None:
        """Mirror the live event bus into /agent/events, off the hot path."""
        from roborun import events as bus
        if self._bus_thread is not None:
            return
        self._bus_queue = bus.subscribe()

        def drain() -> None:
            while not self._closed and self._bus_queue is not None:
                try:
                    evt = self._bus_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                try:
                    self.write_event(evt)
                except Exception:
                    pass

        self._bus_thread = threading.Thread(target=drain, daemon=True,
                                            name="RecorderEventTap")
        self._bus_thread.start()

    def _detach_event_bus(self) -> None:
        if self._bus_queue is not None:
            from roborun import events as bus
            bus.unsubscribe(self._bus_queue)
            self._bus_queue = None

    # ── integrity ────────────────────────────────────────────────────────

    def _checkpoint_locked(self) -> None:
        seg = self._stream.checkpoint()
        self._last_checkpoint = time.monotonic()
        if seg is None:
            return
        off, length, digest = seg
        entry = {"i": len(self._segments), "off": off, "len": length,
                 "sha256": digest, "prev": self._prev_hash}
        self._chain_fh.write(json.dumps(entry, separators=(",", ":")) + "\n")
        self._prev_hash = hashlib.sha256(
            f"{digest}|{self._prev_hash}".encode()).hexdigest()
        self._segments.append(digest)

    def checkpoint(self) -> None:
        with self._lock:
            self._checkpoint_locked()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "run": self.run_id, "robot_id": self.robot_id,
                "mcap": str(self.mcap_path), "bytes": self._stream.offset,
                "segments": len(self._segments),
                "messages": dict(self._message_counts),
                "recording": not self._closed,
            }

    def close(self, do_anchor: bool = True) -> dict[str, Any]:
        """Finish the MCAP, seal it (O(1) seal + Merkle root), anchor the root."""
        with self._lock:
            if self._closed:
                return json.loads(self.seal_path.read_text())
            self._detach_event_bus()
            self._writer.finish()
            self._checkpoint_locked()  # footer bytes
            self._closed = True
            self._fh.flush()
            self._fh.close()
            self._chain_fh.close()

            root = merkle_root(self._segments)
            sealed_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            seal: dict[str, Any] = {
                "version": 3,
                "format": SEAL_FORMAT,
                "run": self.run_id,
                "robot_id": self.robot_id,
                "started_at": self.started_at,
                "sealed_at": sealed_at,
                "mcap_size": self._stream.offset,
                "segment_count": len(self._segments),
                "message_counts": self._message_counts,
                "merkle_root": root,
                "prev_run": self.prev_run,
                "signature": sign_message(
                    f"{root}|{len(self._segments)}|{sealed_at}".encode()),
            }
            anchor_info: dict[str, Any] = {"status": "unanchored"}
            if do_anchor:
                tsr_bytes = anchor.stamp_digest(bytes.fromhex(root))
                if tsr_bytes is not None:
                    tsr_path = self.seal_path.with_suffix(".seal.tsr")
                    tsr_path.write_bytes(tsr_bytes)
                    anchor_info = {**anchor.status(
                        tsr_path, expected_digest=bytes.fromhex(root)),
                        "tsr": tsr_path.name}
                else:
                    anchor_info["reason"] = "offline or asn1crypto unavailable"
            seal["anchor"] = anchor_info
            self.seal_path.write_text(json.dumps(seal, indent=1))
            return seal


# ── verification ─────────────────────────────────────────────────────────

def _load_chain(chain_path: Path) -> list[dict]:
    if not chain_path.exists():
        return []
    return [json.loads(ln) for ln in chain_path.read_text().splitlines() if ln.strip()]


def verify_mcap_run(mcap_path: str | Path) -> dict[str, Any]:
    """Three-state verify of a recorded run.

    Recomputes every segment hash from the MCAP bytes, checks chain
    continuity, coverage (no gaps, no appended tail), the Merkle root
    against the seal, the Ed25519 signature, and the anchor status.
    """
    mcap_path = Path(mcap_path)
    base = mcap_path.with_suffix("")
    chain_path = Path(str(base) + ".chain.jsonl")
    seal_path = Path(str(base) + ".seal")
    tsr_path = Path(str(base) + ".seal.tsr")

    if not mcap_path.exists():
        return {"state": "broken", "reason": f"missing mcap: {mcap_path}"}
    entries = _load_chain(chain_path)
    if not entries:
        return {"state": "broken", "reason": "missing or empty chain sidecar"}

    data = mcap_path.read_bytes()
    prev = GENESIS
    pos = 0
    for e in entries:
        if e["off"] != pos:
            return {"state": "broken", "segment": e["i"],
                    "reason": f"segment {e['i']} coverage gap at byte {pos}"}
        chunk = data[e["off"]:e["off"] + e["len"]]
        if len(chunk) != e["len"]:
            return {"state": "broken", "segment": e["i"],
                    "reason": f"segment {e['i']} truncated: mcap shorter than chain"}
        found = hashlib.sha256(chunk).hexdigest()
        if found != e["sha256"]:
            return {"state": "broken", "segment": e["i"],
                    "byte_range": [e["off"], e["off"] + e["len"]],
                    "reason": f"segment {e['i']} hash mismatch "
                              f"(bytes {e['off']}–{e['off'] + e['len']})",
                    "expected": e["sha256"], "found": found}
        if e["prev"] != prev:
            return {"state": "broken", "segment": e["i"],
                    "reason": f"segment {e['i']} chain break"}
        prev = hashlib.sha256(f"{e['sha256']}|{prev}".encode()).hexdigest()
        pos += e["len"]

    root = merkle_root([e["sha256"] for e in entries])
    result: dict[str, Any] = {
        "mcap": str(mcap_path), "segments": len(entries),
        "merkle_root": root, "chain_intact": True,
    }

    if not seal_path.exists():
        if pos != len(data):
            return {**result, "state": "broken",
                    "reason": f"unsealed run has {len(data) - pos} unchained trailing bytes"}
        return {**result, "state": "consistent_unanchored",
                "reason": "not sealed (live or crashed run): chain intact"}

    seal = json.loads(seal_path.read_text())
    if pos != len(data) or seal.get("mcap_size") != len(data):
        return {**result, "state": "broken",
                "reason": f"size mismatch: sealed {seal.get('mcap_size')}, "
                          f"chained {pos}, on disk {len(data)} — bytes appended or removed"}
    if seal.get("merkle_root") != root:
        return {**result, "state": "broken",
                "reason": "merkle root mismatch: chain sidecar rewritten",
                "sealed_root": seal.get("merkle_root")}

    sig_msg = f"{root}|{seal['segment_count']}|{seal['sealed_at']}".encode()
    sig_ok = verify_message(seal.get("signature"), sig_msg)
    if sig_ok is False:
        return {**result, "state": "broken", "reason": "seal signature invalid: reseal attempt"}
    result["signature_valid"] = sig_ok
    result["sealed_at"] = seal["sealed_at"]
    result["message_counts"] = seal.get("message_counts", {})

    a = anchor.status(tsr_path, expected_digest=bytes.fromhex(root)) \
        if tsr_path.exists() else {"status": "unanchored", "reason": "no .tsr file"}
    result["anchor"] = a
    if a["status"] == "anchored":
        return {**result, "state": "verified_anchored",
                "reason": f"unchanged since trusted timestamp {a['tsa_time']} "
                          f"(RFC 3161)"}
    return {**result, "state": "consistent_unanchored",
            "reason": "chain and seal intact, but never externally timestamped"}


def list_runs(root: Path | None = None) -> list[dict]:
    """All MCAP runs across robots, newest first, with verify-lite status."""
    root = root or runs_root()
    out = []
    if not root.exists():
        return out
    for mcap_file in sorted(root.glob("*/*.mcap")):
        if "_clip_" in mcap_file.stem:
            continue  # clip exports are artifacts of a run, not runs
        base = mcap_file.with_suffix("")
        seal_path = Path(str(base) + ".seal")
        entry = {
            "run": mcap_file.stem, "robot_id": mcap_file.parent.name,
            "mcap": str(mcap_file), "size": mcap_file.stat().st_size,
            "sealed": seal_path.exists(),
            "anchored": Path(str(base) + ".seal.tsr").exists(),
        }
        if seal_path.exists():
            try:
                seal = json.loads(seal_path.read_text())
                entry["sealed_at"] = seal.get("sealed_at")
                entry["merkle_root"] = seal.get("merkle_root")
                entry["message_counts"] = seal.get("message_counts", {})
            except Exception:
                pass
        out.append(entry)
    out.sort(key=lambda e: e["run"], reverse=True)
    return out


def _latest_sealed_run(robot_dir: Path) -> dict | None:
    seals = sorted(robot_dir.glob("*.seal"))
    if not seals:
        return None
    try:
        seal = json.loads(seals[-1].read_text())
        return {"run": seal["run"], "merkle_root": seal["merkle_root"]}
    except Exception:
        return None


# ── verified clip export ─────────────────────────────────────────────────

def export_clip(mcap_path: str | Path, start_ts: float, end_ts: float,
                out_path: str | Path | None = None) -> dict[str, Any]:
    """Cut a time window into a new MCAP plus a proof file.

    The clip is re-encoded, so its bytes are not the source bytes; the proof
    ties it back: it embeds the source seal (Merkle root + signature + anchor
    ref) and the sha256 of every clipped message, then signs the bundle.
    A verifier holding the source run can re-derive both sides.
    """
    from mcap.reader import make_reader
    from mcap.writer import Writer

    mcap_path = Path(mcap_path)
    source = verify_mcap_run(mcap_path)
    if source["state"] == "broken":
        return {"ok": False, "error": f"refusing to clip a broken run: {source['reason']}"}

    base = mcap_path.with_suffix("")
    seal_path = Path(str(base) + ".seal")
    seal = json.loads(seal_path.read_text()) if seal_path.exists() else None

    out_path = Path(out_path) if out_path else Path(
        str(base) + f"_clip_{int(start_ts)}_{int(end_ts)}.mcap")

    msg_hashes: list[str] = []
    count = 0
    with open(mcap_path, "rb") as src, open(out_path, "wb") as dst:
        reader = make_reader(src)
        writer = Writer(dst)
        writer.start(profile="", library="roborun-clip")
        schema_map: dict[int, int] = {}
        channel_map: dict[int, int] = {}
        for schema, channel, message in reader.iter_messages(
                start_time=int(start_ts * 1e9), end_time=int(end_ts * 1e9)):
            if schema and schema.id not in schema_map:
                schema_map[schema.id] = writer.register_schema(
                    schema.name, schema.encoding, schema.data)
            if channel.id not in channel_map:
                channel_map[channel.id] = writer.register_channel(
                    channel.topic, channel.message_encoding,
                    schema_map.get(schema.id, 0) if schema else 0)
            writer.add_message(channel_map[channel.id], message.log_time,
                               message.data, message.publish_time)
            msg_hashes.append(hashlib.sha256(message.data).hexdigest())
            count += 1
        writer.finish()

    clip_sha = hashlib.sha256(out_path.read_bytes()).hexdigest()
    messages_root = merkle_root(msg_hashes)
    proof: dict[str, Any] = {
        "format": "roborun-clip-proof-v1",
        "source_run": mcap_path.stem,
        "source_state": source["state"],
        "source_merkle_root": source.get("merkle_root"),
        "source_seal": seal,
        "window": {"start_ts": start_ts, "end_ts": end_ts},
        "message_count": count,
        "messages_merkle_root": messages_root,
        "clip_sha256": clip_sha,
        "exported_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "note": "clip is re-encoded; the proof binds its messages to the "
                "sealed, anchored source run",
    }
    proof["signature"] = sign_message(
        f"{clip_sha}|{messages_root}|{count}".encode())
    proof_path = out_path.with_suffix(".proof.json")
    proof_path.write_text(json.dumps(proof, indent=1))
    return {"ok": True, "clip": str(out_path), "proof": str(proof_path),
            "messages": count, "source_state": source["state"]}


# ── module-level active recorder (used by webcam pipeline + routes) ──────

_active: RunRecorder | None = None
_active_lock = threading.Lock()


def start_recording(robot_id: str = "local", **kwargs) -> RunRecorder:
    global _active
    with _active_lock:
        if _active is not None and not _active._closed:
            return _active
        _active = RunRecorder(robot_id=robot_id, **kwargs)
        _active.attach_event_bus()
        return _active


def stop_recording(do_anchor: bool = True) -> dict[str, Any] | None:
    global _active
    with _active_lock:
        if _active is None:
            return None
        seal = _active.close(do_anchor=do_anchor)
        _active = None
        return seal


def active_recorder() -> RunRecorder | None:
    with _active_lock:
        if _active is not None and _active._closed:
            return None
        return _active


# ── CLI ──────────────────────────────────────────────────────────────────

def _cli() -> int:
    import argparse
    p = argparse.ArgumentParser(prog="roborun.recorder",
                                description="Verify, anchor-upgrade, and clip MCAP runs")
    p.add_argument("command", choices=["verify", "upgrade", "clip", "list"])
    p.add_argument("mcap", nargs="?")
    p.add_argument("start", nargs="?", type=float)
    p.add_argument("end", nargs="?", type=float)
    args = p.parse_args()

    if args.command == "list":
        for r in list_runs():
            badge = "anchored" if r["anchored"] else ("sealed" if r["sealed"] else "open")
            print(f"{r['robot_id']}/{r['run']}  {r['size']:>10} B  [{badge}]")
        return 0
    if not args.mcap:
        p.error("mcap path required")
    if args.command == "upgrade":
        base = Path(args.mcap).with_suffix("")
        seal_path = Path(str(base) + ".seal")
        digest = None
        if seal_path.exists():
            try:
                digest = bytes.fromhex(json.loads(seal_path.read_text())["merkle_root"])
            except Exception:
                pass
        print(json.dumps(anchor.upgrade(Path(str(base) + ".seal.tsr"), digest=digest),
                         indent=1))
        return 0
    if args.command == "clip":
        if args.start is None or args.end is None:
            p.error("clip requires start and end timestamps (unix seconds)")
        print(json.dumps(export_clip(args.mcap, args.start, args.end), indent=1))
        return 0
    r = verify_mcap_run(args.mcap)
    state = r["state"]
    mark = {"verified_anchored": "VERIFIED + ANCHORED",
            "consistent_unanchored": "CONSISTENT (unanchored)",
            "broken": "BROKEN"}[state]
    print(f"{mark}: {r.get('reason', '')}")
    if "merkle_root" in r:
        print(f"merkle root: {r['merkle_root']}")
    return 0 if state != "broken" else 1


if __name__ == "__main__":
    raise SystemExit(_cli())
