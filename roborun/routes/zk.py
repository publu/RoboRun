"""ZK proof routes."""
from __future__ import annotations

import threading

from roborun.routes import get, post, send_json, ApiError
from roborun.routes._singletons import get_memory
from roborun.routes.tasks import log_event


@get("/api/zk/status")
def status(h):
    from roborun.zk_prover import get_prover
    prover = get_prover()
    send_json(h, 200, {
        "ok": True,
        "ezkl_available": prover.is_available(),
        "circuit_ready": prover._ready,
        "circuit_path": str(prover._circuit) if prover._ready else None,
    })


@get("/api/zk/verify/(?P<shard_id>.+)")
def verify(h, shard_id):
    from roborun.zk_prover import get_prover
    prover = get_prover()
    proof_bytes, meta = prover.load_proof(shard_id)
    if not proof_bytes:
        send_json(h, 404, {"ok": False, "error": f"No proof found for shard {shard_id}"})
        return
    verified = prover.verify(proof_bytes)
    send_json(h, 200, {"ok": True, "shard_id": shard_id, "verified": verified, "meta": meta})


@post("/api/zk/setup")
def setup(h, payload):
    force = bool(payload.get("force", False))
    from roborun.zk_prover import get_prover
    prover = get_prover()
    if not prover.is_available():
        send_json(h, 503, {"ok": False, "error": "ezkl not installed — pip install ezkl"})
        return

    def _do():
        result = prover.setup(force=force)
        log_event("zk_setup", "ZK circuit setup complete" if result.get("ok") else "ZK setup failed",
                  data=result, level="info" if result.get("ok") else "error")
    threading.Thread(target=_do, daemon=True).start()
    send_json(h, 202, {"ok": True, "message": "ZK circuit setup started (background)."})


@post("/api/zk/prove")
def prove(h, payload):
    shard_id = str(payload.get("shard_id", "")).strip()
    if not shard_id:
        raise ApiError(400, "shard_id required")
    from roborun.zk_prover import get_prover
    prover = get_prover()
    if not prover.is_available():
        send_json(h, 503, {"ok": False, "error": "ezkl not installed"})
        return
    send_json(h, 202, {"ok": True, "message": "Proof generation started (background).",
                        "shard_id": shard_id})

    def _do():
        try:
            import numpy as np
            import cv2
            mem = get_memory()
            records = [r for r in mem.list_memories(limit=1000) if r.get("shard_id") == shard_id]
            if not records:
                log_event("zk_prove", f"Shard {shard_id} not found", level="error")
                return
            frames, embeddings = [], []
            for rec in records:
                thumb = mem.get_thumbnail(rec["id"])
                if thumb:
                    arr = cv2.imdecode(np.frombuffer(thumb, np.uint8), cv2.IMREAD_COLOR)
                    if arr is not None:
                        frames.append(arr)
                        if rec.get("embedding"):
                            embeddings.append(np.frombuffer(bytes(rec["embedding"]), np.float32))
            if not frames:
                log_event("zk_prove", f"No frames in shard {shard_id}", level="error")
                return
            proof = prover.prove(frames, embeddings)
            if proof and "proof" in proof:
                prover.save_proof(proof, shard_id)
                log_event("zk_prove", f"Proof generated for shard {shard_id}",
                          data={"proof_hash": proof["proof_hash"], "frames": len(frames)})
            else:
                log_event("zk_prove", f"Proof failed for shard {shard_id}",
                          data=proof or {}, level="error")
        except Exception as exc:
            log_event("zk_prove", f"Proof exception: {exc}", level="error")
    threading.Thread(target=_do, daemon=True).start()
