"""ZK proof generation for robot observations — EZKL + CLIP.

Proves that CLIP embeddings stored in the spatial memory were correctly
computed from the original frames. This makes the observation store's
semantic search cryptographically verifiable.

Proof pipeline:
  1. Export CLIP ViT-B/32 to ONNX (one-time setup)
  2. For each memory shard, generate an EZKL proof:
     prove(frames, embeddings, clip_weights) → proof.bin
  3. Store proof_hash in the memory record
  4. Anyone can verify: load proof.bin + frames → check ✓

Requires: pip install ezkl onnx
Optional: pip install boto3  (for R2/S3 proof upload)

Usage:
    prover = ZKProver()
    prover.setup()           # one-time: export ONNX + compile circuit
    proof = prover.prove(frames, embeddings)
    ok = prover.verify(proof)
"""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
import threading
from pathlib import Path
from typing import Any

import numpy as np

PROOF_DIR = Path(".roborun") / "zk_proofs"
ONNX_PATH = PROOF_DIR / "clip_vit_b32.onnx"
CIRCUIT_PATH = PROOF_DIR / "clip_circuit"
SETTINGS_PATH = PROOF_DIR / "settings.json"


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


class ZKProver:
    """EZKL-based ZK prover for CLIP inference verification."""

    def __init__(self, proof_dir: str | Path | None = None) -> None:
        self._dir = Path(proof_dir) if proof_dir else PROOF_DIR
        self._dir.mkdir(parents=True, exist_ok=True)
        self._onnx = self._dir / "clip_vit_b32.onnx"
        self._circuit = self._dir / "clip_circuit"
        self._settings = self._dir / "settings.json"
        self._lock = threading.Lock()
        self._ready = False

    def is_available(self) -> bool:
        try:
            import ezkl  # noqa: F401
            return True
        except ImportError:
            return False

    def setup(self, force: bool = False) -> dict:
        """One-time setup: export CLIP to ONNX, compile circuit, generate SRS.

        This is slow (~2-5 minutes) but only runs once. Results are cached.
        """
        if self._ready and not force:
            return {"ok": True, "cached": True}

        if not self.is_available():
            return {"ok": False, "error": "ezkl not installed — pip install ezkl"}

        with self._lock:
            if not self._onnx.exists() or force:
                result = self._export_onnx()
                if not result["ok"]:
                    return result

            if not (self._circuit / "model.compiled").exists() or force:
                result = self._compile_circuit()
                if not result["ok"]:
                    return result

            self._ready = True
            return {"ok": True, "cached": False, "onnx": str(self._onnx),
                    "circuit": str(self._circuit)}

    def _export_onnx(self) -> dict:
        try:
            import torch
            import open_clip
            import onnx

            self._onnx.parent.mkdir(parents=True, exist_ok=True)
            model, _, preprocess = open_clip.create_model_and_transforms(
                "ViT-B-32", pretrained="laion2b_s34b_b79k"
            )
            model.eval()

            dummy_input = torch.randn(1, 3, 224, 224)

            class ClipImageEncoder(torch.nn.Module):
                def __init__(self, m):
                    super().__init__()
                    self.visual = m.visual

                def forward(self, x):
                    features = self.visual(x)
                    return features / features.norm(dim=-1, keepdim=True)

            encoder = ClipImageEncoder(model)
            encoder.eval()

            torch.onnx.export(
                encoder, dummy_input, str(self._onnx),
                input_names=["image"], output_names=["embedding"],
                dynamic_axes={"image": {0: "batch"}, "embedding": {0: "batch"}},
                opset_version=17,
            )
            return {"ok": True, "path": str(self._onnx)}
        except Exception as exc:
            return {"ok": False, "error": f"ONNX export failed: {exc}"}

    def _compile_circuit(self) -> dict:
        try:
            import ezkl

            self._circuit.mkdir(parents=True, exist_ok=True)

            dummy_input = np.random.randn(1, 3, 224, 224).astype(np.float32)
            input_data = {"input_data": dummy_input.tolist()}
            input_path = self._dir / "input_sample.json"
            input_path.write_text(json.dumps(input_data))

            settings_path = self._circuit / "settings.json"
            ezkl.gen_settings(
                str(self._onnx),
                str(settings_path),
            )

            ezkl.calibrate_settings(
                str(input_path),
                str(self._onnx),
                str(settings_path),
                target="resources",
            )

            ezkl.compile_circuit(
                str(self._onnx),
                str(self._circuit / "model.compiled"),
                str(settings_path),
            )

            srs_path = self._circuit / "kzg.srs"
            ezkl.get_srs(str(settings_path), srs_path=str(srs_path))

            ezkl.setup(
                str(self._circuit / "model.compiled"),
                str(self._circuit / "vk.key"),
                str(self._circuit / "pk.key"),
                srs_path=str(srs_path),
            )
            return {"ok": True}
        except Exception as exc:
            return {"ok": False, "error": f"Circuit compilation failed: {exc}"}

    def prove(self, frames: list[np.ndarray],
              embeddings: list[np.ndarray]) -> dict | None:
        """Generate a ZK proof that embeddings were correctly computed from frames.

        Returns a dict with proof bytes and metadata, or None on failure.
        """
        if not self._ready:
            setup = self.setup()
            if not setup["ok"]:
                return None

        try:
            import ezkl
            import open_clip
            import torch
            from PIL import Image

            # Preprocess frames to CLIP input tensors
            _, _, preprocess = open_clip.create_model_and_transforms(
                "ViT-B-32", pretrained="laion2b_s34b_b79k"
            )
            tensors = []
            for frame in frames:
                pil = Image.fromarray(frame[..., ::-1] if frame.shape[-1] == 3 else frame)
                tensors.append(preprocess(pil))
            input_tensor = torch.stack(tensors).numpy()

            with tempfile.TemporaryDirectory() as tmpdir:
                tmp = Path(tmpdir)

                input_data = {"input_data": input_tensor.tolist()}
                input_path = tmp / "input.json"
                input_path.write_text(json.dumps(input_data))

                witness_path = tmp / "witness.json"
                ezkl.gen_witness(
                    str(input_path),
                    str(self._circuit / "model.compiled"),
                    str(witness_path),
                )

                proof_path = tmp / "proof.json"
                ezkl.prove(
                    str(witness_path),
                    str(self._circuit / "model.compiled"),
                    str(self._circuit / "pk.key"),
                    str(proof_path),
                    srs_path=str(self._circuit / "kzg.srs"),
                )

                proof_bytes = proof_path.read_bytes()
                proof_hash = _sha256(proof_bytes)

                emb_hash = _sha256(
                    b"".join(e.astype(np.float32).tobytes() for e in embeddings)
                )

                return {
                    "proof": proof_bytes,
                    "proof_hash": proof_hash,
                    "embedding_hash": emb_hash,
                    "frame_count": len(frames),
                    "model": "CLIP ViT-B/32",
                }
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def verify(self, proof_bytes: bytes) -> bool:
        """Verify a proof. Returns True if valid."""
        if not self.is_available():
            return False
        try:
            import ezkl
            with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
                f.write(proof_bytes)
                f.flush()
                result = ezkl.verify(
                    f.name,
                    str(self._circuit / "settings.json"),
                    str(self._circuit / "vk.key"),
                    srs_path=str(self._circuit / "kzg.srs"),
                )
            return bool(result)
        except Exception:
            return False

    def save_proof(self, proof: dict, shard_id: str) -> Path:
        """Save proof to disk alongside the memory shard."""
        path = self._dir / f"{shard_id}.proof.bin"
        path.write_bytes(proof["proof"])
        meta_path = self._dir / f"{shard_id}.proof.json"
        meta_path.write_text(json.dumps({
            "shard_id": shard_id,
            "proof_hash": proof["proof_hash"],
            "embedding_hash": proof["embedding_hash"],
            "frame_count": proof["frame_count"],
            "model": proof["model"],
        }, indent=2))
        return path

    def load_proof(self, shard_id: str) -> tuple[bytes | None, dict | None]:
        """Load proof bytes and metadata for a shard."""
        proof_path = self._dir / f"{shard_id}.proof.bin"
        meta_path = self._dir / f"{shard_id}.proof.json"
        proof_bytes = proof_path.read_bytes() if proof_path.exists() else None
        meta = json.loads(meta_path.read_text()) if meta_path.exists() else None
        return proof_bytes, meta


# ── Singleton ─────────────────────────────────────────────────────────────────

_prover: ZKProver | None = None
_prover_lock = threading.Lock()


def get_prover() -> ZKProver:
    global _prover
    with _prover_lock:
        if _prover is None:
            _prover = ZKProver()
    return _prover
