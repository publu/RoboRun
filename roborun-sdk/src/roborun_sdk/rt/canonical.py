"""Canonical bytes for signatures and content addresses, matching the wire schema package (manifest.wire.v1).

- A ``sig`` covers ``canonical_bytes(message minus sig)``: the canonical JSON (sorted keys, no whitespace, bytes as
  standard base64) of the message as received -- ``manifest_wire.signing_payload(raw)``, the one convention the API
  and the SDK share. Never a model re-dump (defaults, 0 -> 0.0 would change the bytes).
- Content addresses (a pack's ``sha256:<hex>`` that segments name in ``context.pack``) are the wire's
  ``digest_ref()``: the canonical JSON of the whole model, defaults and signature included.
JSON files may carry bytes as ``{"$b64": ...}`` (this SDK's dev files) or as plain base64 strings (the wire's JSON
rendering); both load to the same message and sign alike.
"""
from __future__ import annotations

import base64
import hashlib
import json
import math

from . import wire


def normalize(obj):
    """Finite, string-keyed, JSON-shaped values (bytes kept as bytes)."""
    if isinstance(obj, dict):
        out = {}
        for key in sorted(obj):
            if not isinstance(key, str):
                raise TypeError("message keys must be strings")
            out[key] = normalize(obj[key])
        return out
    if isinstance(obj, (list, tuple)):
        return [normalize(v) for v in obj]
    if isinstance(obj, (bytes, bytearray, memoryview)):
        return bytes(obj)
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, int):
        return int(obj)
    if isinstance(obj, float):
        if not math.isfinite(obj):
            raise ValueError("non-finite number in a signed message")
        return float(obj)
    item = getattr(obj, "item", None)
    if callable(item):
        return normalize(item())
    raise TypeError(f"unsupported value in a signed message: {type(obj).__name__}")


def wire_jsonable(obj):
    """The wire's JSON rendering of a raw message: bytes as plain base64 strings (manifest_wire Blob)."""
    if isinstance(obj, dict):
        return {k: wire_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [wire_jsonable(v) for v in obj]
    if isinstance(obj, (bytes, bytearray, memoryview)):
        return base64.b64encode(bytes(obj)).decode()
    return obj


def canonical_json(value) -> bytes:
    """manifest_wire.canonical_json: sorted keys, no whitespace, no NaN, UTF-8."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False, ensure_ascii=False).encode()


def canonical_bytes(obj) -> bytes:
    """Canonical JSON of a raw message (bytes as base64)."""
    return canonical_json(wire_jsonable(normalize(obj)))


def unsigned_view(msg: dict) -> dict:
    return {k: v for k, v in msg.items() if k != "sig"}


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def digest(obj) -> str:
    """Content address of a raw message, signature excluded (local bookkeeping only)."""
    return "sha256:" + sha256_hex(canonical_bytes(unsigned_view(obj) if isinstance(obj, dict) else obj))


def pack_digest(raw: dict) -> str:
    """The address segments name in ``context.pack``: the wire's ``SafetyPack.digest_ref()`` (signature included)."""
    if wire.available():
        try:
            return wire.model(raw, "safety-pack").digest_ref()
        except ValueError:
            pass
    return "sha256:" + sha256_hex(canonical_bytes(raw))


def to_jsonable(obj):
    if isinstance(obj, dict):
        return {k: to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]
    if isinstance(obj, (bytes, bytearray, memoryview)):
        return {"$b64": base64.b64encode(bytes(obj)).decode()}
    return obj


def from_jsonable(obj):
    if isinstance(obj, dict):
        if set(obj) == {"$b64"} and isinstance(obj["$b64"], str):
            return base64.b64decode(obj["$b64"], validate=True)
        return {k: from_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [from_jsonable(v) for v in obj]
    return obj
