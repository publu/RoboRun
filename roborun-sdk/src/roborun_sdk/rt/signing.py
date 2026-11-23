"""Ed25519 signatures on segments, controls, admits and packs (the only local proof that
a segment came from the Roborun service).

One convention with the API (manifest.wire.v1):
- The signed bytes are the canonical JSON of the message AS RECEIVED (its JSON rendering: bytes as standard base64)
  without its top-level ``sig`` -- ``manifest_wire.signing_payload(raw)``, never a model re-dump (that would add
  defaults and turn 0 into 0.0). The message must also be a valid wire message of the expected kind (checked, when
  the wire schema package is importable), and it must NAME that kind: ``type`` (``schema`` for a safety pack), which the signed
  bytes cover, so one kind's signature never passes as another's.
- ``sig.value`` and every trusted key's ``pub`` are standard padded base64 (RFC 4648 section 4) in the JSON rendering.
  In msgpack they may travel as ``bin``, the raw 64 / 32 bytes: ``manifest_wire.to_msgpack`` sends every bytes field
  that way (Blob, Sig.value, pub), and its JSON rendering is that same base64 text. ``raw_bytes`` accepts exactly these
  two forms (text: base64; bytes: the raw value, of the exact length), so a frame a sender encoded with the wire's own
  msgpack encoder verifies here.
Backends in order: ``cryptography`` (57-85 us per verify), ``PyNaCl``, then the pure-python
RFC 8032 fallback in ``_ed25519`` (~3 ms; today's venvs have neither library).
"""
from __future__ import annotations

import base64
import binascii
import datetime
from dataclasses import dataclass

from . import _ed25519, wire
from .canonical import canonical_bytes, unsigned_view

PLACEHOLDER_SIG = {"kid": "unsigned", "alg": "Ed25519", "value": base64.b64encode(bytes(64)).decode()}
PACK_KIND = "safety-pack"


def names_kind(msg: dict, kind: str) -> bool:
    """The message names ``kind`` inside its signed bytes: ``type`` == kind, or for a pack a ``manifest.safety-pack/N``
    ``schema``."""
    if kind == PACK_KIND:
        schema = msg.get("schema")
        return isinstance(schema, str) and schema.startswith("manifest.safety-pack/")
    return msg.get("type") == kind


def payload(msg: dict, kind: str) -> bytes:
    """The bytes a ``sig`` covers: the canonical JSON of the message as received, minus ``sig``. Raises ValueError when
    the message does not name ``kind`` or is not a valid wire message of that kind."""
    if not isinstance(msg, dict):
        raise ValueError("a signed message is a JSON object")
    body = unsigned_view(msg)
    if not names_kind(body, kind):
        raise ValueError(f"wrong_kind: the message names {body.get('schema' if kind == PACK_KIND else 'type')!r}, "
                         f"not {kind!r}")
    if wire.available():
        wire.model(dict(body, sig=PLACEHOLDER_SIG), kind)        # schema check only: the bytes are the received ones
    return canonical_bytes(body)


def _backend_name() -> str:
    try:
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey  # noqa: F401
        return "cryptography"
    except Exception:
        pass
    try:
        import nacl.signing  # noqa: F401
        return "nacl"
    except Exception:
        return "pure-python"


BACKEND = _backend_name()


def _verify_raw(public: bytes, message: bytes, signature: bytes) -> bool:
    if BACKEND == "cryptography":
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
        try:
            Ed25519PublicKey.from_public_bytes(public).verify(signature, message)
            return True
        except (InvalidSignature, ValueError):
            return False
    if BACKEND == "nacl":
        import nacl.exceptions
        import nacl.signing
        try:
            nacl.signing.VerifyKey(public).verify(message, signature)
            return True
        except (nacl.exceptions.BadSignatureError, ValueError, TypeError):
            return False
    return _ed25519.verify(public, message, signature)


def b64(data: bytes) -> str:
    return base64.b64encode(data).decode()


def unb64(text: str) -> bytes:
    return base64.b64decode(text, validate=True)


def raw_bytes(value, size: int) -> bytes:
    """A signature value (64 bytes) or public key (32 bytes) as transmitted: standard padded base64 text (the JSON
    rendering) or the raw bytes (msgpack ``bin``, as ``manifest_wire.to_msgpack`` sends them). Raises ValueError for
    any other type or length (bytes are never read as base64 text: one value, one encoding per transport)."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        raw = bytes(value)
    elif isinstance(value, str):
        raw = unb64(value)
    else:
        raise ValueError("expected standard base64 text or raw bytes")
    if len(raw) != size:
        raise ValueError(f"expected {size} bytes, got {len(raw)}")
    return raw


@dataclass(frozen=True)
class SigningKey:
    """A signing key from a 32-byte seed. Dev tools and tests only."""
    kid: str
    seed: bytes

    @property
    def public(self) -> bytes:
        return _ed25519.public_key(self.seed)

    def sign_bytes(self, message: bytes) -> bytes:
        if BACKEND == "cryptography":
            from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
            return Ed25519PrivateKey.from_private_bytes(self.seed).sign(message)
        if BACKEND == "nacl":
            import nacl.signing
            return nacl.signing.SigningKey(self.seed).sign(message).signature
        return _ed25519.sign(self.seed, message)

    def sign(self, msg: dict, kind: str) -> dict:
        """Return a copy of ``msg`` with ``sig`` set (over the wire schema package's signing payload)."""
        body = unsigned_view(msg)
        value = self.sign_bytes(payload(body, kind))
        out = dict(body)
        out["sig"] = {"kid": self.kid, "alg": "Ed25519", "value": b64(value)}
        return out

    def trusted_key(self, not_after: str = "2099-01-01") -> dict:
        return {"kid": self.kid, "alg": "Ed25519", "pub": b64(self.public), "not_after": not_after}


def verify(msg: dict, kind: str, keys: dict[str, bytes]) -> tuple[bool, str]:
    """(ok, reason). ``keys`` maps kid to a 32-byte public key. Total: a malformed message, signature or key never
    raises (a relayed Stop is checked only to fill its log field, after it has latched; K1 has no preconditions)."""
    try:
        return _verify(msg, kind, keys)
    except Exception:
        return False, "malformed_sig"


def _verify(msg, kind: str, keys) -> tuple[bool, str]:
    if not isinstance(msg, dict):
        return False, "malformed"
    sig = msg.get("sig")
    if not isinstance(sig, dict):
        return False, "unsigned"
    if sig.get("alg") != "Ed25519":
        return False, "unsupported_alg"
    kid = sig.get("kid")
    if not isinstance(kid, str):
        return False, "malformed_sig"
    public = keys.get(kid)
    if public is None:
        return False, "unknown_kid"
    try:
        value = raw_bytes(sig.get("value", ""), 64)       # base64 text (JSON) or the raw 64 bytes (msgpack bin)
    except (binascii.Error, ValueError, TypeError):
        return False, "malformed_sig"
    if not names_kind(msg, kind):
        return False, "wrong_kind"
    try:
        message = payload(msg, kind)
    except (TypeError, ValueError) as exc:
        return False, f"wire_invalid: {exc}"[:300]
    return (True, "ok") if _verify_raw(public, message, value) else (False, "bad_signature")


def today_utc() -> datetime.date:
    return datetime.datetime.now(datetime.timezone.utc).date()


def _not_after(entry) -> datetime.date | None:
    """A key's last valid day (``YYYY-MM-DD``, inclusive); None when the entry names none. Raises on a malformed date."""
    value = entry.get("not_after")
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError("not_after must be a YYYY-MM-DD string")
    return datetime.date.fromisoformat(value)


def trusted_entries(entries, *, today: datetime.date | None = None) -> dict[str, tuple[bytes, datetime.date | None]]:
    """``trusted_keys`` list (pack, root key file) -> {kid: (public bytes, not_after)}. Malformed or expired entries are
    skipped, never trusted."""
    today = today or today_utc()
    out = {}
    for entry in entries or ():
        try:
            if entry.get("alg") != "Ed25519":
                continue
            pub = raw_bytes(entry["pub"], 32)              # base64 text (JSON) or the raw 32 bytes (msgpack bin)
            not_after = _not_after(entry)
            if not_after is not None and not_after < today:
                continue                                   # expired: a segment it signed no longer verifies
            out[str(entry["kid"])] = (pub, not_after)
        except (KeyError, binascii.Error, ValueError, TypeError, AttributeError):
            continue
    return out


def keys_from_trusted(entries, *, today: datetime.date | None = None) -> dict[str, bytes]:
    """``trusted_keys`` list (pack) -> {kid: public bytes}, valid today. Malformed or expired entries are skipped."""
    return {kid: pub for kid, (pub, _) in trusted_entries(entries, today=today).items()}


def valid_keys(keys: dict, not_after: dict, *, today: datetime.date | None = None, exclude=()) -> dict[str, bytes]:
    """The keys still in date (checked at use time: a long-running rt outlives a key's last day), minus ``exclude``."""
    today = today or today_utc()
    return {kid: pub for kid, pub in keys.items()
            if kid not in exclude and (not_after.get(kid) is None or today <= not_after[kid])}
