"""Bridge to the wire schema package (``manifest_wire``, the shared manifest.wire.v1 schemas).

Signatures follow the wire's one convention (manifest_wire.base): the signed bytes are the canonical JSON of the
message as received, minus ``sig`` (bytes as standard base64), so the SDK needs no model to compute them
(``canonical.canonical_bytes``, the same rule as ``manifest_wire.signing_payload(raw)``). The models are used to
validate what arrives: ``model(msg, kind)``. rt imports the wire schema package for that (pydantic + stdlib; no torch/cv2).

When the wire schema package is installed it is used. Without it, ``available()`` is False: signatures still verify
(same bytes), only the schema check is skipped, and ``roborun doctor`` says so.
"""
from __future__ import annotations

import sys

_cached = None
_tried = False


def module():
    """The ``manifest_wire`` module, or None."""
    global _cached, _tried
    if _tried:
        return _cached
    _tried = True
    try:
        import manifest_wire  # noqa: F401
        _cached = sys.modules["manifest_wire"]
    except ImportError:
        _cached = None
    return _cached


def available() -> bool:
    return module() is not None


def model(msg: dict, kind: str):
    """Validate ``msg`` as its wire model (raises ValueError with a short summary)."""
    w = module()
    if w is None:
        raise RuntimeError("the wire schema package is not importable")
    try:
        if kind == "safety-pack":
            from manifest_wire.pack import SafetyPack
            return SafetyPack.model_validate(msg)
        return w.parse_message(msg)
    except Exception as exc:  # pydantic.ValidationError and friends
        errors = getattr(exc, "errors", None)
        if callable(errors):
            try:
                first = errors()[0]
                loc = ".".join(str(p) for p in first.get("loc", ()))
                raise ValueError(f"{loc}: {first.get('msg')}"[:300]) from None
            except (IndexError, TypeError):
                pass
        raise ValueError(str(exc)[:300]) from None
