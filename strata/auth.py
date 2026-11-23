"""Bearer-token auth with per-resource permissions (ReductStore-compatible).

Tokens live in the catalog (so they persist on the object store). Each token
grants `full_access`, or explicit `read`/`write` lists of buckets/namespaces
("*" = all). Auth is opt-in: if no `STRATA_API_TOKEN` / init token is configured
the server runs open (handy for local dev), exactly like ReductStore.
"""
from __future__ import annotations

import hashlib
import secrets
import time
from dataclasses import dataclass, field
from typing import Optional

from .manifest import Catalog


def _hash(value: str) -> str:
    return hashlib.sha256(value.encode()).hexdigest()


@dataclass
class Permissions:
    full_access: bool = False
    read: list[str] = field(default_factory=list)
    write: list[str] = field(default_factory=list)

    def can_read(self, resource: str) -> bool:
        return self.full_access or "*" in self.read or resource in self.read

    def can_write(self, resource: str) -> bool:
        return self.full_access or "*" in self.write or resource in self.write


class TokenStore:
    def __init__(self, catalog: Catalog):
        self.catalog = catalog

    def create(self, name: str, permissions: dict, *, value: Optional[str] = None) -> str:
        value = value or ("strata-" + secrets.token_urlsafe(32))
        perm = Permissions(**{k: permissions.get(k, getattr(Permissions(), k))
                              for k in Permissions().__dict__})

        def fn(cat):
            cat.setdefault("tokens", {})[name] = {
                "hash": _hash(value), "permissions": perm.__dict__,
                "created": time.time()}
        self.catalog.update(fn)
        return value          # shown once; only the hash is stored

    def remove(self, name: str) -> None:
        self.catalog.update(lambda cat: cat.get("tokens", {}).pop(name, None))

    def list(self) -> list[dict]:
        return [{"name": n, "permissions": t["permissions"], "created": t.get("created")}
                for n, t in self.catalog.get().get("tokens", {}).items()]

    def any_configured(self) -> bool:
        return bool(self.catalog.get().get("tokens"))

    def verify(self, value: Optional[str]) -> Optional[Permissions]:
        """Return Permissions for a token value, or None if invalid.
        If no tokens exist at all, auth is disabled → full access."""
        toks = self.catalog.get().get("tokens", {})
        if not toks:
            return Permissions(full_access=True)
        if not value:
            return None
        h = _hash(value)
        for t in toks.values():
            if secrets.compare_digest(t["hash"], h):
                return Permissions(**t["permissions"])
        return None
