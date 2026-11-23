"""On-disk cache of signed safety packs, content-addressed. A pack is stored only after its root
signature verifies; the file name is the pack's own address, ``SafetyPack.digest_ref()`` (``canonical.pack_digest``):
the digest segments name in ``context.pack`` and the API serves at ``GET /v1/packs/{digest}``. An edited file no
longer matches its address and is refused on read. Activation is rt's decision (the loosening rule), never the
cache's."""
from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path

from ..rt.canonical import from_jsonable, pack_digest, to_jsonable
from ..rt.datatypes import Refused
from ..rt.pack import SIG_KIND
from ..rt.signing import verify


class PackCache:
    def __init__(self, root: str | Path, root_keys: dict):
        self.root = Path(root)
        self.root_keys = root_keys

    def _path(self, dg: str) -> Path:
        if not dg.startswith("sha256:") or len(dg) != 71:
            raise Refused("malformed", "pack digests are sha256:<64 hex>")
        return self.root / (dg.split(":", 1)[1] + ".json")

    def put(self, raw: dict) -> str:
        ok, why = verify(raw, SIG_KIND, self.root_keys)
        if not ok:
            raise Refused("pack_signature", f"pack not cached: {why}")
        dg = pack_digest(raw)
        self.root.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(self.root), prefix=".pack-", suffix=".json")
        with os.fdopen(fd, "w") as fh:
            json.dump(to_jsonable(raw), fh, sort_keys=True)
        os.replace(tmp, self._path(dg))
        return dg

    def get(self, dg: str) -> dict | None:
        path = self._path(dg)
        try:
            raw = from_jsonable(json.loads(path.read_text()))
        except (OSError, ValueError):
            return None
        if pack_digest(raw) != dg or not verify(raw, SIG_KIND, self.root_keys)[0]:
            raise Refused("pack_tampered", f"cached pack {dg} does not match its address or signature")
        return raw

    def digests(self) -> list[str]:
        if not self.root.is_dir():
            return []
        return sorted("sha256:" + p.stem for p in self.root.glob("*.json"))
