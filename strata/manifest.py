"""Serializable commits on object storage — the consistency core.

A `Collection` is a logical stream of immutable segments (one per time-series
entry, one per vector namespace). Its state — which segments exist plus a small
metadata blob — is maintained as a **numbered commit log** in the object store:

    <prefix>/manifest/0000000001.json   commit (delta: adds / removes / meta)
    <prefix>/manifest/0000000002.json
    <prefix>/checkpoint/0000000005.json full snapshot (so reads replay few deltas)

The current version is the highest manifest number that exists. A writer reads
the current version V, computes its delta, and **`put_if_absent(version V+1)`**.
If two writers race, only one create wins; the loser gets `PreconditionFailed`,
reloads, and retries. That one atomic primitive gives lock-free, serializable
writes directly on S3 — no external lock service, no database. This is the same
optimistic-concurrency design used by object-store table formats (Iceberg/Delta)
and serverless vector stores.
"""
from __future__ import annotations

import gzip
import json
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .backend import ObjectStore, PreconditionFailed

CHECKPOINT_EVERY = 32          # write a full snapshot every N commits
_PAD = 10


def _vkey(prefix: str, n: int) -> str:
    return f"{prefix}/manifest/{n:0{_PAD}d}.json.gz"


def _ckey(prefix: str, n: int) -> str:
    return f"{prefix}/checkpoint/{n:0{_PAD}d}.json.gz"


def _vnum(key: str) -> int:
    return int(key.rsplit("/", 1)[-1].split(".", 1)[0])


@dataclass
class State:
    version: int = 0
    segments: dict[str, dict] = field(default_factory=dict)   # seg_id -> metadata
    meta: dict = field(default_factory=dict)                  # collection settings

    def seg_list(self) -> list[dict]:
        return list(self.segments.values())


@dataclass
class Commit:
    """One transaction: segments to add/remove and metadata to merge."""
    add: list[dict] = field(default_factory=list)            # each must carry "id"
    remove: list[str] = field(default_factory=list)
    meta: dict = field(default_factory=dict)

    def empty(self) -> bool:
        return not self.add and not self.remove and not self.meta


class Collection:
    """A versioned set of segments under one object-store prefix."""

    def __init__(self, store: ObjectStore, prefix: str):
        self.store = store
        self.prefix = prefix.rstrip("/")

    # — version discovery —
    def current_version(self) -> int:
        keys = self.store.list(f"{self.prefix}/manifest/")
        return _vnum(keys[-1]) if keys else 0

    def exists(self) -> bool:
        return self.current_version() > 0

    # — load —
    def load(self) -> State:
        v = self.current_version()
        if v == 0:
            return State()
        # newest checkpoint at or below v
        cps = [k for k in self.store.list(f"{self.prefix}/checkpoint/") if _vnum(k) <= v]
        st = State()
        start = 1
        if cps:
            cp = max(cps, key=_vnum)
            snap = json.loads(gzip.decompress(self.store.get(cp)))
            st.segments = {s["id"]: s for s in snap.get("segments", [])}
            st.meta = snap.get("meta", {})
            start = _vnum(cp) + 1
        for n in range(start, v + 1):
            raw = self.store.get(_vkey(self.prefix, n))
            if raw is None:
                continue
            c = json.loads(gzip.decompress(raw))
            for s in c.get("add", []):
                st.segments[s["id"]] = s
            for sid in c.get("remove", []):
                st.segments.pop(sid, None)
            if c.get("meta"):
                st.meta.update(c["meta"])
        st.version = v
        return st

    # — commit (optimistic, retried) —
    def commit(self, build: Callable[[State], Commit], *, retries: int = 64) -> State:
        """Apply a transaction. `build(state)` returns a Commit computed against
        the *current* state; it is re-invoked on every retry so the writer always
        diffs against fresh data. Returns the new State."""
        for _ in range(retries):
            st = self.load()
            c = build(st)
            if c.empty():
                return st
            nv = st.version + 1
            body = {"v": nv, "ts": time.time(), "add": c.add,
                    "remove": c.remove, "meta": c.meta}
            try:
                self.store.put_if_absent(_vkey(self.prefix, nv),
                                         gzip.compress(json.dumps(body).encode()))
            except PreconditionFailed:
                continue   # someone else committed nv — reload and retry
            # apply locally and maybe checkpoint
            for s in c.add:
                st.segments[s["id"]] = s
            for sid in c.remove:
                st.segments.pop(sid, None)
            st.meta.update(c.meta)
            st.version = nv
            if nv % CHECKPOINT_EVERY == 0:
                self._checkpoint(st)
            return st
        raise RuntimeError(f"commit failed after {retries} retries (contention) on {self.prefix}")

    def _checkpoint(self, st: State) -> None:
        snap = {"v": st.version, "segments": st.seg_list(), "meta": st.meta}
        try:
            self.store.put(_ckey(self.prefix, st.version),
                           gzip.compress(json.dumps(snap).encode()))
        except Exception:
            pass   # checkpoints are an optimization; never fatal

    # — destroy —
    def drop(self, *, delete_segments: bool = True) -> None:
        if delete_segments:
            for s in self.load().seg_list():
                if s.get("key"):
                    self.store.delete(s["key"])
        self.store.delete_prefix(f"{self.prefix}/")


class Catalog:
    """The top-level registry of collections (buckets, vector namespaces).

    A tiny CAS'd index object so `list_collections()` is one GET, not a slow
    prefix-scan. Membership here is advisory; segment manifests are authoritative.
    """

    def __init__(self, store: ObjectStore, root: str = "_catalog"):
        self.store = store
        self.key = f"{root}/index.json"

    def _load(self) -> dict:
        raw = self.store.get(self.key)
        return json.loads(raw) if raw else {"buckets": {}, "namespaces": {}, "tokens": {}}

    def get(self) -> dict:
        return self._load()

    def update(self, fn: Callable[[dict], None], *, retries: int = 64) -> dict:
        for _ in range(retries):
            cur = self._load()
            etag = json.dumps(cur, sort_keys=True)
            fn(cur)
            new = json.dumps(cur).encode()
            # optimistic: re-read, bail if changed under us
            check = self.store.get(self.key)
            if (check or b"") == b"" and not self.store.exists(self.key):
                try:
                    self.store.put_if_absent(self.key, new)
                    return cur
                except PreconditionFailed:
                    continue
            if check is not None and json.dumps(json.loads(check), sort_keys=True) != etag:
                continue
            self.store.put(self.key, new)
            return cur
        raise RuntimeError("catalog update contention")
