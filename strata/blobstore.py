"""Time-series blob store — ReductStore feature parity, object-store native.

Model (ReductStore-compatible):

    store ─ buckets ─ entries ─ records(timestamp, labels, content-type, blob)

* Records are written by microsecond timestamp into an entry (a named stream).
* Recent writes coalesce into an in-memory block; it seals into an immutable
  segment when it hits `max_block_records` / `max_block_size` / `max_block_age`,
  or on explicit `flush()`. Reads/queries flush first for read-your-writes.
* Queries select a time window `[start, stop)` and filter by labels.
* Buckets enforce a **FIFO quota**: once total size exceeds `quota_size`, the
  oldest segments are evicted automatically.

The engine holds no authoritative state in RAM — every sealed segment and every
manifest commit lives in the object store, so a second process (or a restart)
sees all flushed data immediately.
"""
from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Iterator, Optional

from .backend import ObjectStore
from .manifest import Catalog, Collection, Commit
from .query import matches
from .segment import BlobSegment, Record, new_id

US = 1_000_000


def now_us() -> int:
    return int(time.time() * US)


class NoSuchBucket(KeyError):
    pass


class NoSuchEntry(KeyError):
    pass


class NoSuchRecord(KeyError):
    pass


@dataclass
class BucketSettings:
    quota_type: str = "NONE"           # NONE | FIFO
    quota_size: int = 0                # bytes; 0 = unlimited
    max_block_size: int = 64 * 1024 * 1024
    max_block_records: int = 1024
    max_block_age: float = 5.0         # seconds before an idle block seals

    @classmethod
    def from_dict(cls, d: dict) -> "BucketSettings":
        f = {k: d[k] for k in cls.__annotations__ if k in d}
        return cls(**f)


class BlobStore:
    def __init__(self, store: ObjectStore):
        self.store = store
        self.catalog = Catalog(store)
        self._buf: dict[tuple, list[Record]] = {}
        self._buf_bytes: dict[tuple, int] = {}
        self._buf_since: dict[tuple, float] = {}
        self._lock = threading.RLock()

    # ── buckets ──────────────────────────────────────────────────────────────
    def create_bucket(self, name: str, settings: Optional[dict] = None) -> dict:
        s = BucketSettings.from_dict(settings or {})
        def fn(cat):
            cat.setdefault("buckets", {})
            if name in cat["buckets"]:
                raise FileExistsError(f"bucket exists: {name}")
            cat["buckets"][name] = {"settings": asdict(s), "entries": {},
                                    "created": time.time()}
        self.catalog.update(fn)
        return self.get_bucket(name)

    def ensure_bucket(self, name: str, settings: Optional[dict] = None) -> dict:
        try:
            return self.create_bucket(name, settings)
        except FileExistsError:
            return self.get_bucket(name)

    def get_bucket(self, name: str) -> dict:
        b = self.catalog.get().get("buckets", {}).get(name)
        if b is None:
            raise NoSuchBucket(name)
        return b

    def list_buckets(self) -> list[dict]:
        out = []
        for name in self.catalog.get().get("buckets", {}):
            st = self.bucket_stats(name)
            out.append(st)
        return out

    def update_bucket(self, name: str, settings: dict) -> dict:
        def fn(cat):
            if name not in cat.get("buckets", {}):
                raise NoSuchBucket(name)
            cur = cat["buckets"][name]["settings"]
            cur.update(asdict(BucketSettings.from_dict({**cur, **settings})))
        self.catalog.update(fn)
        return self.get_bucket(name)

    def remove_bucket(self, name: str) -> None:
        with self._lock:
            for k in list(self._buf):
                if k[0] == name:
                    self._buf.pop(k, None); self._buf_bytes.pop(k, None); self._buf_since.pop(k, None)
        b = self.get_bucket(name)
        for entry in list(b.get("entries", {})):
            Collection(self.store, self._eprefix(name, entry)).drop()
        def fn(cat):
            cat.get("buckets", {}).pop(name, None)
        self.catalog.update(fn)

    def _settings(self, name: str) -> BucketSettings:
        return BucketSettings.from_dict(self.get_bucket(name)["settings"])

    def _eprefix(self, bucket: str, entry: str) -> str:
        return f"b/{bucket}/{entry}"

    def _register_entry(self, bucket: str, entry: str) -> None:
        def fn(cat):
            b = cat.get("buckets", {}).get(bucket)
            if b is None:
                raise NoSuchBucket(bucket)
            b.setdefault("entries", {}).setdefault(entry, {"created": time.time()})
        self.catalog.update(fn)

    # ── writes ───────────────────────────────────────────────────────────────
    def write(self, bucket: str, entry: str, data: bytes, *, time: Optional[int] = None,
              labels: Optional[dict] = None, content_type: str = "application/octet-stream",
              durable: bool = False) -> int:
        ts = time if time is not None else now_us()
        rec = Record(time=ts, data=bytes(data), labels=labels or {}, content_type=content_type)
        self.write_records(bucket, entry, [rec], durable=durable)
        return ts

    def write_records(self, bucket: str, entry: str, records: list[Record], *,
                       durable: bool = False) -> None:
        self.get_bucket(bucket)  # validate
        key = (bucket, entry)
        with self._lock:
            if key not in self._buf:
                self._buf[key] = []
                self._buf_bytes[key] = 0
                self._buf_since[key] = time.time()
                self._register_entry(bucket, entry)
            self._buf[key].extend(records)
            self._buf_bytes[key] += sum(len(r.data) for r in records)
            s = self._settings(bucket)
            seal = (durable or len(self._buf[key]) >= s.max_block_records
                    or self._buf_bytes[key] >= s.max_block_size)
        if seal:
            self.flush(bucket, entry)

    def _flush_one(self, bucket: str, entry: str) -> None:
        key = (bucket, entry)
        with self._lock:
            recs = self._buf.pop(key, None)
            self._buf_bytes.pop(key, None)
            self._buf_since.pop(key, None)
        if not recs:
            return
        seg = BlobSegment(recs)
        sid = new_id()
        skey = f"{self._eprefix(bucket, entry)}/seg/{sid}.seg"
        body = seg.serialize()
        self.store.put(skey, body)
        meta = {"id": sid, "key": skey, "min_t": seg.min_time, "max_t": seg.max_time,
                "count": len(recs), "size": len(body)}
        Collection(self.store, self._eprefix(bucket, entry)).commit(
            lambda st: Commit(add=[meta]))
        self._enforce_quota(bucket)

    def flush(self, bucket: Optional[str] = None, entry: Optional[str] = None) -> None:
        with self._lock:
            keys = [k for k in self._buf
                    if (bucket is None or k[0] == bucket) and (entry is None or k[1] == entry)]
        for b, e in keys:
            self._flush_one(b, e)

    def flush_stale(self) -> None:
        """Seal blocks idle longer than their bucket's max_block_age (server tick)."""
        now = time.time()
        with self._lock:
            stale = []
            for k, since in list(self._buf_since.items()):
                try:
                    age = self._settings(k[0]).max_block_age
                except NoSuchBucket:
                    age = 0
                if now - since >= age:
                    stale.append(k)
        for b, e in stale:
            self._flush_one(b, e)

    # ── reads ────────────────────────────────────────────────────────────────
    def entries(self, bucket: str) -> list[dict]:
        b = self.get_bucket(bucket)
        out = []
        for entry in b.get("entries", {}):
            self.flush(bucket, entry)
            st = Collection(self.store, self._eprefix(bucket, entry)).load()
            segs = st.seg_list()
            if not segs:
                out.append({"name": entry, "size": 0, "record_count": 0,
                            "block_count": 0, "oldest_record": 0, "latest_record": 0})
                continue
            out.append({
                "name": entry,
                "size": sum(s["size"] for s in segs),
                "record_count": sum(s["count"] for s in segs),
                "block_count": len(segs),
                "oldest_record": min(s["min_t"] for s in segs),
                "latest_record": max(s["max_t"] for s in segs),
            })
        return out

    def _segments(self, bucket: str, entry: str) -> list[dict]:
        b = self.get_bucket(bucket)
        if entry not in b.get("entries", {}):
            raise NoSuchEntry(entry)
        self.flush(bucket, entry)
        return Collection(self.store, self._eprefix(bucket, entry)).load().seg_list()

    def read(self, bucket: str, entry: str, time: Optional[int] = None) -> Record:
        segs = self._segments(bucket, entry)
        if not segs:
            raise NoSuchRecord(f"{bucket}/{entry}")
        if time is None:                              # latest record
            seg_meta = max(segs, key=lambda s: s["max_t"])
            seg = BlobSegment.deserialize(self.store.get(seg_meta["key"]))
            return seg.records[-1]
        # find the segment whose window covers `time`, then the exact record
        for s in sorted(segs, key=lambda s: s["min_t"]):
            if s["min_t"] <= time <= s["max_t"]:
                seg = BlobSegment.deserialize(self.store.get(s["key"]))
                for r in seg.records:
                    if r.time == time:
                        return r
        raise NoSuchRecord(f"{bucket}/{entry}@{time}")

    def latest(self, bucket: str, entry: str) -> Record:
        return self.read(bucket, entry, None)

    def query(self, bucket: str, entry: str, *, start: Optional[int] = None,
              stop: Optional[int] = None, labels: Optional[list] = None,
              limit: Optional[int] = None, head_only: bool = False,
              each_n: Optional[int] = None, each_s: Optional[float] = None) -> Iterator[Record]:
        """Records in `[start, stop)` matching `labels`. Downsample with `each_n`
        (every Nth matching record) or `each_s` (at most one per S seconds)."""
        segs = self._segments(bucket, entry)
        lo = start if start is not None else -1
        hi = stop if stop is not None else (1 << 63)
        n = 0           # emitted
        seen = 0        # matched (for each_n)
        last_t = None   # last emitted time (for each_s)
        gap = int(each_s * US) if each_s else None
        for s in sorted(segs, key=lambda s: s["min_t"]):
            if s["max_t"] < lo or s["min_t"] >= hi:
                continue
            raw = self.store.get(s["key"])
            if raw is None:
                continue
            seg = BlobSegment.deserialize(raw)
            for r in seg.records:
                if r.time < lo or r.time >= hi:
                    continue
                if labels and not matches(labels, r.labels):
                    continue
                if each_n and (seen % each_n) != 0:
                    seen += 1
                    continue
                seen += 1
                if gap is not None and last_t is not None and (r.time - last_t) < gap:
                    continue
                last_t = r.time
                if head_only:
                    r = Record(time=r.time, data=b"", labels=r.labels, content_type=r.content_type)
                yield r
                n += 1
                if limit and n >= limit:
                    return

    # ── stats + quota ────────────────────────────────────────────────────────
    def bucket_stats(self, bucket: str) -> dict:
        b = self.get_bucket(bucket)
        entries = self.entries(bucket)
        size = sum(e["size"] for e in entries)
        return {
            "name": bucket, "settings": b["settings"], "entry_count": len(entries),
            "size": size, "record_count": sum(e["record_count"] for e in entries),
            "oldest_record": min((e["oldest_record"] for e in entries if e["record_count"]), default=0),
            "latest_record": max((e["latest_record"] for e in entries if e["record_count"]), default=0),
            "entries": entries,
        }

    def _enforce_quota(self, bucket: str) -> None:
        from .retention import enforce_fifo
        s = self._settings(bucket)
        if s.quota_type == "FIFO" and s.quota_size > 0:
            enforce_fifo(self, bucket, s.quota_size)

    # ── compaction ───────────────────────────────────────────────────────────
    def compact(self, bucket: str, entry: str) -> dict:
        """Merge many small segments into size-bounded blocks (fewer objects →
        faster queries). Records keyed by time, so merging is conflict-free with
        concurrent writes: a segment that appears mid-compaction is simply left
        as-is. Returns {before, after, records}."""
        self.flush(bucket, entry)
        col = Collection(self.store, self._eprefix(bucket, entry))
        st = col.load()
        old = st.seg_list()
        if len(old) <= 1:
            return {"before": len(old), "after": len(old), "records": 0}
        recs: list[Record] = []
        for s in sorted(old, key=lambda s: s["min_t"]):
            raw = self.store.get(s["key"])
            if raw:
                recs.extend(BlobSegment.deserialize(raw).records)
        recs.sort(key=lambda r: r.time)
        setn = self._settings(bucket)
        # repack into blocks bounded by max_block_records / max_block_size
        blocks: list[list[Record]] = []
        cur: list[Record] = []
        cur_bytes = 0
        for r in recs:
            cur.append(r); cur_bytes += len(r.data)
            if len(cur) >= setn.max_block_records or cur_bytes >= setn.max_block_size:
                blocks.append(cur); cur = []; cur_bytes = 0
        if cur:
            blocks.append(cur)
        new_meta = []
        for blk in blocks:
            seg = BlobSegment(blk)
            sid = new_id()
            skey = f"{self._eprefix(bucket, entry)}/seg/{sid}.seg"
            body = seg.serialize()
            self.store.put(skey, body)
            new_meta.append({"id": sid, "key": skey, "min_t": seg.min_time,
                             "max_t": seg.max_time, "count": len(blk), "size": len(body)})
        old_ids = [s["id"] for s in old]

        def build(state):
            present = [i for i in old_ids if i in state.segments]
            if not present:
                return Commit()
            return Commit(add=new_meta, remove=present)

        col.commit(build)
        for s in old:                     # reclaim the now-unreferenced blobs
            self.store.delete(s["key"])
        return {"before": len(old), "after": len(new_meta), "records": len(recs)}
