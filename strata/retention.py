"""FIFO quota enforcement — ReductStore's defining retention behaviour.

When a bucket's total size exceeds its quota, the oldest data is evicted first.
Eviction works at segment granularity across *all* entries in the bucket: repeat
"drop the globally-oldest segment" until the bucket is back under quota. Segments
are immutable, so eviction is just a manifest `remove` commit plus an object
delete — no rewrite.
"""
from __future__ import annotations

from .manifest import Collection, Commit


def enforce_fifo(blobstore, bucket: str, quota_size: int, *, max_evictions: int = 10000) -> int:
    """Evict oldest segments until bucket size <= quota_size. Returns count evicted."""
    evicted = 0
    for _ in range(max_evictions):
        # gather every segment across the bucket's entries with its source entry
        b = blobstore.get_bucket(bucket)
        catalog_entries = list(b.get("entries", {}))
        total = 0
        all_segs: list[tuple[str, dict]] = []
        for entry in catalog_entries:
            st = Collection(blobstore.store, blobstore._eprefix(bucket, entry)).load()
            for s in st.seg_list():
                total += s["size"]
                all_segs.append((entry, s))
        if total <= quota_size or len(all_segs) <= 1:
            break
        # drop the globally oldest segment (lowest min_t)
        entry, victim = min(all_segs, key=lambda es: es[1]["min_t"])
        Collection(blobstore.store, blobstore._eprefix(bucket, entry)).commit(
            lambda st, vid=victim["id"]: Commit(remove=[vid]))
        if victim.get("key"):
            blobstore.store.delete(victim["key"])
        evicted += 1
    return evicted
