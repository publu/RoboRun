"""Replication — mirror records from one Strata to another (ReductStore parity).

A replication task tails new records in a source bucket (optionally restricted to
some entries and a label filter) and writes them to a destination Strata over the
HTTP client. Progress is checkpointed in the source object store, so a restarted
task resumes exactly where it left off — at-least-once delivery, idempotent
because records are keyed by (entry, timestamp).

    rep = Replication(src_blobs, dest_client, name="to-cloud",
                      bucket="telemetry", dest_bucket="telemetry",
                      label_filter=["floor", "Eq", "3"])
    rep.sync()                 # one pass
    rep.run_forever(interval=5)
"""
from __future__ import annotations

import json
import threading
import time
from typing import Optional

from .blobstore import BlobStore
from .client import StrataClient


class Replication:
    def __init__(self, source: BlobStore, dest: StrataClient, *, name: str,
                 bucket: str, dest_bucket: Optional[str] = None,
                 entries: Optional[list[str]] = None, label_filter: Optional[list] = None,
                 batch: int = 256):
        self.src = source
        self.dest = dest
        self.name = name
        self.bucket = bucket
        self.dest_bucket = dest_bucket or bucket
        self.entries = entries
        self.label_filter = label_filter
        self.batch = batch
        self._ck_key = f"_replication/{name}/checkpoint.json"
        self._stop = threading.Event()

    # — checkpoint (per-entry last replicated timestamp) —
    def _load_ck(self) -> dict:
        raw = self.src.store.get(self._ck_key)
        return json.loads(raw) if raw else {}

    def _save_ck(self, ck: dict) -> None:
        self.src.store.put(self._ck_key, json.dumps(ck).encode())

    def _entries(self) -> list[str]:
        if self.entries:
            return self.entries
        return [e["name"] for e in self.src.entries(self.bucket)]

    def sync(self) -> int:
        """One pass. Returns the number of records replicated."""
        # make sure the destination bucket exists (mirror source settings)
        try:
            self.dest.create_bucket(self.dest_bucket,
                                    self.src.get_bucket(self.bucket)["settings"])
        except Exception:
            pass
        ck = self._load_ck()
        total = 0
        for entry in self._entries():
            since = ck.get(entry, -1)
            start = since + 1 if since >= 0 else None
            recs = list(self.src.query(self.bucket, entry, start=start,
                                       labels=self.label_filter, limit=self.batch))
            if not recs:
                continue
            payload = [{"time": r.time, "data": r.data, "labels": r.labels,
                        "content_type": r.content_type} for r in recs]
            self.dest.write_batch(self.dest_bucket, entry, payload)
            ck[entry] = max(r.time for r in recs)
            total += len(recs)
        if total:
            self._save_ck(ck)
        return total

    def drain(self, *, max_passes: int = 1000) -> int:
        """Sync repeatedly until caught up. Returns total replicated."""
        total = 0
        for _ in range(max_passes):
            n = self.sync()
            total += n
            if n < self.batch:
                break
        return total

    def run_forever(self, interval: float = 5.0) -> None:
        while not self._stop.wait(0):
            try:
                self.drain()
            except Exception:
                pass
            if self._stop.wait(interval):
                break

    def stop(self) -> None:
        self._stop.set()
