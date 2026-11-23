"""Strata — an object-storage-native data engine for robotics & AI.

One engine, two data models, on any S3-compatible endpoint:

* **Time-series blobs** with labels, time/label queries, and FIFO quotas
  (ReductStore feature parity).
* **Vectors + full-text** with attribute filters and BM25 ranking
  (turbopuffer-class search).

Object storage (S3 / MinIO / R2 / B2 / local) is the source of truth; local
RAM/NVMe is just cache. Serializable writes come from a CAS'd, numbered manifest
log — no external database or lock service.

    from strata import Strata
    db = Strata("s3://my-bucket/strata?endpoint=http://localhost:9000")
    db.blobs.create_bucket("telemetry", {"quota_type": "FIFO", "quota_size": 10<<30})
    db.blobs.write("telemetry", "odom", payload, labels={"floor": "3"})
    db.vectors.upsert("memory", [{"id": "x", "vector": v, "attributes": {...}}])
    hits = db.vectors.query("memory", vector=v, top_k=10, filters=["floor","Eq",3])
"""
from __future__ import annotations

from typing import Optional, Union

from .backend import (ObjectStore, LocalObjectStore, S3ObjectStore,
                      MemoryObjectStore, open_store, PreconditionFailed)
from .blobstore import BlobStore, BucketSettings, NoSuchBucket, NoSuchEntry, NoSuchRecord
from .vectors import VectorStore, NoSuchNamespace
from .segment import Record
from .auth import TokenStore, Permissions
from .manifest import Catalog

__version__ = "0.1.0"
__all__ = ["Strata", "ObjectStore", "LocalObjectStore", "S3ObjectStore",
           "MemoryObjectStore", "open_store", "Record", "BucketSettings",
           "BlobStore", "VectorStore", "TokenStore", "Permissions",
           "PreconditionFailed", "NoSuchBucket", "NoSuchEntry", "NoSuchRecord",
           "NoSuchNamespace"]


class Strata:
    """The engine: blob store + vector store + tokens over one object store."""

    def __init__(self, store: Union[str, ObjectStore]):
        self.store: ObjectStore = open_store(store) if isinstance(store, str) else store
        self.blobs = BlobStore(self.store)
        self.vectors = VectorStore(self.store)
        self.catalog = Catalog(self.store)
        self.tokens = TokenStore(self.catalog)

    def info(self) -> dict:
        cat = self.catalog.get()
        return {
            "engine": "strata", "version": __version__,
            "backend": repr(self.store),
            "buckets": len(cat.get("buckets", {})),
            "namespaces": len(cat.get("namespaces", {})),
            "auth": "enabled" if cat.get("tokens") else "disabled",
        }

    def flush(self) -> None:
        self.blobs.flush()
