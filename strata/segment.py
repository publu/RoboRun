"""Immutable on-object segment formats.

Two kinds, both write-once and self-describing so any reader can decode a
segment with only its bytes:

* `BlobSegment` — a batch of time-series records (timestamp + labels +
  content-type + opaque blob). Records are sorted by time; a gzip'd JSON index
  carries the metadata and a flat heap carries the blobs. This is the
  ReductStore record model.

* `VectorSegment` — a columnar batch of vectors: ids, a float32 `(n, dim)`
  matrix stored raw, plus schemaless columnar attributes. A precomputed L2 norm
  vector and centroid let the query layer skip whole segments cheaply. This is
  the turbopuffer namespace model.

Segments are never mutated. Deletes are tombstones recorded in the manifest;
space is reclaimed by compaction (rewriting live records into a fresh segment).
"""
from __future__ import annotations

import gzip
import json
import struct
import uuid
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

_BLOB_MAGIC = b"STRBSEG\x01"
_VEC_MAGIC = b"STRVSEG\x01"
_H = struct.Struct("<8sI")     # magic, meta_len


def new_id() -> str:
    return uuid.uuid4().hex


# ─────────────────────────────── blob segments ───────────────────────────────
@dataclass
class Record:
    time: int                         # unix microseconds (matches ReductStore)
    data: bytes
    labels: dict[str, str] = field(default_factory=dict)
    content_type: str = "application/octet-stream"


class BlobSegment:
    def __init__(self, records: list[Record]):
        self.records = sorted(records, key=lambda r: r.time)

    @property
    def min_time(self) -> int:
        return self.records[0].time if self.records else 0

    @property
    def max_time(self) -> int:
        return self.records[-1].time if self.records else 0

    def serialize(self) -> bytes:
        heap = bytearray()
        index = []
        for r in self.records:
            off = len(heap)
            heap += r.data
            index.append({"t": r.time, "l": r.labels, "ct": r.content_type,
                          "o": off, "n": len(r.data)})
        meta = gzip.compress(json.dumps({
            "records": index, "min_t": self.min_time, "max_t": self.max_time,
            "count": len(self.records),
        }).encode())
        return _H.pack(_BLOB_MAGIC, len(meta)) + meta + bytes(heap)

    @classmethod
    def deserialize(cls, raw: bytes) -> "BlobSegment":
        magic, mlen = _H.unpack_from(raw, 0)
        if magic != _BLOB_MAGIC:
            raise ValueError("not a blob segment")
        off = _H.size
        meta = json.loads(gzip.decompress(raw[off:off + mlen]))
        heap = memoryview(raw)[off + mlen:]
        recs = [Record(time=e["t"], data=bytes(heap[e["o"]:e["o"] + e["n"]]),
                       labels=e["l"], content_type=e["ct"]) for e in meta["records"]]
        return cls(recs)

    @staticmethod
    def peek_meta(raw: bytes) -> dict:
        """Decode just the index (no blob copy) — for query planning."""
        magic, mlen = _H.unpack_from(raw, 0)
        if magic != _BLOB_MAGIC:
            raise ValueError("not a blob segment")
        return json.loads(gzip.decompress(raw[_H.size:_H.size + mlen]))


# ────────────────────────────── vector segments ──────────────────────────────
class VectorSegment:
    def __init__(self, ids: list[str], vectors: np.ndarray,
                 attributes: Optional[dict[str, list]] = None):
        vectors = np.ascontiguousarray(vectors, dtype=np.float32)
        if vectors.ndim != 2:
            raise ValueError("vectors must be 2-D (n, dim)")
        if len(ids) != vectors.shape[0]:
            raise ValueError("ids/vectors length mismatch")
        self.ids = list(ids)
        self.vectors = vectors
        self.attributes = attributes or {}

    @property
    def dim(self) -> int:
        return int(self.vectors.shape[1])

    @property
    def count(self) -> int:
        return int(self.vectors.shape[0])

    def centroid(self) -> list[float]:
        return self.vectors.mean(axis=0).tolist() if self.count else []

    def serialize(self) -> bytes:
        norms = np.linalg.norm(self.vectors, axis=1)
        meta = gzip.compress(json.dumps({
            "ids": self.ids, "dim": self.dim, "count": self.count,
            "attrs": self.attributes, "centroid": self.centroid(),
            "norms": norms.tolist(),
        }).encode())
        return _H.pack(_VEC_MAGIC, len(meta)) + meta + self.vectors.tobytes()

    @classmethod
    def deserialize(cls, raw: bytes) -> "VectorSegment":
        magic, mlen = _H.unpack_from(raw, 0)
        if magic != _VEC_MAGIC:
            raise ValueError("not a vector segment")
        off = _H.size
        meta = json.loads(gzip.decompress(raw[off:off + mlen]))
        body = raw[off + mlen:]
        n, d = meta["count"], meta["dim"]
        vecs = np.frombuffer(body, dtype=np.float32, count=n * d).reshape(n, d).copy()
        seg = cls(meta["ids"], vecs, meta.get("attrs", {}))
        seg._norms = np.asarray(meta.get("norms", []), dtype=np.float32)
        return seg

    @property
    def norms(self) -> np.ndarray:
        n = getattr(self, "_norms", None)
        if n is None or len(n) != self.count:
            n = np.linalg.norm(self.vectors, axis=1)
            self._norms = n
        return n

    @staticmethod
    def peek_meta(raw: bytes) -> dict:
        magic, mlen = _H.unpack_from(raw, 0)
        if magic != _VEC_MAGIC:
            raise ValueError("not a vector segment")
        m = json.loads(gzip.decompress(raw[_H.size:_H.size + mlen]))
        m.pop("norms", None)   # keep planning metadata small
        return m
