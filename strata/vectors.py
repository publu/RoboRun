"""Vector + full-text store — turbopuffer feature parity, object-store native.

Model (turbopuffer-compatible):

    store ─ namespaces ─ documents(id, vector, attributes...)

* `upsert` writes a columnar `VectorSegment` (ids, an `(n, dim)` float32 matrix,
  schemaless attribute columns) and commits it to the namespace manifest. Re-
  upserting an id shadows the older row (newest segment wins); `delete` records a
  tombstone. Both are pure manifest commits — no segment rewrite.
* `query` does exact ANN over the live, deduped row set with NumPy (cosine / l2 /
  dot), pushing attribute filters down so only matching rows are scored. Results
  carry the requested attributes back.
* `query(rank_by=...)` does BM25 full-text ranking over a text attribute, so a
  namespace supports vector search, keyword search, or filter-only listing.

Object storage is the source of truth; a per-namespace **live view** (deduped
matrix + norms + attribute columns) is cached in RAM keyed by manifest version,
so repeated queries hit memory and only a new write invalidates the cache. That
"cold object store, hot local cache" split is what makes it cheap *and* fast.
"""
from __future__ import annotations

import math
import re
import threading
from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .backend import ObjectStore
from .manifest import Catalog, Collection, Commit
from .query import matches, referenced_fields
from .segment import VectorSegment, new_id

_WORD = re.compile(r"[a-z0-9]+")


class NoSuchNamespace(KeyError):
    pass


@dataclass
class _Live:
    version: int
    ids: list[str]
    matrix: np.ndarray            # (n, dim) float32
    norms: np.ndarray             # (n,) float32
    attrs: list[dict]             # per-row attribute dicts


def _tokenize(text: str) -> list[str]:
    return _WORD.findall(text.lower()) if isinstance(text, str) else []


class VectorStore:
    def __init__(self, store: ObjectStore, *, cache_views: int = 64):
        self.store = store
        self.catalog = Catalog(store)
        self._views: "OrderedDict[str, _Live]" = OrderedDict()
        self._seg_cache: "OrderedDict[str, VectorSegment]" = OrderedDict()
        self._cap = cache_views
        self._lock = threading.RLock()

    # ── namespaces ───────────────────────────────────────────────────────────
    def _nsprefix(self, ns: str) -> str:
        return f"v/{ns}"

    def namespace_meta(self, ns: str) -> dict:
        return Collection(self.store, self._nsprefix(ns)).load().meta

    def list_namespaces(self) -> list[dict]:
        out = []
        for ns in self.catalog.get().get("namespaces", {}):
            col = Collection(self.store, self._nsprefix(ns))
            st = col.load()
            live = sum(s["count"] for s in st.seg_list())
            out.append({"name": ns, "dimensions": st.meta.get("dim"),
                        "distance_metric": st.meta.get("metric", "cosine"),
                        "approx_count": live - len(st.meta.get("tombstones", []))})
        return out

    def delete_namespace(self, ns: str) -> None:
        Collection(self.store, self._nsprefix(ns)).drop()
        with self._lock:
            self._views.pop(ns, None)
        self.catalog.update(lambda cat: cat.get("namespaces", {}).pop(ns, None))

    def _register_ns(self, ns: str, dim: int, metric: str) -> None:
        def fn(cat):
            cat.setdefault("namespaces", {}).setdefault(
                ns, {"dim": dim, "metric": metric})
        self.catalog.update(fn)

    # ── upsert / delete ──────────────────────────────────────────────────────
    def upsert(self, ns: str, rows: list[dict], *, distance_metric: str = "cosine") -> int:
        """rows: [{"id": str, "vector": [..], "attributes": {..}}]. Returns count."""
        if not rows:
            return 0
        ids, vecs, attr_rows = [], [], []
        for r in rows:
            ids.append(str(r["id"]))
            vecs.append(np.asarray(r["vector"], dtype=np.float32))
            attr_rows.append(dict(r.get("attributes", r.get("attrs", {})) or {}))
        matrix = np.vstack(vecs).astype(np.float32)
        dim = matrix.shape[1]
        cols = sorted({k for a in attr_rows for k in a})
        attributes = {c: [a.get(c) for a in attr_rows] for c in cols}
        seg = VectorSegment(ids, matrix, attributes)
        sid = new_id()
        skey = f"{self._nsprefix(ns)}/seg/{sid}.vseg"
        body = seg.serialize()
        self.store.put(skey, body)
        meta = {"id": sid, "key": skey, "count": len(ids), "dim": dim,
                "seq": None}

        col = Collection(self.store, self._nsprefix(ns))

        def build(st):
            existing = st.meta.get("dim")
            if existing is not None and existing != dim:
                raise ValueError(f"dim mismatch: namespace={existing} upsert={dim}")
            meta["seq"] = st.version + 1                 # newer seq shadows older ids
            mset = {}
            if existing is None:
                mset = {"dim": dim, "metric": distance_metric}
            # clear tombstones for re-upserted ids
            tomb = set(st.meta.get("tombstones", []))
            if tomb & set(ids):
                mset["tombstones"] = sorted(tomb - set(ids))
            return Commit(add=[meta], meta=mset)

        col.commit(build)
        self._register_ns(ns, dim, distance_metric)
        with self._lock:
            self._views.pop(ns, None)
        return len(ids)

    def delete(self, ns: str, ids: list[str]) -> int:
        col = Collection(self.store, self._nsprefix(ns))

        def build(st):
            tomb = set(st.meta.get("tombstones", []))
            tomb |= set(map(str, ids))
            return Commit(meta={"tombstones": sorted(tomb)})

        col.commit(build)
        with self._lock:
            self._views.pop(ns, None)
        return len(ids)

    # ── live view (deduped, cached by manifest version) ──────────────────────
    def _segment(self, key: str) -> VectorSegment:
        with self._lock:
            seg = self._seg_cache.get(key)
            if seg is not None:
                self._seg_cache.move_to_end(key)
                return seg
        seg = VectorSegment.deserialize(self.store.get(key))
        with self._lock:
            self._seg_cache[key] = seg
            while len(self._seg_cache) > 256:
                self._seg_cache.popitem(last=False)
        return seg

    def _live(self, ns: str) -> _Live:
        col = Collection(self.store, self._nsprefix(ns))
        st = col.load()
        if st.version == 0:
            raise NoSuchNamespace(ns)
        with self._lock:
            v = self._views.get(ns)
            if v is not None and v.version == st.version:
                self._views.move_to_end(ns)
                return v
        tomb = set(st.meta.get("tombstones", []))
        # newest segment first → first occurrence of an id wins (upsert shadowing)
        segs = sorted(st.seg_list(), key=lambda s: s.get("seq") or 0, reverse=True)
        seen: set[str] = set()
        ids: list[str] = []
        rows: list[np.ndarray] = []
        attrs: list[dict] = []
        for s in segs:
            seg = self._segment(s["key"])
            cols = seg.attributes
            for i, rid in enumerate(seg.ids):
                if rid in seen or rid in tomb:
                    continue
                seen.add(rid)
                ids.append(rid)
                rows.append(seg.vectors[i])
                attrs.append({c: cols[c][i] for c in cols})
        if rows:
            matrix = np.vstack(rows).astype(np.float32)
            norms = np.linalg.norm(matrix, axis=1)
        else:
            dim = st.meta.get("dim", 1)
            matrix = np.zeros((0, dim), dtype=np.float32)
            norms = np.zeros((0,), dtype=np.float32)
        live = _Live(st.version, ids, matrix, norms, attrs)
        with self._lock:
            self._views[ns] = live
            while len(self._views) > self._cap:
                self._views.popitem(last=False)
        return live

    # ── query ────────────────────────────────────────────────────────────────
    def query(self, ns: str, *, vector: Optional[list] = None, top_k: int = 10,
              filters: Optional[list] = None, distance_metric: Optional[str] = None,
              include_attributes: bool = True, rank_by: Optional[tuple] = None) -> list[dict]:
        """Vector ANN (`vector`), BM25 text (`rank_by=("field","BM25","query")`),
        or filter-only listing. Returns ranked [{id, dist|score, attributes}]."""
        live = self._live(ns)
        meta = self.namespace_meta(ns)
        metric = distance_metric or meta.get("metric", "cosine")
        n = len(live.ids)
        if n == 0:
            return []

        # push filters down to a candidate index set
        if filters:
            cand = np.array([i for i in range(n) if matches(filters, live.attrs[i])], dtype=np.int64)
        else:
            cand = np.arange(n, dtype=np.int64)
        if cand.size == 0:
            return []

        if rank_by is not None:
            return self._bm25(live, cand, rank_by, top_k, include_attributes)

        if vector is None:
            # filter-only listing
            order = cand[:top_k]
            return [self._row(live, int(i), 0.0, include_attributes, score_key="dist") for i in order]

        q = np.asarray(vector, dtype=np.float32)
        M = live.matrix[cand]
        if metric in ("cosine", "cosine_distance"):
            qn = np.linalg.norm(q) or 1.0
            sims = (M @ q) / (live.norms[cand] * qn + 1e-12)
            dist = 1.0 - sims                       # ascending = closer
        elif metric in ("euclidean", "l2", "euclidean_squared"):
            diff = M - q
            dist = np.einsum("ij,ij->i", diff, diff)
            if metric == "euclidean" or metric == "l2":
                dist = np.sqrt(dist)
        elif metric in ("dot", "dot_product", "inner_product"):
            dist = -(M @ q)                         # ascending = larger dot
        else:
            raise ValueError(f"unknown distance_metric: {metric}")

        k = min(top_k, dist.size)
        idx = np.argpartition(dist, k - 1)[:k]
        idx = idx[np.argsort(dist[idx], kind="stable")]
        out = []
        for j in idx:
            gi = int(cand[j])
            d = float(dist[j])
            if metric in ("dot", "dot_product", "inner_product"):
                d = -d                              # report the dot product itself
            out.append(self._row(live, gi, d, include_attributes,
                                  score_key="dist" if metric != "dot" else "score"))
        return out

    def _row(self, live: _Live, i: int, val: float, incl: bool, score_key: str) -> dict:
        r = {"id": live.ids[i], score_key: val}
        if incl:
            r["attributes"] = live.attrs[i]
        return r

    # ── BM25 full-text ───────────────────────────────────────────────────────
    def _bm25(self, live: _Live, cand: np.ndarray, rank_by: tuple, top_k: int,
              incl: bool, k1: float = 1.5, b: float = 0.75) -> list[dict]:
        field, op, query = rank_by[0], rank_by[1], rank_by[2]
        if str(op).upper() not in ("BM25", "TEXT"):
            raise ValueError("rank_by op must be BM25")
        q_terms = _tokenize(query)
        docs = [_tokenize(live.attrs[int(i)].get(field, "")) for i in cand]
        N = len(docs)
        avgdl = (sum(len(d) for d in docs) / N) if N else 0.0
        # document frequency per query term
        df = {t: 0 for t in set(q_terms)}
        for d in docs:
            ds = set(d)
            for t in df:
                if t in ds:
                    df[t] += 1
        idf = {t: math.log(1 + (N - df[t] + 0.5) / (df[t] + 0.5)) for t in df}
        scores = []
        for li, d in enumerate(docs):
            if not d:
                continue
            dl = len(d)
            tf = {}
            for w in d:
                tf[w] = tf.get(w, 0) + 1
            s = 0.0
            for t in q_terms:
                f = tf.get(t, 0)
                if f:
                    s += idf[t] * (f * (k1 + 1)) / (f + k1 * (1 - b + b * dl / (avgdl or 1)))
            if s > 0:
                scores.append((s, int(cand[li])))
        scores.sort(key=lambda x: -x[0])
        return [self._row(live, gi, sc, incl, score_key="score") for sc, gi in scores[:top_k]]
