# Strata — object-storage-native data engine

**One engine, two data models, on any S3-compatible endpoint.**

Strata stores *time-series blobs* (ReductStore's model) **and** *vectors +
full-text* (turbopuffer's model) with object storage as the single source of
truth. Local RAM/NVMe is only a cache. There is no external database and no lock
service: serializability comes from a compare-and-swap'd, numbered manifest log
written directly to the bucket.

```python
from strata import Strata
db = Strata("s3://my-bucket/strata?endpoint=http://minio:9000")

# time-series blobs (ReductStore-style)
db.blobs.create_bucket("telemetry", {"quota_type": "FIFO", "quota_size": 10 << 30})
db.blobs.write("telemetry", "odom", payload, labels={"floor": "3"})

# vectors + full-text (turbopuffer-style)
db.vectors.upsert("memory", [{"id": "x", "vector": v, "attributes": {"floor": 3}}])
hits = db.vectors.query("memory", vector=v, top_k=10, filters=["floor", "Eq", 3])
text = db.vectors.query("memory", rank_by=("caption", "BM25", "yellow crate"))
```

Run it as a server: `strata serve --store s3://bucket/prefix?endpoint=... --port 8420`.

---

## Why object storage is the source of truth

Object stores (S3/MinIO/R2/B2) are cheap, infinitely durable, and globally
available — but they have no transactions. Strata gets transactions from a single
atomic primitive every modern S3 implementation supports: **conditional create**
(`PUT If-None-Match: *`). On that one primitive it builds a serializable commit
log, exactly like object-store table formats (Iceberg/Delta) and serverless
vector databases.

```
<prefix>/_catalog/index.json              registry of buckets + namespaces + tokens
<prefix>/b/<bucket>/<entry>/manifest/NNNN.json.gz   commit log (delta: add/remove/meta)
<prefix>/b/<bucket>/<entry>/checkpoint/NNNN.json.gz full snapshot every 32 commits
<prefix>/b/<bucket>/<entry>/seg/<uuid>.seg          immutable blob segment
<prefix>/v/<namespace>/manifest/NNNN.json.gz        vector namespace commit log
<prefix>/v/<namespace>/seg/<uuid>.vseg              immutable columnar vector segment
```

**Commit protocol.** A writer reads the current version `V` (highest manifest
number), computes its delta, and `put_if_absent`s manifest `V+1`. If a
concurrent writer already created `V+1`, the create fails, the writer reloads and
retries against fresh state. No write is ever lost or partially applied. Reads
load the newest checkpoint ≤ V and replay ≤ 32 deltas.

**Segments are immutable.** Writes append new segments; deletes/upserts are
tombstones and shadowing recorded in the manifest. Space is reclaimed by
compaction (rewriting live records into a fresh segment) and, for blobs, by FIFO
eviction.

**Hot/cold split.** Cold truth lives in the bucket; the engine caches
deserialized segments and a per-namespace *live view* (deduped matrix + norms +
attribute columns) in RAM, keyed by manifest version, so repeated queries never
touch object storage and a new write simply invalidates the cache.

---

## Architecture

| Module | Responsibility |
|---|---|
| `backend.py` | `ObjectStore` interface + `Local`/`S3`/`Memory` backends; CAS via `put_if_absent`. |
| `manifest.py` | `Collection` (numbered commit log + checkpoints, optimistic retry) + `Catalog`. |
| `segment.py` | Immutable `BlobSegment` (time-series records) and columnar `VectorSegment`. |
| `blobstore.py` | Buckets/entries/records, write-block coalescing, time/label queries, downsampling. |
| `vectors.py` | Namespaces, upsert/delete, exact ANN (cosine/l2/dot), attribute filters, BM25. |
| `query.py` | Shared filter AST (`Eq/Gt/In/Glob/And/Or/Not/...`). |
| `retention.py` | FIFO quota eviction (drop globally-oldest segment until under quota). |
| `auth.py` | Bearer tokens with per-resource read/write/full permissions. |
| `replication.py` | Checkpointed, filtered source→destination mirroring. |
| `server.py` | REST API (blob + vector) + idle-block sealing ticker. |
| `client.py` / `cli.py` | Python client and `strata` CLI. |

Backends are swappable via URL: `file:///path`, bare path, or
`s3://bucket/prefix?endpoint=...&region=...&access_key=...&secret_key=...`. The
*same* engine code runs on every backend — the S3 path is exercised in tests
against a fake S3 client including the `If-None-Match` CAS.

---

## ReductStore feature parity

| ReductStore feature | Strata |
|---|---|
| Buckets | ✅ `create_bucket`, settings, stats |
| Entries (time-series streams) | ✅ auto-created on first write |
| Records: timestamp (µs) + labels + content-type + blob | ✅ `Record` |
| Write by timestamp / batched write | ✅ `write`, `write_records`, `/batch` endpoint |
| Read latest / read at timestamp | ✅ `latest`, `read(ts)` |
| Query by time range | ✅ `query(start, stop)` |
| Query by labels (conditional) | ✅ filter AST on labels |
| Downsampling (`each_n`, `each_s`) | ✅ `query(each_n=…, each_s=…)` |
| FIFO quota (auto-evict oldest) | ✅ `quota_type=FIFO`, segment-granularity eviction |
| Block (write buffer) sealing by size/records/age | ✅ `max_block_*`, idle ticker |
| Bearer-token auth with permissions | ✅ full / read / write per bucket |
| Replication (filtered, checkpointed) | ✅ `Replication`, at-least-once, idempotent |
| HTTP REST API | ✅ `/api/v1/b/...` |
| Runs on object storage / disk | ✅ object storage *is* the store, not just a tier |

## turbopuffer feature parity

| turbopuffer feature | Strata |
|---|---|
| Namespaces | ✅ |
| Upsert documents (id, vector, attributes) | ✅ `upsert` |
| Schemaless attributes | ✅ columnar, per-segment |
| ANN vector query | ✅ exact NumPy ANN over the cached live view |
| Distance metrics: cosine / euclidean / dot | ✅ |
| Attribute filters (pushed down) | ✅ shared filter AST |
| Full-text BM25 ranking | ✅ `rank_by=(field, "BM25", query)` |
| Delete by id | ✅ tombstones |
| Upsert overwrites (latest wins) | ✅ segment-seq shadowing |
| `include_attributes` in results | ✅ |
| Object storage as source of truth, cached locally | ✅ live-view cache keyed by manifest version |
| Serverless / serializable writes on S3 | ✅ CAS'd manifest log |

> ANN is **exact** (brute force over the cached live view with NumPy). It returns
> ground-truth nearest neighbours and is the right default up to large per-
> namespace sizes; an approximate index (IVF/HNSW persisted per segment with a
> centroid pre-filter — the centroid is already stored in every `VectorSegment`)
> is the planned drop-in for very large namespaces and does not change the API.

---

## HTTP API

Auth (when any token exists): `Authorization: Bearer <token>`.

**Meta** — `GET /api/v1/info`, `GET /api/v1/list`

**Blobs**
- `POST /api/v1/b/{bucket}` — create (body = settings JSON)
- `PUT /api/v1/b/{bucket}` — update settings · `DELETE` — remove · `GET` — stats
- `POST /api/v1/b/{bucket}/{entry}?ts=<µs>` — write (body = blob; labels via
  `x-strata-label-<k>` headers; `Content-Type` preserved)
- `GET  /api/v1/b/{bucket}/{entry}?ts=<µs>` — read (omit `ts` → latest)
- `POST /api/v1/b/{bucket}/{entry}/batch` — batch write (JSON, base64 data)
- `GET  /api/v1/b/{bucket}/{entry}/q?start=&stop=&limit=&labels=&each_n=&each_s=&data=1`

**Vectors**
- `GET    /api/v1/vectors` — list namespaces
- `POST   /api/v1/vectors/{ns}` — upsert (`{upserts:[...], distance_metric}`)
- `POST   /api/v1/vectors/{ns}/query` — `{vector, top_k, filters, distance_metric, rank_by, include_attributes}`
- `POST   /api/v1/vectors/{ns}/delete` — `{ids:[...]}` · `DELETE` — drop namespace

**Tokens** (admin) — `GET /api/v1/tokens`, `POST /api/v1/tokens/{name}`, `DELETE`

---

## Filter language

JSON AST, identical for blob labels and vector attributes:

```
["floor", "Eq", 3]                    Eq NotEq Gt Gte Lt Lte In NotIn Glob Contains
["label", "In", ["pallet", "crate"]]
["And", [["floor", "Eq", 3], ["score", "Gte", 0.8]]]
["Or",  [ ... ]]      ["Not", [ ... ]]
```

Missing fields never satisfy a positive predicate and always satisfy
`NotEq`/`NotIn`.

---

## Status

Implemented and tested (`tests/test_strata.py`, 26 cases): both backends + CAS,
segment formats, manifest commit/checkpoint/**concurrent writers**, blob
write/read/query/downsample/FIFO/durability, vector ANN/metrics/shadow/delete/
BM25, auth, server+client e2e, replication, and the **whole engine running on
the S3 backend**.

Roadmap: per-segment ANN index (IVF/HNSW) for very large namespaces, background
compaction of small blob segments, multi-region read replicas (object storage
already gives durable fan-out), and a RoboRun adapter so MCAP runs + spatial
memory persist straight to Strata on S3.
