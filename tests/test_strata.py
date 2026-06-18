"""Strata — object-storage-native time-series + vector engine. Full unit + e2e."""
import sys, os, time, threading
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from strata import Strata, Record
from strata.backend import (LocalObjectStore, MemoryObjectStore, S3ObjectStore,
                            PreconditionFailed, open_store)
from strata.segment import BlobSegment, VectorSegment
from strata.manifest import Collection, Commit, Catalog
from strata.blobstore import BlobStore
from strata.vectors import VectorStore
from strata.auth import TokenStore
from strata.query import matches


# ───────────────────────────── backends ──────────────────────────────────────
@pytest.mark.parametrize("make", [
    lambda tp: MemoryObjectStore(),
    lambda tp: LocalObjectStore(str(tp)),
])
def test_backend_basics(make, tmp_path):
    s = make(tmp_path)
    assert s.get("nope") is None
    s.put("a/b.txt", b"hello")
    assert s.get("a/b.txt") == b"hello"
    assert "a/b.txt" in s.list("a/")
    s.delete("a/b.txt")
    assert s.get("a/b.txt") is None


@pytest.mark.parametrize("make", [
    lambda tp: MemoryObjectStore(),
    lambda tp: LocalObjectStore(str(tp)),
])
def test_put_if_absent_cas(make, tmp_path):
    s = make(tmp_path)
    s.put_if_absent("k", b"first")
    with pytest.raises(PreconditionFailed):
        s.put_if_absent("k", b"second")
    assert s.get("k") == b"first"


def test_local_store_sandbox(tmp_path):
    s = LocalObjectStore(str(tmp_path))
    with pytest.raises(ValueError):
        s.put("../escape", b"x")


# ───────────────────────────── segments ──────────────────────────────────────
def test_blob_segment_roundtrip():
    recs = [Record(time=3, data=b"c", labels={"a": "1"}),
            Record(time=1, data=b"a", labels={"a": "2"}, content_type="text/plain"),
            Record(time=2, data=b"b")]
    seg = BlobSegment.deserialize(BlobSegment(recs).serialize())
    assert [r.time for r in seg.records] == [1, 2, 3]      # sorted
    assert seg.records[0].content_type == "text/plain"
    assert seg.records[0].labels == {"a": "2"}
    assert BlobSegment.peek_meta(BlobSegment(recs).serialize())["count"] == 3


def test_vector_segment_roundtrip():
    m = np.random.rand(5, 8).astype(np.float32)
    seg = VectorSegment(["a", "b", "c", "d", "e"], m, {"floor": [1, 2, 1, 3, 2]})
    out = VectorSegment.deserialize(seg.serialize())
    assert out.ids == ["a", "b", "c", "d", "e"]
    assert np.allclose(out.vectors, m)
    assert out.attributes["floor"] == [1, 2, 1, 3, 2]
    assert out.dim == 8 and out.count == 5


# ───────────────────────────── manifest ──────────────────────────────────────
def test_manifest_commit_and_load():
    s = MemoryObjectStore()
    col = Collection(s, "c1")
    assert col.current_version() == 0
    col.commit(lambda st: Commit(add=[{"id": "s1", "size": 10}]))
    col.commit(lambda st: Commit(add=[{"id": "s2", "size": 20}]))
    col.commit(lambda st: Commit(remove=["s1"], meta={"dim": 3}))
    st = col.load()
    assert set(st.segments) == {"s2"}
    assert st.meta["dim"] == 3
    assert st.version == 3


def test_manifest_checkpoint_replay():
    s = MemoryObjectStore()
    col = Collection(s, "c2")
    for i in range(70):                                    # > CHECKPOINT_EVERY
        col.commit(lambda st, i=i: Commit(add=[{"id": f"s{i}"}]))
    assert s.list("c2/checkpoint/")                        # a checkpoint was written
    assert len(col.load().segments) == 70


def test_manifest_concurrent_writers():
    s = MemoryObjectStore()
    errors = []

    def worker(tid):
        col = Collection(s, "shared")
        try:
            for i in range(20):
                col.commit(lambda st: Commit(add=[{"id": f"{tid}-{i}-{st.version}"}]))
        except Exception as e:
            errors.append(e)

    threads = [threading.Thread(target=worker, args=(t,)) for t in range(4)]
    [t.start() for t in threads]
    [t.join() for t in threads]
    assert not errors
    assert len(Collection(s, "shared").load().segments) == 80   # no lost commits


# ───────────────────────────── blob store ────────────────────────────────────
def test_blobstore_write_read_query():
    bs = BlobStore(MemoryObjectStore())
    bs.create_bucket("b")
    t0 = 1_000_000
    for i in range(10):
        bs.write("b", "e", f"d{i}".encode(), time=t0 + i, labels={"floor": str(i % 3)})
    bs.flush()
    assert bs.latest("b", "e").data == b"d9"
    assert bs.read("b", "e", t0 + 5).data == b"d5"
    got = list(bs.query("b", "e", start=t0 + 2, stop=t0 + 9, labels=["floor", "Eq", "2"]))
    assert [r.time - t0 for r in got] == [2, 5, 8]
    with pytest.raises(Exception):
        bs.read("b", "e", t0 + 999)


def test_blobstore_downsample():
    bs = BlobStore(MemoryObjectStore())
    bs.create_bucket("b")
    for i in range(20):
        bs.write("b", "e", b"x", time=1_000_000 + i * 100_000)   # 0.1s apart
    bs.flush()
    assert len(list(bs.query("b", "e", each_n=5))) == 4          # every 5th of 20
    assert len(list(bs.query("b", "e", each_s=0.5))) == 4        # one per 0.5s over ~1.9s


def test_blobstore_fifo_eviction():
    bs = BlobStore(MemoryObjectStore())
    bs.create_bucket("b", {"quota_type": "FIFO", "quota_size": 1500, "max_block_records": 1})
    t0 = 1_000_000
    for i in range(30):
        bs.write("b", "e", b"x" * 200, time=t0 + i)             # 30 * (200+overhead)
    bs.flush()
    st = bs.bucket_stats("b")
    assert st["size"] <= 1500                                   # under quota
    assert st["record_count"] < 30                             # oldest evicted
    assert st["oldest_record"] > t0                            # newest survive


def test_blobstore_durability_reopen():
    s = MemoryObjectStore()
    bs = BlobStore(s)
    bs.create_bucket("b")
    bs.write("b", "e", b"persisted", time=5)
    bs.flush()
    assert BlobStore(s).latest("b", "e").data == b"persisted"   # fresh engine, same store


# ───────────────────────────── vector store ──────────────────────────────────
def test_vectors_ann_and_filters():
    vs = VectorStore(MemoryObjectStore())
    vs.upsert("ns", [
        {"id": "a", "vector": [1, 0, 0], "attributes": {"floor": 1}},
        {"id": "b", "vector": [0, 1, 0], "attributes": {"floor": 2}},
        {"id": "c", "vector": [0.9, 0.1, 0], "attributes": {"floor": 1}},
    ])
    assert [h["id"] for h in vs.query("ns", vector=[1, 0, 0], top_k=2)] == ["a", "c"]
    assert {h["id"] for h in vs.query("ns", vector=[1, 0, 0], top_k=3,
                                      filters=["floor", "Eq", 1])} == {"a", "c"}


def test_vectors_metrics():
    vs = VectorStore(MemoryObjectStore())
    vs.upsert("l2", [{"id": "a", "vector": [0, 0]}, {"id": "b", "vector": [3, 4]}],
              distance_metric="euclidean")
    r = vs.query("l2", vector=[0, 0], top_k=2, distance_metric="euclidean")
    assert r[0]["id"] == "a" and abs(r[1]["dist"] - 5.0) < 1e-4
    vs.upsert("dp", [{"id": "a", "vector": [1, 1]}, {"id": "b", "vector": [2, 2]}],
              distance_metric="dot")
    r = vs.query("dp", vector=[1, 1], top_k=2, distance_metric="dot")
    assert r[0]["id"] == "b"                                    # larger dot first


def test_vectors_upsert_shadow_and_delete():
    vs = VectorStore(MemoryObjectStore())
    vs.upsert("ns", [{"id": "a", "vector": [1, 0], "attributes": {"v": "old"}},
                     {"id": "b", "vector": [0, 1]}])
    vs.upsert("ns", [{"id": "a", "vector": [-1, 0], "attributes": {"v": "new"}}])   # opposite dir
    top = vs.query("ns", vector=[1, 0], top_k=2)
    assert [h["id"] for h in top] == ["b", "a"]                # a now farthest
    assert next(h for h in top if h["id"] == "a")["attributes"]["v"] == "new"  # shadow took
    vs.delete("ns", ["b"])
    ids = {h["id"] for h in vs.query("ns", vector=[1, 0], top_k=5)}
    assert ids == {"a"}


def test_vectors_bm25():
    vs = VectorStore(MemoryObjectStore())
    vs.upsert("kb", [
        {"id": "a", "vector": [1, 0], "attributes": {"t": "yellow crate dock"}},
        {"id": "b", "vector": [0, 1], "attributes": {"t": "blue door office"}},
        {"id": "c", "vector": [1, 1], "attributes": {"t": "yellow forklift"}},
    ])
    r = vs.query("kb", rank_by=("t", "BM25", "yellow crate"), top_k=3)
    assert r[0]["id"] == "a" and r[0]["score"] > r[1]["score"]


def test_vectors_hybrid_fusion():
    vs = VectorStore(MemoryObjectStore())
    vs.upsert("kb", [
        {"id": "a", "vector": [1.0, 0.0], "attributes": {"t": "yellow crate dock"}},
        {"id": "b", "vector": [0.95, 0.05], "attributes": {"t": "blue door office"}},
        {"id": "c", "vector": [0.0, 1.0], "attributes": {"t": "yellow crate shelf"}},
    ])
    # 'a' is both near [1,0] AND matches "yellow crate" → fusion should rank it #1
    r = vs.query("kb", vector=[1.0, 0.0], rank_by=("t", "BM25", "yellow crate"), top_k=3)
    assert r[0]["id"] == "a"
    assert {x["id"] for x in r} == {"a", "b", "c"}


def test_vectors_dim_mismatch():
    vs = VectorStore(MemoryObjectStore())
    vs.upsert("ns", [{"id": "a", "vector": [1, 0, 0]}])
    with pytest.raises(ValueError):
        vs.upsert("ns", [{"id": "b", "vector": [1, 0]}])


def test_blob_compaction():
    s = MemoryObjectStore()
    bs = BlobStore(s)
    bs.create_bucket("b", {"max_block_records": 1, "max_block_size": 1 << 30})  # 1 seg/record
    t0 = 1_000_000
    for i in range(20):
        bs.write("b", "e", f"d{i}".encode(), time=t0 + i, labels={"k": str(i % 2)})
    bs.flush()
    before = list(bs.query("b", "e"))
    assert len(Collection(s, "b/b/e").load().segments) == 20
    bs.update_bucket("b", {"max_block_records": 100})
    res = bs.compact("b", "e")
    assert res["before"] == 20 and res["after"] < res["before"]
    after = list(bs.query("b", "e"))
    assert [(r.time, r.data) for r in before] == [(r.time, r.data) for r in after]   # identical
    assert list(bs.query("b", "e", labels=["k", "Eq", "1"]))                          # labels survive
    assert BlobStore(s).latest("b", "e").data == b"d19"                               # durable


def test_vector_compaction_drops_dead_rows():
    s = MemoryObjectStore()
    vs = VectorStore(s)
    vs.upsert("ns", [{"id": "a", "vector": [1, 0], "attributes": {"v": 1}},
                     {"id": "b", "vector": [0, 1], "attributes": {"v": 1}}])
    vs.upsert("ns", [{"id": "c", "vector": [1, 1], "attributes": {"v": 1}}])
    vs.upsert("ns", [{"id": "a", "vector": [2, 0], "attributes": {"v": 2}}])   # shadow a
    vs.delete("ns", ["b"])                                                     # tombstone b
    before = {h["id"]: tuple(h["attributes"].items()) for h in vs.query("ns", vector=[1, 0], top_k=9)}
    res = vs.compact("ns")
    assert res["after"] == 1 and res["rows"] == 2                              # only live a,c
    after = {h["id"]: tuple(h["attributes"].items()) for h in vs.query("ns", vector=[1, 0], top_k=9)}
    assert before == after                                                    # results unchanged
    assert after["a"] == (("v", 2),)                                          # newest a kept
    # one physical segment, no tombstones, fully durable on a fresh engine
    st = Collection(s, "v/ns").load()
    assert len(st.segments) == 1 and not st.meta.get("tombstones")
    assert {h["id"] for h in VectorStore(s).query("ns", vector=[1, 0], top_k=9)} == {"a", "c"}


# ───────────────────────────── auth ──────────────────────────────────────────
def test_auth_permissions():
    ts = TokenStore(Catalog(MemoryObjectStore()))
    assert ts.verify(None).full_access                         # open when no tokens
    val = ts.create("reader", {"read": ["telemetry"]})
    assert ts.verify(None) is None                             # now closed
    p = ts.verify(val)
    assert p.can_read("telemetry") and not p.can_write("telemetry")
    assert not p.can_read("other")
    admin = ts.create("admin", {"full_access": True})
    assert ts.verify(admin).can_write("anything")
    assert ts.verify("bogus") is None


# ───────────────────────────── filter language ───────────────────────────────
def test_filter_language():
    a = {"floor": 3, "label": "pallet", "score": 0.9}
    assert matches(["floor", "Eq", 3], a)
    assert matches(["score", "Gte", 0.8], a)
    assert matches(["label", "In", ["pallet", "crate"]], a)
    assert matches(["And", [["floor", "Eq", 3], ["label", "Eq", "pallet"]]], a)
    assert matches(["Or", [["floor", "Eq", 9], ["label", "Eq", "pallet"]]], a)
    assert matches(["Not", ["floor", "Eq", 9]], a)
    assert matches(["label", "Glob", "pall*"], a)
    assert not matches(["missing", "Eq", 1], a)
    assert matches(["missing", "NotEq", 1], a)                 # missing field


# ───────────────────────────── server + client e2e ───────────────────────────
@pytest.fixture
def server(tmp_path):
    from strata.server import StrataServer
    srv = StrataServer(str(tmp_path / "store"), port=0).start_background()
    yield srv
    srv.shutdown()


def test_server_blobs_and_vectors(server):
    from strata.client import StrataClient
    c = StrataClient(f"http://127.0.0.1:{server.port}")
    c.create_bucket("tele", {"quota_type": "FIFO", "quota_size": 1 << 20})
    t0 = 1_000_000
    for i in range(5):
        c.write("tele", "odom", f"d{i}".encode(), time=t0 + i, labels={"floor": str(i % 2)})
    data, meta = c.read("tele", "odom")
    assert data == b"d4" and meta["labels"]["floor"] == "0"
    rows = c.query("tele", "odom", labels=["floor", "Eq", "1"], include_data=True)
    assert [r["data"] for r in rows] == [b"d1", b"d3"]
    assert c.write_batch("tele", "cam", [{"time": t0 + 10 + i, "data": f"f{i}".encode()} for i in range(8)]) == 8

    c.vector_upsert("mem", [
        {"id": "a", "vector": [1, 0, 0], "attributes": {"floor": 1, "t": "yellow crate"}},
        {"id": "b", "vector": [0, 1, 0], "attributes": {"floor": 2, "t": "blue door"}},
    ])
    assert c.vector_query("mem", vector=[1, 0, 0], top_k=1)[0]["id"] == "a"
    assert c.vector_query("mem", rank_by=["t", "BM25", "crate"], top_k=1)[0]["id"] == "a"
    assert any(n["name"] == "mem" for n in c.namespaces())
    # compaction over the wire
    for i in range(6):
        c.write("tele", "odom", f"x{i}".encode(), time=t0 + 100 + i)
    res = c.compact("tele", "odom")
    assert res["after"] <= res["before"]
    assert c.vector_compact("mem")["after"] >= 0


def test_server_auth_enforced(server):
    from strata.client import StrataClient, StrataError
    base = f"http://127.0.0.1:{server.port}"
    open_client = StrataClient(base)
    open_client.create_bucket("tele")
    tok = open_client.create_token("reader", {"read": ["tele"]})    # flips auth ON
    with pytest.raises(StrataError) as e:
        StrataClient(base).write("tele", "x", b"y")
    assert e.value.status == 401
    reader = StrataClient(base, token=tok)
    reader.bucket("tele")                                            # allowed
    with pytest.raises(StrataError) as e:
        reader.write("tele", "x", b"y")
    assert e.value.status == 403


# ───────────────────────────── replication ───────────────────────────────────
def test_replication(server, tmp_path):
    from strata.client import StrataClient
    from strata.replication import Replication
    # source = a local engine; destination = the running server
    src = BlobStore(LocalObjectStore(str(tmp_path / "src")))
    src.create_bucket("tele")
    t0 = 1_000_000
    for i in range(10):
        src.write("tele", "odom", f"d{i}".encode(), time=t0 + i, labels={"floor": str(i % 2)})
    src.flush()
    dest = StrataClient(f"http://127.0.0.1:{server.port}")
    rep = Replication(src, dest, name="mirror", bucket="tele",
                      label_filter=["floor", "Eq", "0"])
    n = rep.drain()
    assert n == 5                                                    # only floor==0
    rows = dest.query("tele", "odom", include_data=True)
    assert sorted(r["data"] for r in rows) == [f"d{i}".encode() for i in range(0, 10, 2)]
    # idempotent: nothing new on a second pass
    assert rep.sync() == 0


# ───────────────────────────── S3 backend (fake client) ──────────────────────
class _FakeS3:
    """Minimal in-memory S3 API — exercises S3ObjectStore incl. IfNoneMatch CAS."""
    def __init__(self):
        self.d = {}
        from botocore.exceptions import ClientError
        self._CE = ClientError

    def _err(self, code, op):
        return self._CE({"Error": {"Code": code}}, op)

    def get_object(self, Bucket, Key):
        if Key not in self.d:
            raise self._err("NoSuchKey", "GetObject")
        import io
        return {"Body": io.BytesIO(self.d[Key])}

    def head_object(self, Bucket, Key):
        if Key not in self.d:
            raise self._err("404", "HeadObject")
        return {}

    def put_object(self, Bucket, Key, Body, IfNoneMatch=None):
        if IfNoneMatch == "*" and Key in self.d:
            raise self._err("PreconditionFailed", "PutObject")
        self.d[Key] = bytes(Body)
        return {}

    def delete_object(self, Bucket, Key):
        self.d.pop(Key, None)
        return {}

    def list_objects_v2(self, Bucket, Prefix="", ContinuationToken=None):
        keys = sorted(k for k in self.d if k.startswith(Prefix))
        return {"Contents": [{"Key": k} for k in keys], "IsTruncated": False}


def test_s3_backend_full_engine_on_fake_s3():
    s3 = S3ObjectStore("bucket", "prefix", client=_FakeS3())
    # CAS works on the S3 path
    s3.put_if_absent("k", b"1")
    with pytest.raises(PreconditionFailed):
        s3.put_if_absent("k", b"2")
    # the WHOLE engine runs on the S3 backend
    db = Strata(s3)
    db.blobs.create_bucket("b")
    db.blobs.write("b", "e", b"on-s3", time=7)
    db.blobs.flush()
    assert db.blobs.latest("b", "e").data == b"on-s3"
    db.vectors.upsert("ns", [{"id": "a", "vector": [1, 0], "attributes": {"x": 1}}])
    assert db.vectors.query("ns", vector=[1, 0], top_k=1)[0]["id"] == "a"
    assert db.info()["buckets"] == 1


def test_open_store_urls(tmp_path):
    assert isinstance(open_store(str(tmp_path)), LocalObjectStore)
    assert isinstance(open_store(f"file://{tmp_path}"), LocalObjectStore)
    with pytest.raises(ValueError):
        open_store("ftp://nope")
