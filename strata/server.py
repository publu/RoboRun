"""Strata HTTP server — REST over both data models, stdlib only.

Blob (ReductStore-style) endpoints under `/api/v1/b/...` and vector/full-text
(turbopuffer-style) endpoints under `/api/v1/vectors/...`, on one port over one
object store. Bearer-token auth is enforced when any token exists.

    strata serve --store s3://bucket/prefix?endpoint=http://localhost:9000 --port 8420
"""
from __future__ import annotations

import base64
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional
from urllib.parse import urlparse, parse_qs

from . import Strata, __version__
from .blobstore import NoSuchBucket, NoSuchEntry, NoSuchRecord
from .vectors import NoSuchNamespace
from .segment import Record

LABEL_PREFIX = "x-strata-label-"


class _Route:
    def __init__(self, method: str, pattern: str, fn: str, *, perm: str = "read",
                 resource_arg: Optional[str] = None):
        self.method = method
        self.regex = re.compile("^" + pattern + "$")
        self.fn = fn
        self.perm = perm                       # read | write | admin | none
        self.resource_arg = resource_arg


ROUTES = [
    _Route("GET", r"/", "root", perm="none"),
    _Route("GET", r"/api/v1/info", "info", perm="none"),
    _Route("GET", r"/api/v1/list", "list_buckets"),
    # blobs
    _Route("POST", r"/api/v1/b/(?P<bucket>[^/]+)", "create_bucket", perm="write", resource_arg="bucket"),
    _Route("PUT", r"/api/v1/b/(?P<bucket>[^/]+)", "update_bucket", perm="write", resource_arg="bucket"),
    _Route("GET", r"/api/v1/b/(?P<bucket>[^/]+)", "bucket_info", resource_arg="bucket"),
    _Route("DELETE", r"/api/v1/b/(?P<bucket>[^/]+)", "remove_bucket", perm="write", resource_arg="bucket"),
    _Route("POST", r"/api/v1/b/(?P<bucket>[^/]+)/(?P<entry>[^/]+)/batch", "batch_write", perm="write", resource_arg="bucket"),
    _Route("POST", r"/api/v1/b/(?P<bucket>[^/]+)/(?P<entry>[^/]+)/compact", "compact_blob", perm="write", resource_arg="bucket"),
    _Route("GET", r"/api/v1/b/(?P<bucket>[^/]+)/(?P<entry>[^/]+)/q", "query_blobs", resource_arg="bucket"),
    _Route("POST", r"/api/v1/b/(?P<bucket>[^/]+)/(?P<entry>[^/]+)", "write_blob", perm="write", resource_arg="bucket"),
    _Route("GET", r"/api/v1/b/(?P<bucket>[^/]+)/(?P<entry>[^/]+)", "read_blob", resource_arg="bucket"),
    _Route("HEAD", r"/api/v1/b/(?P<bucket>[^/]+)/(?P<entry>[^/]+)", "read_blob", resource_arg="bucket"),
    # vectors
    _Route("GET", r"/api/v1/vectors", "list_namespaces"),
    _Route("POST", r"/api/v1/vectors/(?P<ns>[^/]+)/query", "query_vectors", resource_arg="ns"),
    _Route("POST", r"/api/v1/vectors/(?P<ns>[^/]+)/compact", "compact_vectors", perm="write", resource_arg="ns"),
    _Route("POST", r"/api/v1/vectors/(?P<ns>[^/]+)/delete", "delete_vectors", perm="write", resource_arg="ns"),
    _Route("POST", r"/api/v1/vectors/(?P<ns>[^/]+)", "upsert_vectors", perm="write", resource_arg="ns"),
    _Route("DELETE", r"/api/v1/vectors/(?P<ns>[^/]+)", "delete_namespace", perm="write", resource_arg="ns"),
    # tokens (admin)
    _Route("GET", r"/api/v1/tokens", "list_tokens", perm="admin"),
    _Route("POST", r"/api/v1/tokens/(?P<name>[^/]+)", "create_token", perm="admin"),
    _Route("DELETE", r"/api/v1/tokens/(?P<name>[^/]+)", "remove_token", perm="admin"),
]


class StrataHandler(BaseHTTPRequestHandler):
    server_version = f"Strata/{__version__}"
    db: Strata = None      # injected

    def log_message(self, *a):       # quiet by default
        pass

    # — dispatch —
    def _handle(self, method: str):
        u = urlparse(self.path)
        path = u.path.rstrip("/") or "/"
        self.query = {k: v[-1] for k, v in parse_qs(u.query).items()}
        for r in ROUTES:
            if r.method != method:
                continue
            m = r.regex.match(path)
            if not m:
                continue
            args = m.groupdict()
            perms = self._authorize(r, args)
            if perms is None:
                return
            try:
                getattr(self, "h_" + r.fn)(**args)
            except (NoSuchBucket, NoSuchEntry, NoSuchRecord, NoSuchNamespace) as e:
                self._json({"error": f"not found: {e}"}, 404)
            except FileExistsError as e:
                self._json({"error": str(e)}, 409)
            except (ValueError, KeyError) as e:
                self._json({"error": str(e)}, 422)
            except Exception as e:               # pragma: no cover
                self._json({"error": f"internal: {e}"}, 500)
            return
        self._json({"error": "no such route"}, 404)

    def do_GET(self): self._handle("GET")
    def do_POST(self): self._handle("POST")
    def do_PUT(self): self._handle("PUT")
    def do_DELETE(self): self._handle("DELETE")
    def do_HEAD(self): self._handle("HEAD")

    # — auth —
    def _authorize(self, route: _Route, args: dict):
        tok = None
        h = self.headers.get("Authorization", "")
        if h.startswith("Bearer "):
            tok = h[7:].strip()
        perms = self.db.tokens.verify(tok)
        if route.perm == "none":
            return perms or self.db.tokens.verify(None) or True
        if perms is None:
            self._json({"error": "unauthorized"}, 401)
            return None
        ok = True
        if route.perm == "admin":
            ok = perms.full_access
        elif route.perm == "write":
            ok = perms.can_write(args.get(route.resource_arg, "*")) if route.resource_arg else perms.full_access
        elif route.perm == "read":
            ok = perms.can_read(args.get(route.resource_arg, "*")) if route.resource_arg else True
        if not ok:
            self._json({"error": "forbidden"}, 403)
            return None
        return perms

    # — io helpers —
    def _body(self) -> bytes:
        n = int(self.headers.get("Content-Length", 0) or 0)
        return self.rfile.read(n) if n else b""

    def _json_body(self) -> dict:
        b = self._body()
        return json.loads(b) if b else {}

    def _json(self, obj, code: int = 200):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(data)

    def _labels(self) -> dict:
        out = {}
        for k, v in self.headers.items():
            kl = k.lower()
            if kl.startswith(LABEL_PREFIX):
                out[kl[len(LABEL_PREFIX):]] = v
        return out

    # ── handlers: meta ───────────────────────────────────────────────────────
    def h_root(self):
        self._json({"engine": "strata", "version": __version__,
                    "docs": "/api/v1/info", "models": ["blobs", "vectors"]})

    def h_info(self):
        self._json(self.db.info())

    def h_list_buckets(self):
        self._json({"buckets": self.db.blobs.list_buckets()})

    # ── handlers: blobs ──────────────────────────────────────────────────────
    def h_create_bucket(self, bucket):
        self.db.blobs.create_bucket(bucket, self._json_body())
        self._json({"bucket": bucket, "created": True}, 201)

    def h_update_bucket(self, bucket):
        self._json(self.db.blobs.update_bucket(bucket, self._json_body()))

    def h_bucket_info(self, bucket):
        self._json(self.db.blobs.bucket_stats(bucket))

    def h_remove_bucket(self, bucket):
        self.db.blobs.remove_bucket(bucket)
        self._json({"bucket": bucket, "removed": True})

    def h_write_blob(self, bucket, entry):
        ts = self.query.get("ts")
        ts = int(ts) if ts else None
        ct = self.headers.get("Content-Type", "application/octet-stream")
        written = self.db.blobs.write(bucket, entry, self._body(), time=ts,
                                      labels=self._labels(), content_type=ct)
        self._json({"bucket": bucket, "entry": entry, "time": written}, 201)

    def h_batch_write(self, bucket, entry):
        body = self._json_body()
        recs = []
        for it in body.get("records", body if isinstance(body, list) else []):
            data = base64.b64decode(it["data"]) if isinstance(it.get("data"), str) else bytes(it.get("data", b""))
            recs.append(Record(time=int(it["time"]), data=data,
                               labels=it.get("labels", {}),
                               content_type=it.get("content_type", "application/octet-stream")))
        self.db.blobs.write_records(bucket, entry, recs, durable=True)
        self._json({"bucket": bucket, "entry": entry, "written": len(recs)}, 201)

    def h_compact_blob(self, bucket, entry):
        self._json(self.db.blobs.compact(bucket, entry))

    def h_read_blob(self, bucket, entry):
        ts = self.query.get("ts")
        rec = self.db.blobs.read(bucket, entry, int(ts) if ts else None)
        self.send_response(200)
        self.send_header("Content-Type", rec.content_type)
        self.send_header("Content-Length", str(len(rec.data)))
        self.send_header("x-strata-time", str(rec.time))
        for k, v in rec.labels.items():
            self.send_header(LABEL_PREFIX + k, str(v))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(rec.data)

    def h_query_blobs(self, bucket, entry):
        q = self.query
        start = int(q["start"]) if "start" in q else None
        stop = int(q["stop"]) if "stop" in q else None
        limit = int(q["limit"]) if "limit" in q else None
        labels = json.loads(q["labels"]) if "labels" in q else None
        include_data = q.get("data") in ("1", "true")
        out = []
        for r in self.db.blobs.query(bucket, entry, start=start, stop=stop,
                                     labels=labels, limit=limit, head_only=not include_data):
            item = {"time": r.time, "labels": r.labels, "content_type": r.content_type}
            if include_data:
                item["data"] = base64.b64encode(r.data).decode()
                item["size"] = len(r.data)
            out.append(item)
        self._json({"bucket": bucket, "entry": entry, "records": out, "count": len(out)})

    # ── handlers: vectors ────────────────────────────────────────────────────
    def h_list_namespaces(self):
        self._json({"namespaces": self.db.vectors.list_namespaces()})

    def h_upsert_vectors(self, ns):
        body = self._json_body()
        rows = body.get("upserts", body.get("rows", []))
        n = self.db.vectors.upsert(ns, rows,
                                   distance_metric=body.get("distance_metric", "cosine"))
        self._json({"namespace": ns, "upserted": n}, 200)

    def h_query_vectors(self, ns):
        body = self._json_body()
        rank_by = body.get("rank_by")
        rank_by = tuple(rank_by) if rank_by else None
        res = self.db.vectors.query(
            ns, vector=body.get("vector"), top_k=int(body.get("top_k", 10)),
            filters=body.get("filters"), distance_metric=body.get("distance_metric"),
            include_attributes=body.get("include_attributes", True), rank_by=rank_by)
        self._json({"namespace": ns, "results": res, "count": len(res)})

    def h_compact_vectors(self, ns):
        self._json(self.db.vectors.compact(ns))

    def h_delete_vectors(self, ns):
        ids = self._json_body().get("ids", [])
        self._json({"namespace": ns, "deleted": self.db.vectors.delete(ns, ids)})

    def h_delete_namespace(self, ns):
        self.db.vectors.delete_namespace(ns)
        self._json({"namespace": ns, "removed": True})

    # ── handlers: tokens ─────────────────────────────────────────────────────
    def h_list_tokens(self):
        self._json({"tokens": self.db.tokens.list()})

    def h_create_token(self, name):
        value = self.db.tokens.create(name, self._json_body())
        self._json({"name": name, "value": value}, 201)

    def h_remove_token(self, name):
        self.db.tokens.remove(name)
        self._json({"name": name, "removed": True})


class StrataServer:
    """Threaded HTTP server + a background ticker that seals idle blocks."""

    def __init__(self, store, host: str = "127.0.0.1", port: int = 8420):
        self.db = Strata(store)
        handler = type("BoundHandler", (StrataHandler,), {"db": self.db})
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self.host, self.port = host, self.httpd.server_address[1]
        self._stop = threading.Event()
        self._ticker = threading.Thread(target=self._tick, daemon=True)

    def _tick(self):
        while not self._stop.wait(2.0):
            try:
                self.db.blobs.flush_stale()
            except Exception:
                pass

    def serve_forever(self):
        self._ticker.start()
        try:
            self.httpd.serve_forever()
        finally:
            self.shutdown()

    def start_background(self):
        self._ticker.start()
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        return self

    def shutdown(self):
        self._stop.set()
        try:
            self.db.flush()
        except Exception:
            pass
        self.httpd.shutdown()
        self.httpd.server_close()
