"""Strata Python client — talks to a Strata server over HTTP (stdlib only).

    c = StrataClient("http://localhost:8420", token="strata-...")
    c.create_bucket("telemetry", {"quota_type": "FIFO", "quota_size": 10 << 30})
    c.write("telemetry", "odom", payload, labels={"floor": "3"})
    c.vector_upsert("memory", [{"id": "x", "vector": v, "attributes": {...}}])
    hits = c.vector_query("memory", vector=v, top_k=10, filters=["floor", "Eq", 3])
"""
from __future__ import annotations

import base64
import json
from typing import Optional
from urllib.error import HTTPError
from urllib.parse import quote, urlencode
from urllib.request import Request, urlopen

from .blobstore import now_us


class StrataError(Exception):
    def __init__(self, status: int, message: str):
        super().__init__(f"[{status}] {message}")
        self.status = status


class StrataClient:
    def __init__(self, base_url: str, token: Optional[str] = None, *, timeout: float = 30.0):
        self.base = base_url.rstrip("/")
        self.token = token
        self.timeout = timeout

    def _req(self, method: str, path: str, *, data: Optional[bytes] = None,
             headers: Optional[dict] = None, parse_json: bool = True):
        req = Request(self.base + path, data=data, method=method)
        if self.token:
            req.add_header("Authorization", f"Bearer {self.token}")
        for k, v in (headers or {}).items():
            req.add_header(k, str(v))
        try:
            with urlopen(req, timeout=self.timeout) as r:
                body = r.read()
                hdrs = dict(r.headers.items())
        except HTTPError as e:
            msg = e.read().decode(errors="replace")
            try:
                msg = json.loads(msg).get("error", msg)
            except Exception:
                pass
            raise StrataError(e.code, msg)
        if parse_json:
            return json.loads(body) if body else {}
        return body, hdrs

    # — meta —
    def info(self) -> dict:
        return self._req("GET", "/api/v1/info")

    # — blobs —
    def create_bucket(self, name: str, settings: Optional[dict] = None) -> dict:
        return self._req("POST", f"/api/v1/b/{quote(name)}",
                         data=json.dumps(settings or {}).encode())

    def update_bucket(self, name: str, settings: dict) -> dict:
        return self._req("PUT", f"/api/v1/b/{quote(name)}", data=json.dumps(settings).encode())

    def buckets(self) -> list:
        return self._req("GET", "/api/v1/list")["buckets"]

    def bucket(self, name: str) -> dict:
        return self._req("GET", f"/api/v1/b/{quote(name)}")

    def remove_bucket(self, name: str) -> dict:
        return self._req("DELETE", f"/api/v1/b/{quote(name)}")

    def write(self, bucket: str, entry: str, data: bytes, *, time: Optional[int] = None,
              labels: Optional[dict] = None, content_type: str = "application/octet-stream") -> int:
        ts = time if time is not None else now_us()
        headers = {"Content-Type": content_type}
        for k, v in (labels or {}).items():
            headers[f"x-strata-label-{k}"] = v
        r = self._req("POST", f"/api/v1/b/{quote(bucket)}/{quote(entry)}?ts={ts}",
                      data=bytes(data), headers=headers)
        return r["time"]

    def write_batch(self, bucket: str, entry: str, records: list[dict]) -> int:
        payload = {"records": [{
            "time": r["time"], "labels": r.get("labels", {}),
            "content_type": r.get("content_type", "application/octet-stream"),
            "data": base64.b64encode(r["data"] if isinstance(r["data"], (bytes, bytearray))
                                     else str(r["data"]).encode()).decode()} for r in records]}
        return self._req("POST", f"/api/v1/b/{quote(bucket)}/{quote(entry)}/batch",
                         data=json.dumps(payload).encode())["written"]

    def compact(self, bucket: str, entry: str) -> dict:
        return self._req("POST", f"/api/v1/b/{quote(bucket)}/{quote(entry)}/compact")

    def read(self, bucket: str, entry: str, time: Optional[int] = None):
        path = f"/api/v1/b/{quote(bucket)}/{quote(entry)}"
        if time is not None:
            path += f"?ts={time}"
        body, hdrs = self._req("GET", path, parse_json=False)
        labels = {k[len('x-strata-label-'):]: v for k, v in hdrs.items()
                  if k.lower().startswith("x-strata-label-")}
        meta = {"time": int(hdrs.get("x-strata-time", 0)),
                "content_type": hdrs.get("Content-Type"), "labels": labels}
        return body, meta

    def query(self, bucket: str, entry: str, *, start: Optional[int] = None,
              stop: Optional[int] = None, labels: Optional[list] = None,
              limit: Optional[int] = None, include_data: bool = False) -> list[dict]:
        q = {}
        if start is not None: q["start"] = start
        if stop is not None: q["stop"] = stop
        if limit is not None: q["limit"] = limit
        if labels is not None: q["labels"] = json.dumps(labels)
        if include_data: q["data"] = 1
        path = f"/api/v1/b/{quote(bucket)}/{quote(entry)}/q?{urlencode(q)}"
        out = self._req("GET", path)["records"]
        if include_data:
            for r in out:
                if "data" in r:
                    r["data"] = base64.b64decode(r["data"])
        return out

    # — vectors —
    def namespaces(self) -> list:
        return self._req("GET", "/api/v1/vectors")["namespaces"]

    def vector_upsert(self, ns: str, rows: list[dict], *, distance_metric: str = "cosine") -> int:
        return self._req("POST", f"/api/v1/vectors/{quote(ns)}",
                         data=json.dumps({"upserts": rows, "distance_metric": distance_metric}).encode())["upserted"]

    def vector_query(self, ns: str, *, vector=None, top_k: int = 10, filters=None,
                     distance_metric=None, rank_by=None, include_attributes: bool = True) -> list[dict]:
        body = {"top_k": top_k, "include_attributes": include_attributes}
        if vector is not None: body["vector"] = list(vector)
        if filters is not None: body["filters"] = filters
        if distance_metric: body["distance_metric"] = distance_metric
        if rank_by: body["rank_by"] = list(rank_by)
        return self._req("POST", f"/api/v1/vectors/{quote(ns)}/query",
                         data=json.dumps(body).encode())["results"]

    def vector_compact(self, ns: str) -> dict:
        return self._req("POST", f"/api/v1/vectors/{quote(ns)}/compact")

    def vector_delete(self, ns: str, ids: list[str]) -> int:
        return self._req("POST", f"/api/v1/vectors/{quote(ns)}/delete",
                         data=json.dumps({"ids": ids}).encode())["deleted"]

    def delete_namespace(self, ns: str) -> dict:
        return self._req("DELETE", f"/api/v1/vectors/{quote(ns)}")

    # — tokens —
    def create_token(self, name: str, permissions: dict) -> str:
        return self._req("POST", f"/api/v1/tokens/{quote(name)}",
                         data=json.dumps(permissions).encode())["value"]

    def tokens(self) -> list:
        return self._req("GET", "/api/v1/tokens")["tokens"]
