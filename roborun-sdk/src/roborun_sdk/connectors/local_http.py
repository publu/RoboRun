"""Loopback-only JSON client for the native owners.

No redirects (http.client never follows them), bounded replies, explicit per-request timeouts, and POST only when
the connector was built with ``allow_post=True`` (``roborun stop --live``). Read-only connectors (doctor, tests,
``run --live`` acceptance) raise ``ReadOnlyConnector`` before any byte is sent. Every call is recorded in
``calls`` so tests can prove what was (not) sent.
"""
from __future__ import annotations

import http.client
import json
import threading
from urllib.parse import urlsplit

from ..rt.datatypes import ConnectorError, ReadOnlyConnector

LOOPBACK = ("127.0.0.1", "localhost", "::1")


class OwnerError(ConnectorError):
    def __init__(self, status: int, message: str, data: dict | None = None):
        super().__init__(f"HTTP {status}: {message}")
        self.status = status
        self.message = message
        self.data = data or {}


class LocalJSON:
    def __init__(self, base_url: str, *, allow_post: bool = False, headers: dict | None = None,
                 timeout_s: float = 0.5, max_bytes: int = 256 * 1024):
        url = urlsplit(base_url)
        if url.scheme != "http" or url.hostname not in LOOPBACK or not url.port or url.path not in ("", "/"):
            raise ValueError(f"native owners are loopback-only http://127.0.0.1:<port>: {base_url!r}")
        self.host, self.port = url.hostname, url.port
        self.base_url = base_url.rstrip("/")
        self.allow_post = allow_post
        self.headers = dict(headers or {})
        self.timeout_s = timeout_s
        self.max_bytes = max_bytes
        self.calls: list[tuple[str, str]] = []
        self._lock = threading.Lock()

    def _request(self, method: str, path: str, body, timeout_s: float | None) -> dict:
        if method != "GET" and not self.allow_post:
            raise ReadOnlyConnector(f"read-only connector refuses {method} {path}")
        with self._lock:
            self.calls.append((method, path))
        conn = http.client.HTTPConnection(self.host, self.port, timeout=max(0.02, timeout_s or self.timeout_s))
        try:
            headers = dict(self.headers)
            payload = None
            if method != "GET":
                payload = json.dumps(body if body is not None else {}, allow_nan=False).encode()
                headers["Content-Type"] = "application/json"
            conn.request(method, path, body=payload, headers=headers)
            response = conn.getresponse()
            raw = response.read(self.max_bytes + 1)
            if len(raw) > self.max_bytes:
                raise ConnectorError(f"{method} {path}: reply larger than {self.max_bytes} bytes")
            try:
                data = json.loads(raw) if raw else {}
            except ValueError:
                raise ConnectorError(f"{method} {path}: reply is not JSON") from None
            if not isinstance(data, dict):
                raise ConnectorError(f"{method} {path}: reply is not a JSON object")
            if response.status != 200:
                raise OwnerError(response.status, str(data.get("error") or f"HTTP {response.status}"), data)
            return data
        except (OSError, http.client.HTTPException) as exc:
            raise ConnectorError(f"{method} {self.base_url}{path}: {type(exc).__name__}: {exc}") from None
        finally:
            conn.close()

    def get(self, path: str, *, timeout_s: float | None = None) -> dict:
        return self._request("GET", path, None, timeout_s)

    def post(self, path: str, body: dict | None = None, *, timeout_s: float | None = None) -> dict:
        return self._request("POST", path, body, timeout_s)

    def posts(self) -> list[str]:
        return [p for m, p in self.calls if m != "GET"]
