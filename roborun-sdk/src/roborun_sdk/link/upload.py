"""Bulk uploads on their own connection: content-addressed ``PUT /v1/blobs/{sha256}``, paced to a
byte rate (<= 50 % of the measured uplink), never on the session connection (a 156 KB frame ahead of a heartbeat
delays it 62-250 ms). Resumable chunked uploads are not built yet (whole-blob PUT; the content address makes a
retry idempotent).
"""
from __future__ import annotations

import hashlib
import ssl
import time
import urllib.request
from urllib.parse import urlsplit


class UploadRefused(RuntimeError):
    pass


class BlobUploader:
    def __init__(self, base_url: str, *, token: str | None = None, max_bytes_per_s: float | None = None,
                 timeout_s: float = 10.0, retries: int = 2, ca: str | None = None, require_face_blur: bool = True,
                 sleep=time.sleep):
        url = urlsplit(base_url)
        self.loopback = url.hostname in ("127.0.0.1", "localhost", "::1")
        if url.scheme != "https" and not (url.scheme == "http" and self.loopback):
            raise ValueError("blobs go over https (http only to loopback for local testing)")
        self.base = base_url.rstrip("/")
        self.token = token
        self.rate = max_bytes_per_s
        self.timeout_s = timeout_s
        self.retries = retries
        self.require_face_blur = require_face_blur
        self.sleep = sleep
        self.ctx = ssl.create_default_context(cafile=ca) if url.scheme == "https" else None
        self.sent: list[tuple[str, int]] = []

    def put(self, sha256: str, data: bytes, content_type: str) -> dict:
        if hashlib.sha256(data).hexdigest() != sha256:
            raise UploadRefused("content does not match its address")
        headers = {"Content-Type": content_type, "X-Content-SHA256": sha256}
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
        last = None
        for attempt in range(self.retries + 1):
            started = time.monotonic()
            req = urllib.request.Request(f"{self.base}/v1/blobs/{sha256}", data=data, method="PUT", headers=headers)
            try:
                with urllib.request.urlopen(req, timeout=self.timeout_s, context=self.ctx) as resp:
                    status = resp.status
                self.sent.append((sha256, len(data)))
                if self.rate:
                    # pace: this blob may not have used more than its share of the byte budget
                    spare = len(data) / self.rate - (time.monotonic() - started)
                    if spare > 0:
                        self.sleep(spare)
                return {"sha256": sha256, "status": status, "bytes": len(data)}
            except OSError as exc:
                last = exc
                self.sleep(min(2.0, 0.25 * (2 ** attempt)))
        raise UploadRefused(f"upload of {sha256[:12]} failed: {last}")

    def upload_observation(self, manifest: dict, blobs: dict, order=("rgb", "state", "depth")) -> list:
        if self.require_face_blur and not (manifest.get("privacy") or {}).get("face_blur") and not self.loopback:
            raise UploadRefused("frames leave this computer only face-blurred")
        done = []
        for role in order:
            if role in blobs:
                sha, data, ctype = blobs[role]
                done.append(self.put(sha, data, ctype))
        for role, (sha, data, ctype) in blobs.items():
            if role not in order:
                done.append(self.put(sha, data, ctype))
        return done
