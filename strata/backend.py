"""Object-store backend — the substrate Strata is built on.

Everything in Strata is just objects: immutable segments + numbered manifests.
The engine never touches a real filesystem path or an S3 client directly; it
goes through this `ObjectStore` interface. Swap the backend and the *same*
engine runs on a laptop (`LocalObjectStore`), on AWS S3, MinIO, Cloudflare R2,
Backblaze B2, or any S3-compatible endpoint (`S3ObjectStore`) — that's the whole
point: object storage is the source of truth, local disk/RAM is only cache.

The one non-trivial primitive is **compare-and-swap create** (`put_if_absent`):
write an object only if the key does not already exist. That single atomic
operation is enough to build serializable, lock-free commits on top of object
storage (see `manifest.py`). S3 has supported it via `If-None-Match: *` since
2024; MinIO/R2/B2 support it too; the local backend uses `O_EXCL`.
"""
from __future__ import annotations

import os
import shutil
from dataclasses import dataclass
from typing import Iterable, Optional
from urllib.parse import urlparse, parse_qs


class PreconditionFailed(Exception):
    """A conditional write (put_if_absent) lost the race — the key now exists."""


class ObjectStore:
    """Minimal object-storage interface. All keys are '/'-joined strings."""

    # — reads —
    def get(self, key: str) -> Optional[bytes]:
        raise NotImplementedError

    def exists(self, key: str) -> bool:
        return self.get(key) is not None

    def list(self, prefix: str) -> list[str]:
        raise NotImplementedError

    # — writes —
    def put(self, key: str, data: bytes) -> None:
        raise NotImplementedError

    def put_if_absent(self, key: str, data: bytes) -> None:
        """Create `key` only if it does not exist. Raise PreconditionFailed on race."""
        raise NotImplementedError

    def delete(self, key: str) -> None:
        raise NotImplementedError

    # — convenience —
    def delete_prefix(self, prefix: str) -> int:
        n = 0
        for k in self.list(prefix):
            self.delete(k)
            n += 1
        return n


# ───────────────────────────── local filesystem ──────────────────────────────
class LocalObjectStore(ObjectStore):
    """Filesystem-backed store — the default for dev, tests, and single-node use.

    Behaves like object storage, not a POSIX tree: keys are flat strings, writes
    are atomic (temp-file + rename), and `put_if_absent` uses O_CREAT|O_EXCL so
    the CAS semantics match S3.
    """

    def __init__(self, root: str):
        self.root = os.path.abspath(root)
        os.makedirs(self.root, exist_ok=True)

    def _path(self, key: str) -> str:
        # keys are virtual; map '/' to the filesystem but keep them sandboxed
        p = os.path.normpath(os.path.join(self.root, key))
        if not p.startswith(self.root):
            raise ValueError(f"key escapes store root: {key!r}")
        return p

    def get(self, key: str) -> Optional[bytes]:
        try:
            with open(self._path(key), "rb") as f:
                return f.read()
        except FileNotFoundError:
            return None

    def exists(self, key: str) -> bool:
        return os.path.exists(self._path(key))

    def list(self, prefix: str) -> list[str]:
        base = self._path(prefix)
        # prefix may match a directory or a partial filename
        out: list[str] = []
        search_dir = base if os.path.isdir(base) else os.path.dirname(base)
        if not os.path.isdir(search_dir):
            return out
        for dirpath, _dirs, files in os.walk(search_dir):
            for fn in files:
                full = os.path.join(dirpath, fn)
                rel = os.path.relpath(full, self.root).replace(os.sep, "/")
                if rel.startswith(prefix):
                    out.append(rel)
        return sorted(out)

    def put(self, key: str, data: bytes) -> None:
        p = self._path(key)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        tmp = f"{p}.{os.getpid()}.{id(data) & 0xffffff:x}.tmp"
        with open(tmp, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, p)

    def put_if_absent(self, key: str, data: bytes) -> None:
        p = self._path(key)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        try:
            fd = os.open(p, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            raise PreconditionFailed(key)
        try:
            with os.fdopen(fd, "wb") as f:
                f.write(data)
                f.flush()
                os.fsync(f.fileno())
        except Exception:
            try:
                os.unlink(p)
            except OSError:
                pass
            raise

    def delete(self, key: str) -> None:
        try:
            os.unlink(self._path(key))
        except FileNotFoundError:
            pass

    def __repr__(self) -> str:
        return f"LocalObjectStore({self.root!r})"


# ──────────────────────────────── S3 / S3-compatible ─────────────────────────
class S3ObjectStore(ObjectStore):
    """Any S3-compatible endpoint — AWS S3, MinIO, Cloudflare R2, Backblaze B2.

    boto3 is imported lazily so the engine has no hard dependency on it; install
    `ros-agent[fleet]` (or just `boto3`) to use this backend. `endpoint_url`
    points at non-AWS providers; the rest of Strata is unchanged.
    """

    def __init__(self, bucket: str, prefix: str = "", *, endpoint_url: Optional[str] = None,
                 region: Optional[str] = None, access_key: Optional[str] = None,
                 secret_key: Optional[str] = None, client=None):
        self.bucket = bucket
        self.prefix = prefix.strip("/")
        if client is not None:
            self._s3 = client
        else:
            import boto3  # lazy: optional dependency
            self._s3 = boto3.client(
                "s3", endpoint_url=endpoint_url, region_name=region,
                aws_access_key_id=access_key, aws_secret_access_key=secret_key)
        from botocore.exceptions import ClientError  # noqa
        self._ClientError = ClientError

    def _full(self, key: str) -> str:
        return f"{self.prefix}/{key}" if self.prefix else key

    def get(self, key: str) -> Optional[bytes]:
        try:
            r = self._s3.get_object(Bucket=self.bucket, Key=self._full(key))
            return r["Body"].read()
        except self._ClientError as e:
            if e.response.get("Error", {}).get("Code") in ("NoSuchKey", "404", "NotFound"):
                return None
            raise

    def exists(self, key: str) -> bool:
        try:
            self._s3.head_object(Bucket=self.bucket, Key=self._full(key))
            return True
        except self._ClientError:
            return False

    def list(self, prefix: str) -> list[str]:
        out: list[str] = []
        token = None
        full_prefix = self._full(prefix)
        strip = len(self.prefix) + 1 if self.prefix else 0
        while True:
            kw = dict(Bucket=self.bucket, Prefix=full_prefix)
            if token:
                kw["ContinuationToken"] = token
            r = self._s3.list_objects_v2(**kw)
            for it in r.get("Contents", []):
                out.append(it["Key"][strip:])
            if not r.get("IsTruncated"):
                break
            token = r.get("NextContinuationToken")
        return sorted(out)

    def put(self, key: str, data: bytes) -> None:
        self._s3.put_object(Bucket=self.bucket, Key=self._full(key), Body=data)

    def put_if_absent(self, key: str, data: bytes) -> None:
        try:
            # If-None-Match: * → create-only. Supported by S3 (2024+), MinIO, R2, B2.
            self._s3.put_object(Bucket=self.bucket, Key=self._full(key),
                                Body=data, IfNoneMatch="*")
        except self._ClientError as e:
            code = e.response.get("Error", {}).get("Code")
            if code in ("PreconditionFailed", "412", "ConditionalRequestConflict"):
                raise PreconditionFailed(key)
            raise

    def delete(self, key: str) -> None:
        self._s3.delete_object(Bucket=self.bucket, Key=self._full(key))

    def __repr__(self) -> str:
        ep = getattr(getattr(self._s3, "meta", None), "endpoint_url", "aws")
        return f"S3ObjectStore({self.bucket}/{self.prefix} @ {ep})"


# ──────────────────────────────── url → store ────────────────────────────────
def open_store(url: str) -> ObjectStore:
    """Open a store from a URL.

      file:///var/lib/strata          → LocalObjectStore
      /var/lib/strata                 → LocalObjectStore (bare path)
      s3://my-bucket/prefix           → S3ObjectStore (AWS)
      s3://my-bucket/prefix?endpoint=http://localhost:9000&region=us-east-1
                                      → S3-compatible (MinIO/R2/B2)

    S3 credentials come from the standard AWS chain (env, shared config, IAM) or
    the `access_key`/`secret_key` query params.
    """
    u = urlparse(url)
    if u.scheme in ("", "file"):
        return LocalObjectStore(u.path or url)
    if u.scheme == "s3":
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        return S3ObjectStore(
            bucket=u.netloc, prefix=u.path.lstrip("/"),
            endpoint_url=q.get("endpoint"), region=q.get("region"),
            access_key=q.get("access_key"), secret_key=q.get("secret_key"))
    raise ValueError(f"unsupported store url scheme: {u.scheme!r}")


@dataclass
class _MemBlob:
    data: bytes


class MemoryObjectStore(ObjectStore):
    """In-RAM store — fast, ephemeral; used by tests and `:memory:` mode.

    A lock makes `put_if_absent` an atomic compare-and-swap, emulating the
    server-side atomicity that real S3 `If-None-Match` provides — without it,
    concurrent commits could both pass the existence check and lose a write.
    """

    def __init__(self):
        import threading
        self._d: dict[str, bytes] = {}
        self._lock = threading.Lock()

    def get(self, key: str) -> Optional[bytes]:
        return self._d.get(key)

    def exists(self, key: str) -> bool:
        return key in self._d

    def list(self, prefix: str) -> list[str]:
        with self._lock:
            return sorted(k for k in self._d if k.startswith(prefix))

    def put(self, key: str, data: bytes) -> None:
        self._d[key] = bytes(data)

    def put_if_absent(self, key: str, data: bytes) -> None:
        with self._lock:
            if key in self._d:
                raise PreconditionFailed(key)
            self._d[key] = bytes(data)

    def delete(self, key: str) -> None:
        self._d.pop(key, None)

    def __repr__(self) -> str:
        return f"MemoryObjectStore({len(self._d)} objects)"
