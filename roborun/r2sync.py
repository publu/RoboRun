"""R2/S3 object store sync — the cold tier and the fleet's shared blackboard.

One durable store, used three ways (spec §3):
    runs/<robot_id>/<run_id>.mcap|.seal|.seal.tsr|.chain.jsonl   cold archive
    index/<robot_id>/<date>.parquet                              shared fleet index
    beacons/<robot_id>/<ts>.json                                 live-ish signed beacons

Configure with env vars (works with Cloudflare R2, AWS S3, MinIO):
    ROBORUN_R2_BUCKET     bucket name (sync is disabled when unset)
    ROBORUN_R2_ENDPOINT   endpoint URL (R2: https://<account>.r2.cloudflarestorage.com)
    ROBORUN_R2_PREFIX     optional key prefix
    + standard AWS_ACCESS_KEY_ID / AWS_SECRET_ACCESS_KEY

Everything is best-effort and append-only; an offline robot just syncs later.
"""
from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Any

log = logging.getLogger("roborun.r2")


class R2Store:

    def __init__(self, bucket: str, endpoint: str | None = None, prefix: str = "") -> None:
        import boto3
        kwargs: dict[str, Any] = {}
        if endpoint:
            kwargs["endpoint_url"] = endpoint
        self._s3 = boto3.client("s3", **kwargs)
        self.bucket = bucket
        self.prefix = prefix.strip("/") + "/" if prefix.strip("/") else ""

    @classmethod
    def from_env(cls) -> "R2Store | None":
        bucket = os.environ.get("ROBORUN_R2_BUCKET")
        if not bucket:
            return None
        try:
            return cls(bucket,
                       endpoint=os.environ.get("ROBORUN_R2_ENDPOINT"),
                       prefix=os.environ.get("ROBORUN_R2_PREFIX", ""))
        except Exception as exc:
            log.warning("R2 disabled: %s", exc)
            return None

    def _key(self, key: str) -> str:
        return self.prefix + key.lstrip("/")

    # ── files ────────────────────────────────────────────────────────────

    def upload_file(self, path: str | Path, key: str) -> None:
        self._s3.upload_file(str(path), self.bucket, self._key(key))

    def download_file(self, key: str, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._s3.download_file(self.bucket, self._key(key), str(path))

    def put_json(self, key: str, obj: dict) -> None:
        self._s3.put_object(Bucket=self.bucket, Key=self._key(key),
                            Body=json.dumps(obj, separators=(",", ":")).encode(),
                            ContentType="application/json")

    def get_json(self, key: str) -> dict | None:
        try:
            resp = self._s3.get_object(Bucket=self.bucket, Key=self._key(key))
            return json.loads(resp["Body"].read())
        except Exception:
            return None

    def list_keys(self, prefix: str, limit: int = 1000) -> list[str]:
        out: list[str] = []
        paginator = self._s3.get_paginator("list_objects_v2")
        for page in paginator.paginate(Bucket=self.bucket, Prefix=self._key(prefix)):
            for item in page.get("Contents", []):
                key = item["Key"]
                if self.prefix and key.startswith(self.prefix):
                    key = key[len(self.prefix):]
                out.append(key)
                if len(out) >= limit:
                    return out
        return out

    def sync_down(self, prefix: str, dest: Path) -> int:
        """Mirror a prefix into a local directory (skip already-present sizes)."""
        n = 0
        paginator = self._s3.get_paginator("list_objects_v2")
        for page in paginator.paginate(Bucket=self.bucket, Prefix=self._key(prefix)):
            for item in page.get("Contents", []):
                key = item["Key"]
                rel = key[len(self.prefix):] if self.prefix else key
                rel = rel[len(prefix):] if rel.startswith(prefix) else rel
                local = dest / rel
                if local.exists() and local.stat().st_size == item["Size"]:
                    continue
                local.parent.mkdir(parents=True, exist_ok=True)
                self._s3.download_file(self.bucket, key, str(local))
                n += 1
        return n

    # ── run artifacts (spec §3.1 layout) ─────────────────────────────────

    def upload_run(self, mcap_path: str | Path, robot_id: str,
                   background: bool = True) -> dict[str, Any]:
        """Upload <run>.mcap + .seal + .seal.tsr + .chain.jsonl under runs/<robot>/."""
        mcap_path = Path(mcap_path)
        base = str(mcap_path.with_suffix(""))
        artifacts = [p for p in (mcap_path,
                                 Path(base + ".seal"),
                                 Path(base + ".seal.tsr"),
                                 Path(base + ".chain.jsonl")) if p.exists()]

        def _do() -> None:
            for p in artifacts:
                try:
                    self.upload_file(p, f"runs/{robot_id}/{p.name}")
                except Exception as exc:
                    log.warning("R2 upload failed for %s: %s", p.name, exc)

        if background:
            threading.Thread(target=_do, daemon=True, name="R2RunUpload").start()
        else:
            _do()
        return {"ok": True, "bucket": self.bucket,
                "keys": [f"runs/{robot_id}/{p.name}" for p in artifacts],
                "background": background}
