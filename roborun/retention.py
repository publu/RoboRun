"""Run retention / GC (RECORD_EVERYTHING_SPEC).

Always-on recording will fill disk. This caps `~/.roborun/runs/` by total bytes,
evicting the **oldest MCAP bodies** first, but only ones that are safe to drop:
  * the run is sealed (a `.seal` exists), AND
  * the run is uploaded to R2 (a `<run>.uploaded` marker), unless `require_uploaded`
    is False (offline-only deployments).
The small `.seal`/`.seal.tsr`/`.chain.jsonl` provenance files are **kept** — the
record that the run existed and its proof survive even when the heavy MCAP is gone.

No daemon here; call `enforce()` from a timer. Pure stdlib, no infra.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from roborun.events import runs_root

DEFAULT_MAX_GB = float(os.environ.get("ROBORUN_RUNS_MAX_GB", "20"))


def _mcap_bodies(root: Path) -> list[Path]:
    return sorted(root.glob("*/*.mcap"), key=lambda p: p.stat().st_mtime)


def _is_sealed(mcap: Path) -> bool:
    return mcap.with_suffix(".seal").exists()


def _is_uploaded(mcap: Path) -> bool:
    return mcap.with_suffix(".uploaded").exists()


def dir_bytes(root: Path) -> int:
    return sum(p.stat().st_size for p in root.rglob("*") if p.is_file())


def enforce(root: Path | None = None, max_gb: float | None = None,
            require_uploaded: bool = True) -> dict[str, Any]:
    """Evict oldest evictable MCAP bodies until total ≤ max_gb. Returns a report.
    Only the `.mcap` body is removed; the seal/tsr/chain provenance is retained."""
    root = root or runs_root()
    max_bytes = int((DEFAULT_MAX_GB if max_gb is None else max_gb) * (1 << 30))
    if not root.exists():
        return {"total_bytes": 0, "evicted": [], "kept_over_cap": False}
    total = dir_bytes(root)
    evicted: list[str] = []
    if total <= max_bytes:
        return {"total_bytes": total, "evicted": evicted, "kept_over_cap": False}
    for mcap in _mcap_bodies(root):  # oldest first
        if total <= max_bytes:
            break
        if not _is_sealed(mcap):
            continue  # never drop an unsealed (possibly in-flight) run
        if require_uploaded and not _is_uploaded(mcap):
            continue  # never drop a run not yet backed up
        size = mcap.stat().st_size
        mcap.unlink()
        total -= size
        evicted.append(mcap.name)
    return {"total_bytes": total, "evicted": evicted,
            "kept_over_cap": total > max_bytes}
