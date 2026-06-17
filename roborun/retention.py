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


def status(root: Path | None = None, max_gb: float | None = None) -> dict[str, Any]:
    """Storage footprint + retention picture for fleet-cost awareness (robots make
    30–100 GB/shift; local-first + GC keeps cost sane). Read-only; evicts nothing."""
    root = root or runs_root()
    cap_gb = DEFAULT_MAX_GB if max_gb is None else max_gb
    if not root.exists():
        return {"used_bytes": 0, "used_gb": 0.0, "cap_gb": cap_gb, "pct": 0.0,
                "runs": 0, "sealed": 0, "uploaded": 0, "evictable_bytes": 0}
    bodies = _mcap_bodies(root)
    used = dir_bytes(root)
    sealed = sum(1 for m in bodies if _is_sealed(m))
    uploaded = sum(1 for m in bodies if _is_uploaded(m))
    evictable = sum(m.stat().st_size for m in bodies
                    if _is_sealed(m) and _is_uploaded(m))
    return {"used_bytes": used, "used_gb": round(used / (1 << 30), 3),
            "cap_gb": cap_gb, "pct": round(used / (cap_gb * (1 << 30)) * 100, 1) if cap_gb else 0.0,
            "runs": len(bodies), "sealed": sealed, "uploaded": uploaded,
            "evictable_bytes": evictable,
            "evictable_gb": round(evictable / (1 << 30), 3)}


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


# ── mode-aware retention across projects (platform spec 08) ────────────────
# scratch is throwaway → small cap, evict sealed even if not uploaded.
# production is precious → big cap, evict only sealed + uploaded.
MODE_CAPS = {
    "scratch": float(os.environ.get("ROBORUN_SCRATCH_MAX_GB", "2")),
    "test": float(os.environ.get("ROBORUN_TEST_MAX_GB", "20")),
    "production": float(os.environ.get("ROBORUN_PROD_MAX_GB", "100")),
}


def enforce_all() -> dict[str, Any]:
    """Walk every project/environment and apply its mode's retention policy, so a
    real-robot production env never competes with rapier scratch for space."""
    from roborun import projects, environments
    root = projects.projects_root()
    reports: dict[str, Any] = {}
    if not root.exists():
        return {"projects": reports}
    for pdir in sorted(root.iterdir()):
        if not pdir.is_dir():
            continue
        for edir in sorted(pdir.iterdir()):
            runs = edir / "runs"
            if not runs.is_dir():
                continue
            meta = environments.get(pdir.name, edir.name) or {}
            mode = meta.get("mode", "scratch")
            cap = MODE_CAPS.get(mode, 20.0)
            rep = enforce(root=runs, max_gb=cap,
                          require_uploaded=(mode != "scratch"))
            reports[f"{pdir.name}/{edir.name}"] = {
                "mode": mode, "cap_gb": cap, **rep}
    return {"projects": reports}
