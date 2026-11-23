"""Run retention GC (RECORD_EVERYTHING_SPEC): cap by bytes, evict safely."""
from __future__ import annotations

from roborun import retention


def _run(root, robot, name, mb, sealed=True, uploaded=True):
    d = root / robot
    d.mkdir(parents=True, exist_ok=True)
    (d / f"{name}.mcap").write_bytes(b"\0" * (mb * 1024 * 1024))
    if sealed:
        (d / f"{name}.seal").write_text("{}")
    if uploaded:
        (d / f"{name}.uploaded").write_text("")


def test_under_cap_noop(tmp_path):
    _run(tmp_path, "r", "a", 1)
    out = retention.enforce(tmp_path, max_gb=1.0)
    assert out["evicted"] == []


def test_evicts_oldest_sealed_and_uploaded(tmp_path):
    import os, time
    _run(tmp_path, "r", "old", 2)
    _run(tmp_path, "r", "new", 2)
    # make 'old' older
    os.utime(tmp_path / "r" / "old.mcap", (1, 1))
    out = retention.enforce(tmp_path, max_gb=3 / 1024)  # 3 MB cap
    assert "old.mcap" in out["evicted"]
    assert not (tmp_path / "r" / "old.mcap").exists()
    # provenance kept
    assert (tmp_path / "r" / "old.seal").exists()
    assert (tmp_path / "r" / "new.mcap").exists()


def test_never_evicts_unsealed_or_unuploaded(tmp_path):
    _run(tmp_path, "r", "unsealed", 5, sealed=False)
    _run(tmp_path, "r", "unsynced", 5, uploaded=False)
    out = retention.enforce(tmp_path, max_gb=1 / 1024)  # 1 MB cap, way over
    assert out["evicted"] == []
    assert out["kept_over_cap"] is True  # honestly reports it couldn't get under


def test_status_reports_footprint(tmp_path):
    from roborun import retention
    _run(tmp_path, "r", "a", 2, sealed=True, uploaded=True)
    _run(tmp_path, "r", "b", 3, sealed=True, uploaded=False)
    st = retention.status(tmp_path, max_gb=1.0)
    assert st["runs"] == 2 and st["sealed"] == 2 and st["uploaded"] == 1
    assert st["used_gb"] > 0 and st["cap_gb"] == 1.0
    # only the sealed+uploaded run is evictable
    assert st["evictable_bytes"] == 2 * 1024 * 1024
