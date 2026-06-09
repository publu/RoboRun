"""Run routes — seal, verify, replay, and tamper the chained journal.

The event bus journals every event to disk as it happens (hash-chained).
Sealing closes the current journal, Merkle-seals + signs it, and starts
a fresh journal whose manifest links back to the sealed run.

POST /api/run/seal     close + seal the live journal
POST /api/run/verify   verify a run (default: latest sealed)
POST /api/run/tamper   flip one byte of one event (demo)
GET  /api/run/list     list recorded runs
GET  /api/run/events   events of a run, for replay
"""
from __future__ import annotations

from pathlib import Path

from roborun.routes import get, post, send_json
from roborun import integrity
from roborun import events as bus


def _runs() -> list[Path]:
    root = bus.runs_root()
    if not root.exists():
        return []
    return sorted([p for p in root.iterdir() if (p / "run.jsonl").exists()])


def _latest_sealed() -> Path | None:
    sealed = [p for p in _runs() if (p / "run.seal").exists()]
    return sealed[-1] if sealed else None


def _resolve(payload: dict) -> Path | None:
    name = str(payload.get("run", "")).strip()
    if name:
        p = bus.runs_root() / name
        return p if (p / "run.jsonl").exists() else None
    return _latest_sealed()


@post("/api/run/seal")
def seal(h, payload):
    run_dir = bus.close_journal()
    if run_dir is None:
        send_json(h, 200, {"ok": False, "error": "no events recorded yet"})
        return
    result = integrity.seal_run(run_dir)
    bus.record_sealed(run_dir.name, result["merkle_root"])
    bus.emit("system", "integrity",
             f"RUN SEALED — {result['event_count']} events",
             {"run": run_dir.name, "merkle_root": result["merkle_root"],
              "signed": result["signed"]})
    send_json(h, 200, {**result, "run": run_dir.name,
                       "next_run": (bus.current_run() or {}).get("run")})


@post("/api/run/verify")
def verify(h, payload):
    run_dir = _resolve(payload)
    if run_dir is None:
        send_json(h, 200, {"ok": False, "error": "no sealed runs found"})
        return
    result = integrity.verify_run(run_dir)
    if result.get("verified"):
        chain = "chain intact · " if result.get("chain_intact") else ""
        bus.emit("system", "integrity",
                 f"VERIFIED — {result['event_count']} events · {chain}untampered",
                 {"run": run_dir.name, "merkle_root": result["merkle_root"]})
    else:
        bus.emit("system", "integrity",
                 f"VERIFICATION FAILED — {result.get('reason', 'unknown')}",
                 {"run": run_dir.name})
    send_json(h, 200, {**result, "run": run_dir.name})


@post("/api/run/tamper")
def tamper(h, payload):
    run_dir = _resolve(payload)
    if run_dir is None:
        send_json(h, 200, {"ok": False, "error": "no sealed runs to tamper — seal first"})
        return
    result = integrity.tamper_run(run_dir, payload.get("event"))
    if result.get("ok"):
        bus.emit("system", "integrity",
                 f"run tampered — event {result['tampered_event']:04d} modified",
                 {"run": run_dir.name})
    send_json(h, 200, {**result, "run": run_dir.name})


@get("/api/run/events")
def run_events(h):
    """Events of a recorded run, for replay. ?run=<name>&limit=N"""
    from urllib.parse import parse_qs, urlparse
    q = parse_qs(urlparse(h.path).query)
    name = (q.get("run") or [""])[0]
    limit = int((q.get("limit") or ["2000"])[0])
    run_dir = bus.runs_root() / name if name else _latest_sealed()
    if run_dir is None or not (run_dir / "run.jsonl").exists():
        send_json(h, 200, {"ok": False, "error": "run not found"})
        return
    import json as _json
    lines = [ln for ln in (run_dir / "run.jsonl").read_text().splitlines() if ln.strip()]
    events = [_json.loads(ln) for ln in lines[:limit]]
    send_json(h, 200, {"ok": True, "run": run_dir.name, "events": events,
                       "sealed": (run_dir / "run.seal").exists()})


@get("/api/run/list")
def list_runs(h):
    live = bus.current_run()
    runs = []
    for p in _runs():
        entry = {
            "run": p.name,
            "sealed": (p / "run.seal").exists(),
            "events": sum(1 for ln in (p / "run.jsonl").read_text().splitlines() if ln.strip()),
        }
        if live and p.name == live["run"]:
            entry["recording"] = True
        runs.append(entry)
    send_json(h, 200, {"ok": True, "runs": runs})
