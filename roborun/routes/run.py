"""Run routes — snapshot the live event timeline into a sealed, verifiable run.

POST /api/run/seal     snapshot current events → runs/<id>/, seal it
POST /api/run/verify   verify a run (default: latest)
POST /api/run/tamper   flip one byte of one event (demo)
GET  /api/run/list     list recorded runs
"""
from __future__ import annotations

import time
from pathlib import Path

from roborun.routes import get, post, send_json
from roborun import integrity
from roborun.events import emit, recent

RUNS_ROOT = Path(__file__).resolve().parent.parent.parent / ".roborun" / "runs"


def _latest_run() -> Path | None:
    if not RUNS_ROOT.exists():
        return None
    runs = sorted([p for p in RUNS_ROOT.iterdir() if (p / "run.jsonl").exists()])
    return runs[-1] if runs else None


def _resolve(payload: dict) -> Path | None:
    name = str(payload.get("run", "")).strip()
    if name:
        p = RUNS_ROOT / name
        return p if (p / "run.jsonl").exists() else None
    return _latest_run()


@post("/api/run/seal")
def seal(h, payload):
    events = recent(500)
    if not events:
        send_json(h, 200, {"ok": False, "error": "no events recorded yet"})
        return
    run_id = time.strftime("run_%Y%m%d_%H%M%S", time.gmtime())
    run_dir = RUNS_ROOT / run_id
    integrity.snapshot_run(events, run_dir, manifest={"source": "event-bus"})
    result = integrity.seal_run(run_dir)
    emit("system", "integrity", f"RUN SEALED — {result['event_count']} events",
         {"run": run_id, "merkle_root": result["merkle_root"], "signed": result["signed"]})
    send_json(h, 200, {**result, "run": run_id})


@post("/api/run/verify")
def verify(h, payload):
    run_dir = _resolve(payload)
    if run_dir is None:
        send_json(h, 200, {"ok": False, "error": "no sealed runs found"})
        return
    result = integrity.verify_run(run_dir)
    if result.get("verified"):
        emit("system", "integrity",
             f"VERIFIED — {result['event_count']} events intact",
             {"run": run_dir.name, "merkle_root": result["merkle_root"]})
    else:
        emit("system", "integrity",
             f"VERIFICATION FAILED — {result.get('reason', 'unknown')}",
             {"run": run_dir.name})
    send_json(h, 200, {**result, "run": run_dir.name})


@post("/api/run/tamper")
def tamper(h, payload):
    run_dir = _resolve(payload)
    if run_dir is None:
        send_json(h, 200, {"ok": False, "error": "no runs found"})
        return
    result = integrity.tamper_run(run_dir, payload.get("event"))
    if result.get("ok"):
        emit("system", "integrity",
             f"run tampered — event {result['tampered_event']:04d} modified",
             {"run": run_dir.name})
    send_json(h, 200, {**result, "run": run_dir.name})


@get("/api/run/list")
def list_runs(h):
    runs = []
    if RUNS_ROOT.exists():
        for p in sorted(RUNS_ROOT.iterdir()):
            if not (p / "run.jsonl").exists():
                continue
            runs.append({
                "run": p.name,
                "sealed": (p / "run.seal").exists(),
                "events": sum(1 for ln in (p / "run.jsonl").read_text().splitlines() if ln.strip()),
            })
    send_json(h, 200, {"ok": True, "runs": runs})
