"""Run routes — one black box: the MCAP recorder (spec §2).

Record start/stop, three-state verify, an anchor badge for the flight
deck, verified clip export, and a tamper demo. The event bus's journal
(run.jsonl) persists the live timeline and serves replay; its integrity
artifact is the MCAP run it feeds into, not a second seal.

POST /api/run/record/start   open an MCAP run (webcam + event bus feed it)
POST /api/run/record/stop    seal + timestamp-anchor + index the MCAP run
GET  /api/run/mcap           list MCAP runs with seal/anchor status
POST /api/run/mcap/verify    three-state verify {run, robot_id} (default newest)
POST /api/run/mcap/clip      verified clip export {run, robot_id, start_ts, end_ts}
POST /api/run/mcap/tamper    flip one byte mid-file (demo)
GET  /api/run/badge          latest run's verify state, for the live badge
GET  /api/run/list           list journal runs (timeline replay)
GET  /api/run/events         events of a journal run, for replay
"""
from __future__ import annotations

from pathlib import Path

from roborun.routes import get, post, send_json
from roborun import events as bus
from roborun import recorder as rec_mod


def _runs() -> list[Path]:
    root = bus.runs_root()
    if not root.exists():
        return []
    return sorted([p for p in root.iterdir() if (p / "run.jsonl").exists()])


@get("/api/run/events")
def run_events(h):
    """Events of a recorded run, for replay. ?run=<name>&limit=N"""
    from urllib.parse import parse_qs, urlparse
    q = parse_qs(urlparse(h.path).query)
    name = (q.get("run") or [""])[0]
    limit = int((q.get("limit") or ["2000"])[0])
    all_runs = _runs()
    run_dir = bus.runs_root() / name if name else (all_runs[-1] if all_runs else None)
    if run_dir is None or not (run_dir / "run.jsonl").exists():
        send_json(h, 200, {"ok": False, "error": "run not found"})
        return
    import json as _json
    lines = [ln for ln in (run_dir / "run.jsonl").read_text().splitlines() if ln.strip()]
    events = [_json.loads(ln) for ln in lines[:limit]]
    send_json(h, 200, {"ok": True, "run": run_dir.name, "events": events})


# ── MCAP runs (spec §2) ───────────────────────────────────────────────────

def _mcap_path(payload: dict) -> Path | None:
    """Resolve {run, robot_id} to an mcap path; default: newest run."""
    name = str(payload.get("run", "")).strip()
    robot = str(payload.get("robot_id", "")).strip()
    if name:
        if robot:
            p = rec_mod.runs_root() / robot / f"{name}.mcap"
            return p if p.exists() else None
        hits = list(rec_mod.runs_root().glob(f"*/{name}.mcap"))
        return hits[0] if hits else None
    runs = rec_mod.list_runs()
    return Path(runs[0]["mcap"]) if runs else None


@post("/api/run/record/start")
def record_start(h, payload):
    rec = rec_mod.start_recording(robot_id=payload.get("robot_id", "local"))
    bus.emit("system", "recorder", f"RECORDING · {rec.run_id}",
             {"run": rec.run_id, "mcap": str(rec.mcap_path)})
    send_json(h, 200, {"ok": True, **rec.status()})


@post("/api/run/record/stop")
def record_stop(h, payload):
    seal = rec_mod.stop_recording(do_anchor=not payload.get("no_anchor", False))
    if seal is None:
        send_json(h, 200, {"ok": False, "error": "nothing is recording"})
        return
    mcap_path = rec_mod.runs_root() / seal["robot_id"] / f"{seal['run']}.mcap"
    indexed = None
    try:
        from roborun.observations import extract_run
        from roborun.routes._singletons import get_memory
        indexed = extract_run(mcap_path, get_memory(), robot_id=seal["robot_id"])
    except Exception as exc:
        indexed = {"ok": False, "error": str(exc)}
    uploaded = None
    try:
        from roborun.r2sync import R2Store
        r2 = R2Store.from_env()
        if r2 is not None:
            uploaded = r2.upload_run(mcap_path, seal["robot_id"])
    except Exception:
        pass
    anchor_state = seal.get("anchor", {}).get("status", "unanchored")
    bus.emit("system", "recorder",
             f"RUN SEALED · {seal['run']} · merkle root · anchor {anchor_state}",
             {"run": seal["run"], "merkle_root": seal["merkle_root"],
              "anchor": anchor_state})
    send_json(h, 200, {"ok": True, "seal": seal, "indexed": indexed,
                       "uploaded": uploaded})


@get("/api/run/mcap")
def mcap_list(h):
    live = rec_mod.active_recorder()
    send_json(h, 200, {"ok": True,
                       "recording": live.status() if live else None,
                       "runs": rec_mod.list_runs()})


@post("/api/run/mcap/verify")
def mcap_verify(h, payload):
    p = _mcap_path(payload)
    if p is None:
        send_json(h, 200, {"ok": False, "error": "no mcap runs found"})
        return
    live = rec_mod.active_recorder()
    if live is not None and Path(live.mcap_path) == p:
        # A live run always has un-chained trailing bytes — that's not
        # tampering, it's a tape still rolling. Don't scare anyone.
        send_json(h, 200, {"ok": True, "run": p.stem, "state": "recording",
                           "reason": "still recording — press M to stop and seal, then verify"})
        return
    result = rec_mod.verify_mcap_run(p)
    if result["state"] == "broken":
        bus.emit("system", "integrity",
                 f"VERIFICATION FAILED · {result.get('reason', '')}", {"run": p.stem})
    else:
        bus.emit("system", "integrity",
                 f"VERIFIED · {p.stem} · {result['state'].replace('_', ' ')}",
                 {"run": p.stem, "merkle_root": result.get("merkle_root")})
    send_json(h, 200, {"ok": True, "run": p.stem, **result})


@post("/api/run/mcap/clip")
def mcap_clip(h, payload):
    p = _mcap_path(payload)
    if p is None:
        send_json(h, 200, {"ok": False, "error": "no mcap runs found"})
        return
    try:
        start = float(payload["start_ts"])
        end = float(payload["end_ts"])
    except (KeyError, ValueError):
        send_json(h, 200, {"ok": False, "error": "start_ts and end_ts (unix seconds) required"})
        return
    result = rec_mod.export_clip(p, start, end)
    if result.get("ok"):
        bus.emit("system", "recorder",
                 f"CLIP EXPORTED · {result['messages']} messages · proof attached",
                 {"clip": result["clip"], "proof": result["proof"]})
    send_json(h, 200, result)


@post("/api/run/mcap/tamper")
def mcap_tamper(h, payload):
    """Demo: flip one byte in the middle of the newest sealed MCAP."""
    p = _mcap_path(payload)
    if p is None:
        send_json(h, 200, {"ok": False, "error": "no mcap runs found"})
        return
    data = bytearray(p.read_bytes())
    if not data:
        send_json(h, 200, {"ok": False, "error": "empty mcap"})
        return
    idx = len(data) // 2
    data[idx] ^= 1
    p.write_bytes(bytes(data))
    bus.emit("system", "integrity", f"run tampered · byte {idx} flipped",
             {"run": p.stem})
    send_json(h, 200, {"ok": True, "run": p.stem, "byte": idx,
                       "note": "one bit flipped mid-file; verify will localize it"})


@get("/api/run/badge")
def badge(h):
    """Verify state of the newest run — the flight deck's live badge."""
    live = rec_mod.active_recorder()
    runs = rec_mod.list_runs()
    if live is not None:
        send_json(h, 200, {"ok": True, "badge": "recording", **live.status()})
        return
    if not runs:
        send_json(h, 200, {"ok": True, "badge": "none"})
        return
    result = rec_mod.verify_mcap_run(runs[0]["mcap"])
    badge_word = {"verified_anchored": "anchored",
                  "consistent_unanchored": "unanchored",
                  "broken": "broken"}[result["state"]]
    send_json(h, 200, {"ok": True, "badge": badge_word, "run": runs[0]["run"],
                       "state": result["state"], "reason": result.get("reason"),
                       "anchor": result.get("anchor"),
                       "sealed_at": result.get("sealed_at")})


@get("/api/run/list")
def list_runs(h):
    live = bus.current_run()
    runs = []
    for p in _runs():
        entry = {
            "run": p.name,
            "events": sum(1 for ln in (p / "run.jsonl").read_text().splitlines() if ln.strip()),
        }
        if live and p.name == live["run"]:
            entry["recording"] = True
        runs.append(entry)
    send_json(h, 200, {"ok": True, "runs": runs})
