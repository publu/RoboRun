"""Local-runner CLI verbs: `roborun search …` and `roborun scenarios …`.

For operators driving the runner locally (no browser needed): query the all-time
index and run/inspect scenarios from the terminal. Lean — composes the same
functions the UI uses.
"""
from __future__ import annotations

import time


def search_cli(argv: list[str]) -> int:
    import argparse
    p = argparse.ArgumentParser(prog="roborun search",
                                description="Find anything/anyone across all recorded runs.")
    p.add_argument("query", nargs="?", default="", help="what/who to find")
    p.add_argument("--by", default="label", choices=["label", "clip", "near", "time"])
    p.add_argument("--k", type=int, default=15)
    p.add_argument("--since", type=float, help="unix seconds")
    p.add_argument("--robot", help="filter to one robot")
    a = p.parse_args(argv)
    from roborun.spatial_memory import SpatialMemoryStore
    from roborun.session import search
    rows = search(SpatialMemoryStore(), a.query, by=a.by, k=a.k,
                  since=a.since, robot_id=a.robot)
    if not rows:
        print("no hits"); return 0
    print(f"{len(rows)} hit(s) across all history:")
    for r in rows:
        labels = ",".join(sorted({d.get("label") for d in (r.get("detections") or [])})) or "—"
        when = time.strftime("%Y-%m-%d %H:%M", time.localtime(r.get("ts", 0)))
        where = f"({r.get('x')},{r.get('y')})" if r.get("x") is not None else "—"
        print(f"  {when}  {labels:18} {where:14} {r.get('source','?')}/{r.get('robot_id','')}")
    return 0


def _base_url() -> str:
    import os
    return f"http://127.0.0.1:{os.environ.get('ROBORUN_PORT', '8765')}"


def ask_cli(argv: list[str]) -> int:
    """Send a natural-language instruction to the running robot's agent (it sees
    the live camera and can move/act). `roborun ask "patrol the lobby"`.
    Needs `roborun` running in another terminal."""
    import json
    import urllib.request
    msg = " ".join(argv).strip()
    if not msg:
        print('usage: roborun ask "what you want the robot to do"'); return 2
    req = urllib.request.Request(_base_url() + "/api/agent/chat",
                                 data=json.dumps({"message": msg}).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        resp = urllib.request.urlopen(req, timeout=120)
    except Exception as exc:
        print(f"can't reach RoboRun ({exc}). Start it with `roborun` first."); return 1
    for raw in resp:
        line = raw.decode().strip()
        if not line.startswith("data:"):
            continue
        try:
            ev = json.loads(line[5:].strip())
        except Exception:
            continue
        if ev.get("type") == "text":
            print(ev.get("text", ""), end="", flush=True)
        elif ev.get("type") == "tool_use":
            print(f"\n  · {ev.get('tool_name')}({json.dumps(ev.get('tool_input', {}))[:60]})", flush=True)
        elif ev.get("type") == "error":
            print(f"\n[agent error] {ev.get('error')}"); return 1
        elif ev.get("type") == "done":
            print(); return 0
    print(); return 0


def status_cli(argv: list[str]) -> int:
    """Quick health: is the server up, what's connected, how much is recorded."""
    import json
    import urllib.request
    up = False
    try:
        with urllib.request.urlopen(_base_url() + "/api/agent/status", timeout=3) as r:
            json.loads(r.read()); up = True
    except Exception:
        pass
    print(f"server:  {'up at ' + _base_url() if up else 'not running (start with `roborun`)'}")
    try:
        from roborun.spatial_memory import SpatialMemoryStore
        from roborun.recorder import list_runs
        from roborun.retention import status as st
        s = SpatialMemoryStore().stats()
        runs = list_runs()
        ss = st()
        print(f"data:    {s.get('total', 0)} things seen across {len(runs)} runs "
              f"({ss.get('used_gb', 0)}/{ss.get('cap_gb', 0)} GB)")
        from roborun.connect import saved_robot
        rb = saved_robot()
        print(f"robot:   {rb['host'] + ' (' + rb.get('type', '?') + ')' if rb else 'none connected — `roborun connect <ip>`'}")
    except Exception as exc:
        print(f"data:    (unavailable: {exc})")
    return 0


def demo_cli(argv: list[str]) -> int:
    """Seed a populated demo so a fresh install shows a live UI immediately:
    a recorded synthetic run (camera+detections+pose, indexed + searchable) and a
    scored scenario suite. `roborun demo` then open the dashboards."""
    import time
    import numpy as np
    from roborun.recorder import RunRecorder
    from roborun.events import runs_root
    from roborun.observations import StreamingExtractor
    from roborun.spatial_memory import SpatialMemoryStore
    from roborun.synthetic_camera import SyntheticCamera
    from roborun.session import PerceptionSession
    import roborun.demo_scenarios  # noqa
    from roborun.scenario_defs import run_suite

    store = SpatialMemoryStore()
    labels = ["person", "forklift", "pallet"]
    for lab in labels:
        rec = RunRecorder(robot_id=f"demo-{lab}", root=runs_root(), checkpoint_interval=0.05)
        rec.extractor = StreamingExtractor(store, robot_id=f"demo-{lab}",
                                           run_id=rec.run_id, source="production")
        cam = SyntheticCamera(label=lab)
        sess = PerceptionSession(cam, store, mode="production", source_id=f"{lab}-cam",
                                 recorder=rec, embed_fn=lambda f: f.reshape(-1, 3).mean(0).astype(np.float32),
                                 hz=30)
        cam.start()
        try:
            for _ in range(10):
                sess.tick(); time.sleep(0.01)
        finally:
            cam.stop()
        rec.close(do_anchor=False)
    run_suite("demo")
    print("Demo seeded: 3 runs recorded + indexed, demo suite scored.")
    print("Open http://localhost:8765  → ▤ VIEWS → Search / Analytics / Scenarios")
    return 0


def dataset_cli(argv: list[str]) -> int:
    import argparse
    p = argparse.ArgumentParser(prog="roborun dataset",
                                description="Curate a labeled dataset from a search.")
    p.add_argument("query")
    p.add_argument("out", help="output directory")
    p.add_argument("--by", default="label", choices=["label", "clip", "near", "time"])
    p.add_argument("--k", type=int, default=500)
    p.add_argument("--since", type=float)
    a = p.parse_args(argv)
    from roborun.spatial_memory import SpatialMemoryStore
    from roborun.session import export_dataset
    r = export_dataset(SpatialMemoryStore(), a.query, a.out, by=a.by, k=a.k, since=a.since)
    print(f"wrote {r['count']} images + labels.jsonl → {r['dir']}")
    return 0


def scenarios_cli(argv: list[str]) -> int:
    import roborun.demo_scenarios  # noqa: F401  (register built-ins)
    from roborun.scenario import list_suites
    from roborun.scenario_defs import list_defs, run_scenario, run_suite
    if argv and argv[0] == "run" and len(argv) > 1:
        rec = run_scenario(argv[1])
        print(f"{argv[1]}: {rec['outcome'].upper()}"
              + (f" — {rec.get('reason')}" if rec.get("reason") else ""))
        return 0 if rec["outcome"] == "passed" else 1
    if argv and argv[0] == "suite" and len(argv) > 1:
        s = run_suite(argv[1])
        print(f"{argv[1]}: {int(s['pass_rate']*100)}% ({s['passed']}/{s['runs']})")
        return 0
    print("Runnable scenarios:")
    for d in list_defs():
        print(f"  {d['name']:20} suite={d['suite'] or '-'}  {d.get('doc','')[:50]}")
    suites = list_suites()
    if suites:
        print("\nSuites (history):")
        for s in suites:
            print(f"  {s['suite']:20} {int(s['pass_rate']*100):>3}%  ({s['passed']}/{s['runs']})")
    print("\n  roborun scenarios run <name>   ·   roborun scenarios suite <name>")
    return 0
