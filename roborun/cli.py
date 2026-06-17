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
