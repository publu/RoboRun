"""Headless episode runner — drive a Fleet under a strategy until the field is
mapped (or time runs out) and report the coverage / overlap / radio numbers.

This is the demo: it shows, in plain numbers, what the sandbox shows visually —
that better coordination trades airtime and message drops for less wasted
re-walking, and that all of it degrades as the radio range shrinks.
"""
from __future__ import annotations

from .comms import Fleet, NCELL
from .strategies import STRATEGIES


def run_episode(strategy: str, *, count=6, range_m=14.0, airtime=4.0, memory=60,
                inbox=8, reliability=0.85, seed=0, targets=8, base=True,
                dt=0.2, max_t=240.0) -> dict:
    policy = STRATEGIES[strategy]
    fleet = Fleet(count=count, range_m=range_m, airtime=airtime, memory=memory,
                  inbox=inbox, reliability=reliability, seed=seed,
                  targets=targets, base=base)
    steps = int(max_t / dt)
    for _ in range(steps):
        fleet.step(dt, policy)
        if fleet.coverage() >= 0.999 and len(fleet.base_delivered) >= len(fleet.targets):
            break
    m = fleet.metrics
    return {
        "strategy": strategy,
        "time_s": round(fleet.t, 1),
        "coverage": round(fleet.coverage() * 100, 1),
        "redundant": m["redundant"],
        "sent": m["sent"],
        "delivered": m["recv"],
        "dropped": m["drop"],
        "loss_pct": round(m["drop"] / max(1, m["drop"] + m["recv"]) * 100, 1),
        "data_home": f"{m['data_delivered']}/{len(fleet.targets)}",
    }


def main(argv: list[str] | None = None) -> None:
    import sys
    argv = sys.argv[1:] if argv is None else argv
    which = [argv[0]] if argv and argv[0] in STRATEGIES else list(STRATEGIES)
    print(f"{'strategy':<13} {'cover':>6} {'time':>7} {'re-walks':>9} "
          f"{'sent':>6} {'deliv':>6} {'dropped':>8} {'loss':>6} {'data→base':>10}")
    print("-" * 80)
    for name in which:
        r = run_episode(name)
        print(f"{r['strategy']:<13} {r['coverage']:>5}% {r['time_s']:>6}s "
              f"{r['redundant']:>9} {r['sent']:>6} {r['delivered']:>6} "
              f"{r['dropped']:>8} {r['loss_pct']:>5}% {r['data_home']:>10}")
    print("\nclaim-and-yield finishes first: claiming *intent* de-conflicts who "
          "goes where. Sharing only the map (gossip) can backfire — robots agree "
          "on the same nearest frontier and clump. A lone commander is tidy but "
          "fragile once a robot drifts out of radio range. All of it degrades as "
          "range shrinks and airtime/memory tighten — try it live at /fleet.")


if __name__ == "__main__":
    main()
