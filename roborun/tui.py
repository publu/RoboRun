"""Headless + terminal-UI run modes.

`roborun run` (a.k.a. `headless`) boots the full robot runtime — telemetry,
ROS bridge, recorder, and behavior hot-reload — with NO web UI, and streams the
see/move/ask event loop to stdout. Zero extra dependencies.

`roborun tui` renders the same runtime as a live full-screen dashboard
(events · behaviors · vision). Needs `rich` (pip install 'ros-agent[tui]');
falls back to a pointer at `roborun run` if it isn't installed.

Both reuse roborun.server.start_runtime() and the roborun.events bus, so the
robot runs identically with or without a browser pointed at it.
"""
from __future__ import annotations

import queue as _queue
import time as _t

# Glyphs per event type (roborun.events.EVENT_TYPES). Web-side glyphs live in
# deck.js; these are the terminal twins.
_GLYPH = {
    "mcp_tool": "🔧", "detection": "👁", "ros": "🤖", "agent": "🧠",
    "system": "•", "task": "▶", "frame": "📷", "notify": "🔔",
}


def _fmt_event(evt: dict) -> str:
    ts = _t.strftime("%H:%M:%S", _t.localtime(evt.get("ts", 0)))
    g = _GLYPH.get(evt.get("type", ""), "·")
    src = evt.get("source", "")
    title = evt.get("title", "")
    # dim timestamp, bold source — readable on any terminal, degrades to plain
    return f"  \033[2m{ts}\033[0m  {g} \033[1m{src}\033[0m  {title}"


def run_headless(argv: list[str]) -> int:
    """Drive behaviors with no web UI; stream live events to the terminal."""
    from roborun.server import start_runtime
    from roborun.events import subscribe, unsubscribe, recent

    print("\n  RoboRun — headless run (no web UI). Ctrl-C to stop.\n")
    start_runtime(announce=print)
    print("\n  Telemetry WS: ws://127.0.0.1:8766   ·   behaviors hot-reload from ./behaviors/")
    print("  Edit a behaviors/*.py file and save — the running policy changes live.\n")
    print("  ── live events ───────────────────────────────────────────────\n")

    q = subscribe()
    try:
        for evt in recent(15):
            print(_fmt_event(evt))
        while True:
            try:
                evt = q.get(timeout=1.0)
            except _queue.Empty:
                continue
            print(_fmt_event(evt))
    except KeyboardInterrupt:
        print("\n  Stopping.\n")
        return 0
    finally:
        unsubscribe(q)


def build_dashboard(log, accent: str = "green"):
    """Build the full-screen dashboard Layout from the current event `log` plus
    live behavior/vision state. Module-level (not closed over run_tui) so it is
    unit-testable without a TTY. Requires `rich`."""
    from rich.layout import Layout
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text

    tbl = Table(expand=True, box=None, pad_edge=False)
    tbl.add_column("behavior", style="bold")
    tbl.add_column("runs", justify="right")
    tbl.add_column("errs", justify="right")
    tbl.add_column("", justify="right")
    try:
        from roborun.behaviors import BehaviorRunner
        sts = BehaviorRunner.get().statuses()
    except Exception:
        sts = []
    if not sts:
        tbl.add_row("[dim]no behaviors loaded[/dim]", "", "", "")
    for s in sts:
        on = "[green]● on[/green]" if s.get("enabled") else "[dim]○ off[/dim]"
        errs = s.get("errors", 0)
        tbl.add_row(str(s.get("name", "")), str(s.get("runs", 0)),
                    f"[red]{errs}[/red]" if errs else "0", on)
    behaviors = Panel(tbl, title="behaviors · ./behaviors/*.py (hot-reload)",
                      border_style=accent, title_align="left")

    labels: dict[str, int] = {}
    try:
        from roborun.routes._singletons import get_webcam
        for d in get_webcam().get_detections():
            lb = d.get("label", "?")
            labels[lb] = labels.get(lb, 0) + 1
    except Exception:
        pass
    vbody = Text()
    if labels:
        for lb, n in sorted(labels.items(), key=lambda x: -x[1]):
            vbody.append(f" {lb}", style="bold")
            vbody.append(f" ×{n}\n", style="dim")
    else:
        vbody.append(" no detections — start a camera or sim\n", style="dim")
    vision = Panel(vbody, title="vision · robot.see()", border_style=accent, title_align="left")

    ebody = Text()
    for evt in list(log)[-40:]:
        ts = _t.strftime("%H:%M:%S", _t.localtime(evt.get("ts", 0)))
        ebody.append(f"{ts} ", style="dim")
        ebody.append(f"{_GLYPH.get(evt.get('type', ''), '·')} ")
        ebody.append(f"{evt.get('source', '')}  ", style="bold")
        ebody.append(f"{evt.get('title', '')}\n")
    events = Panel(ebody, title="live events · see / move / ask",
                   border_style=accent, title_align="left")

    head = Text(
        f" RoboRun headless  ·  ws://127.0.0.1:8766  ·  {len(log)} events  ·  Ctrl-C to quit",
        style=f"bold {accent}")
    layout = Layout()
    layout.split_column(
        Layout(Panel(head, border_style=accent), size=3, name="head"),
        Layout(name="body"),
    )
    layout["body"].split_row(Layout(name="left"), Layout(name="right", ratio=2))
    layout["left"].split_column(behaviors, vision)
    layout["right"].update(events)
    return layout


def run_tui(argv: list[str]) -> int:
    """Full-screen terminal dashboard over the same runtime."""
    try:
        from rich.live import Live
    except ImportError:
        print("\n  The TUI dashboard needs `rich`:  pip install 'ros-agent[tui]'")
        print("  (or run  roborun run  for the no-deps headless event stream.)\n")
        return 1

    from collections import deque
    from roborun.server import start_runtime
    from roborun.events import subscribe, unsubscribe, recent

    # announce into nothing — Live owns the screen; boot status shows up as events
    start_runtime(announce=lambda m: None)
    log: deque = deque(recent(60), maxlen=300)
    q = subscribe()

    with Live(build_dashboard(log), refresh_per_second=5, screen=True) as live:
        try:
            while True:
                try:
                    while True:
                        log.append(q.get_nowait())
                except _queue.Empty:
                    pass
                live.update(build_dashboard(log))
                _t.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            unsubscribe(q)
    return 0


def demo() -> None:
    """Self-check: the dashboard renders to a string buffer without a TTY."""
    import io
    from collections import deque
    from rich.console import Console
    log = deque([{"type": "system", "source": "server", "title": "roborun started", "ts": _t.time()},
                 {"type": "detection", "source": "camera", "title": "person ×1", "ts": _t.time()}])
    out = io.StringIO()
    Console(file=out, width=120, height=30).print(build_dashboard(log))
    text = out.getvalue()
    assert "behaviors" in text and "live events" in text and "roborun started" in text, text[:400]
    print("tui.demo OK — dashboard renders")


if __name__ == "__main__":
    demo()
