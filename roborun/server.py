"""RoboRun server — thin HTTP shell that dispatches to route modules.

All business logic lives in roborun/routes/*.py. This file handles:
  1. Static file serving (web/)
  2. Route dispatch (GET/POST)
  3. MCP SSE endpoints
  4. MJPEG camera stream
  5. Startup (telemetry WS, ROS bridge, trajectory recorder)
"""
from __future__ import annotations

import json
import os
import re
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
WEB_ROOT = Path(__file__).resolve().parent / "web"
HOST = "127.0.0.1"
PORT = int(os.environ.get("ROBORUN_PORT", "8765"))
STATE_ROOT = ROOT / ".roborun"

# CORS allowlist. A running roborun exposes its API to the browser; allow only
# same-machine origins (the local UI / dev) and the known hosted demo platforms
# (Vercel, GitHub Pages), plus anything in ROBORUN_ALLOWED_ORIGINS. Anything else
# is refused, so a random site the user visits can't drive their robot. Replaces
# the old blanket "*".
_ALLOWED_ORIGIN_RE = re.compile(
    r"^https?://(localhost|127\.0\.0\.1)(:\d+)?$"
    r"|^https://([a-z0-9-]+\.)*(vercel\.app|github\.io)$",
    re.IGNORECASE,
)


def allowed_origin(origin: str | None) -> str | None:
    """Echo `origin` back iff it may call this server cross-origin, else None."""
    if not origin:
        return None
    extra = {o.strip() for o in os.environ.get("ROBORUN_ALLOWED_ORIGINS", "").split(",") if o.strip()}
    if origin in extra or _ALLOWED_ORIGIN_RE.match(origin):
        return origin
    return None

# each pipeline writes its own file; the stream picks by ?source=
_SOURCE_FRAMES = {
    "robot": [Path("/tmp/roborun_robot_frame.jpg")],
    "webcam": [Path("/tmp/roborun_frame.jpg"), Path("/tmp/roborun_camera.jpg")],
}


def _stream_paths(source: str) -> list:
    if source in _SOURCE_FRAMES:
        return _SOURCE_FRAMES[source]
    # auto: a robot camera producing fresh frames outranks the webcam
    robot = _SOURCE_FRAMES["robot"][0]
    try:
        if time.time() - robot.stat().st_mtime < 2.0:
            return [robot]
    except OSError:
        pass
    return _SOURCE_FRAMES["webcam"]

# Import route modules — registering all @get/@post handlers
import roborun.routes.dashboard  # noqa: F401
import roborun.routes.sources  # noqa: F401
import roborun.routes.fleet  # noqa: F401
import roborun.routes.tasks  # noqa: F401
import roborun.routes.webcam  # noqa: F401
import roborun.routes.simulator  # noqa: F401
import roborun.routes.agent  # noqa: F401
import roborun.routes.ros  # noqa: F401
import roborun.routes.memory  # noqa: F401
import roborun.routes.launch  # noqa: F401
import roborun.routes.skills  # noqa: F401
import roborun.routes.run  # noqa: F401
import roborun.routes.arena  # noqa: F401
import roborun.routes.behaviors  # noqa: F401
import roborun.routes.scenarios  # noqa: F401
import roborun.routes.search  # noqa: F401
import roborun.routes.projects  # noqa: F401
from roborun.routes import dispatch_get, dispatch_post, read_json, send_json, ApiError
from roborun.routes.mcp import handle_mcp_request, handle_mcp_sse


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(WEB_ROOT), **kwargs)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[roborun] {self.address_string()} - {fmt % args}")

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        # CORS, centralized: the static/hosted site probes this server
        # cross-origin and upgrades itself to the live cockpit. Echo only
        # allowlisted origins (see allowed_origin) — never a blanket "*".
        origin = self.headers.get("Origin") if getattr(self, "headers", None) else None
        allow = allowed_origin(origin)
        if allow:
            self.send_header("Access-Control-Allow-Origin", allow)
            self.send_header("Vary", "Origin")
        super().end_headers()

    def do_GET(self) -> None:
        path_only = self.path.split("?", 1)[0]

        # quiet the browser's favicon probe (was the only console 404)
        if path_only == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return

        # MCP SSE discovery
        if path_only in ("/mcp", "/mcp/ros"):
            handle_mcp_sse(self)
            return

        # Event timeline SSE stream
        if path_only == "/api/events/stream":
            self._event_stream()
            return

        # MJPEG camera stream
        if path_only == "/api/camera/stream":
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            self._mjpeg_stream((q.get("source") or ["auto"])[0])
            return

        # single JPEG frame — robust feed for clients that poll img.src
        if path_only == "/api/camera/frame":
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            self._camera_frame((q.get("source") or ["auto"])[0])
            return

        # Route registry
        if dispatch_get(self.path, self):
            return

        # One canonical view at one URL: "/". The old paths just redirect
        # there so nothing 404s, but there's a single route, not three.
        # Studio is the one front door. The old standalone pages collapse into
        # it: cockpit+fleet-sim → /sims, browser+timeline+run → /runs, the rest
        # are reachable inside Studio's shell. Redirect the absorbed routes.
        _STUDIO_REDIRECT = {
            "/": "/studio/live", "/deck": "/studio/live", "/home": "/studio/live",
            "/arena": "/studio/sims", "/browser": "/studio/runs",
            "/timeline": "/studio/runs", "/run": "/studio/runs",
            "/search": "/studio/search",
        }
        if path_only in _STUDIO_REDIRECT:
            self.send_response(302)
            self.send_header("Location", _STUDIO_REDIRECT[path_only])
            self.end_headers()
            return
        # Pages Studio still hosts (iframed) or links to — keep serving their html.
        if path_only == "/setup":
            self.path = "/setup.html"
        if path_only == "/fleet":            # Swarm Lab (Studio /swarm)
            self.path = "/fleet.html"
        if path_only == "/scenarios":        # Studio /scenarios
            self.path = "/scenarios.html"
        if path_only == "/analytics":        # Studio /analytics
            self.path = "/analytics.html"
        if path_only == "/sim":              # Studio Sims · Arena
            self.path = "/arena.html"
        if path_only == "/projects":
            self.path = "/projects.html"
        if path_only == "/fleet-sim":        # Studio Sims · Fleet
            self.path = "/fleet-sim.html"
        # Studio SPA (React, client-side routing): any /studio/* that isn't a
        # real built asset falls back to its index.html. Isolated subpath so the
        # old pages keep working until parity; flipped to "/" in the last phase.
        if path_only == "/studio" or path_only.startswith("/studio/"):
            if not (WEB_ROOT / path_only.lstrip("/")).is_file():
                self.path = "/studio/index.html"
        super().do_GET()

    def do_OPTIONS(self) -> None:
        self.send_response(200)
        # Allow-Origin comes from end_headers — sending it here too would
        # duplicate the header, which browsers reject outright
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        # Chrome Private Network Access: lets a public https site (the hosted
        # demo) reach this server on 127.0.0.1 once PNA enforcement lands
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.end_headers()

    def _handle_upload(self) -> None:
        """Drop-zone: accept a raw .mcap upload and save it as a replayable run
        (robot_id 'uploaded'). The body is binary, so this bypasses JSON dispatch."""
        import re as _re
        from urllib.parse import parse_qs, urlparse
        from roborun.recorder import runs_root
        q = parse_qs(urlparse(self.path).query)
        name = (q.get("name") or ["upload.mcap"])[0]
        stem = _re.sub(r"[^A-Za-z0-9_.-]", "_", name).rsplit(".mcap", 1)[0][:80] or \
            time.strftime("upload_%Y%m%d_%H%M%S")
        length = int(self.headers.get("Content-Length", "0") or "0")
        data = self.rfile.read(length) if length else b""
        # MCAP files start with the magic \x89MCAP0\r\n — reject anything else
        if not data[:5] == b"\x89MCAP":
            send_json(self, 400, {"ok": False,
                                  "error": "not an .mcap file (.bag/.db3 need conversion to MCAP first)"})
            return
        dest = runs_root() / "uploaded"
        dest.mkdir(parents=True, exist_ok=True)
        path = dest / f"{stem}.mcap"
        n = 1
        while path.exists():
            path = dest / f"{stem}_{n}.mcap"; n += 1
        path.write_bytes(data)
        indexed = None
        try:  # best-effort: index detections/clips so it's searchable too
            from roborun.observations import extract_run
            from roborun.routes._singletons import get_memory
            indexed = extract_run(path, get_memory(), robot_id="uploaded")
        except Exception as exc:
            indexed = {"ok": False, "error": str(exc)}
        from roborun.events import emit
        emit("system", "upload", f"imported {path.name} ({len(data)//1024}KB)", {"run": path.stem})
        send_json(self, 200, {"ok": True, "run": path.stem, "robot_id": "uploaded",
                              "bytes": len(data), "indexed": indexed})

    def do_POST(self) -> None:
        if self.path.split("?", 1)[0] == "/api/run/upload":
            self._handle_upload()
            return
        # MCP JSON-RPC
        if self.path in ("/mcp", "/mcp/ros"):
            try:
                payload = read_json(self)
            except Exception as exc:
                body = json.dumps({"jsonrpc": "2.0", "id": None,
                                   "error": {"code": -32700, "message": f"Parse error: {exc}"}}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            handle_mcp_request(self, payload)
            return

        try:
            payload = read_json(self)
            if dispatch_post(self.path, self, payload):
                return
            raise ApiError(404, "Unknown API route")
        except ApiError as exc:
            send_json(self, exc.status, {"ok": False, "error": exc.message})

    def _event_stream(self) -> None:
        import queue as _queue
        from roborun.events import subscribe, unsubscribe, recent
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        q = subscribe()
        try:
            for evt in recent(50):
                line = f"data: {json.dumps(evt, default=str)}\n\n"
                self.wfile.write(line.encode())
            self.wfile.flush()
            while True:
                try:
                    evt = q.get(timeout=15)
                    line = f"data: {json.dumps(evt, default=str)}\n\n"
                    self.wfile.write(line.encode())
                    self.wfile.flush()
                except _queue.Empty:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            unsubscribe(q)

    def _camera_frame(self, source: str = "auto") -> None:
        for p in _stream_paths(source):
            try:
                data = p.read_bytes()
            except OSError:
                continue
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            self.wfile.write(data)
            return
        self.send_response(503)
        self.end_headers()

    def _mjpeg_stream(self, source: str = "auto") -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            last_mtime = 0.0
            started = time.monotonic()
            try:
                self.connection.settimeout(30.0)
            except Exception:
                pass
            while time.monotonic() - started < 300:
                for p in _stream_paths(source):
                    if p.exists():
                        mtime = p.stat().st_mtime
                        if mtime != last_mtime:
                            last_mtime = mtime
                            data = p.read_bytes()
                            header = (b"--frame\r\nContent-Type: image/jpeg\r\n"
                                      b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n")
                            self.wfile.write(header + data + b"\r\n")
                            self.wfile.flush()
                        break
                time.sleep(0.033)
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass


def _frame_recorder_loop() -> None:
    import hashlib
    from roborun.events import emit
    from roborun.routes._singletons import get_webcam
    hash_interval = float(os.environ.get("ROBORUN_FRAME_HASH_INTERVAL", "2.0"))
    last_hash_at = 0.0
    while True:
        try:
            wc = get_webcam()
            # Visual evidence into the chain: hash the live frame periodically
            if hash_interval > 0 and wc.is_running and time.monotonic() - last_hash_at >= hash_interval:
                frame = wc.snapshot()
                if frame is not None:
                    last_hash_at = time.monotonic()
                    digest = hashlib.sha256(frame.tobytes()).hexdigest()
                    dets = wc.get_detections()
                    labels = ", ".join(sorted({d.get("label", "?") for d in dets})) or "no objects"
                    emit("frame", "camera", f"frame {digest[:12]}… · {labels}",
                         {"sha256": digest, "objects": len(dets),
                          "shape": list(getattr(frame, "shape", []))})
        except Exception:
            pass
        time.sleep(0.1)


_HELP = """RoboRun — run robots, record everything, search it over time.

  roborun                 start the server + UI (http://localhost:8765)
  roborun run             headless: drive behaviors, stream the loop to the terminal (no UI)
  roborun tui             full-screen terminal dashboard (live events · behaviors · vision)
  roborun status          is it running, what's connected, how much is recorded
  roborun demo            load sample data so the dashboards aren't empty
  roborun ask "<task>"    tell the robot what to do in plain English (agent drives)
  roborun stop            EMERGENCY STOP — halt all actuators + disable behaviors
  roborun connect <ip>    connect a real robot (ROS 1 or ROS 2, via rosbridge)
  roborun search <query>  find anything/anyone across every recorded run
  roborun scenarios       list / run scored behavior tests   (run <name> | suite <name>)
  roborun dataset <q> <d> curate a labeled training set from a search (sealed provenance)
  roborun flag <run>      bookmark a moment in a run to revisit
  roborun skill <...>     install / manage skills from GitHub

Open the cockpit, then its ▤ VIEWS menu for: search · scenarios · timeline · analytics · fleet."""


def _run_autostart() -> None:
    """First boot should be alive, not a NO SIGNAL screen: try the webcam with
    YOLO; fall back to the MuJoCo sim. ROBORUN_AUTOSTART=0 disables it."""
    from roborun.events import emit
    time.sleep(2.0)  # let a previous instance release the camera
    # a connected/saved robot IS the camera source — never grab the laptop
    # webcam (and its privacy light) out from under the user
    try:
        from roborun.connect import saved_robot
        if saved_robot():
            emit("system", "server", "autostart: a robot is the source — webcam left off")
            return
    except Exception:
        pass
    why: list[str] = []
    try:
        from roborun.routes._singletons import get_webcam
        result = get_webcam().start(camera_index=0, models=["yolo"])
        if result.get("ok"):
            emit("system", "server", "autostart: webcam live with YOLO")
            return
        why.append(f"webcam: {result.get('error', 'failed')}")
    except ImportError:
        why.append("webcam vision not installed (pip install 'ros-agent[vision]')")
    except Exception as exc:
        why.append(f"webcam: {exc}")
    try:
        from roborun.routes._singletons import get_simulator
        result = get_simulator().start()
        if result.get("ok"):
            emit("system", "server", f"autostart: MuJoCo sim ({result.get('robot', 'robot')})")
            return
        why.append(f"sim: {result.get('error', 'failed')}")
    except ImportError:
        why.append("MuJoCo sim not installed (pip install 'ros-agent[sim]')")
    except Exception as exc:
        why.append(f"sim: {exc}")
    # A blank deck with no explanation reads as broken — say exactly what didn't
    # start and point at the path that needs no installs.
    emit("system", "server",
         "no camera or sim started — " + "; ".join(why) +
         ". Open the cockpit: browser sim, nothing to install.")


def start_runtime(announce=print, autostart: bool = True) -> None:
    """Boot the live robot runtime — telemetry WS, ROS bridge, trajectory
    recorder, frame hashing, and behavior hot-reload — WITHOUT the HTTP server.

    Shared by the web server (`roborun`) and the headless / TUI run modes
    (`roborun run`, `roborun tui`), so the see/move/ask loop runs identically
    with or without a browser. Workers are daemon threads; returns once started.
    """
    STATE_ROOT.mkdir(parents=True, exist_ok=True)

    # A robot saved by `roborun connect` is the robot behaviors drive
    from roborun.connect import saved_robot
    robot = saved_robot()
    if robot:
        from roborun.rosbridge import get_client
        client = get_client(robot["host"], robot.get("port", 9090))
        state = "connected" if client and client.is_connected else "unreachable — will retry"
        announce(f"  Robot {robot['host']} ({robot.get('type', '?')}): {state}")
        if client and client.is_connected:
            try:
                from roborun.ros_camera import get_ros_camera
                cam = get_ros_camera().start()
                if cam.get("ok"):
                    announce(f"  Robot camera: {cam['topic']} → YOLO → robot.see()")
            except Exception:
                pass

    from roborun.skills import load_skills
    count = load_skills()
    if count:
        announce(f"  Loaded {count} skill(s)")

    threading.Thread(target=_frame_recorder_loop, daemon=True, name="FrameRecorder").start()

    from roborun.telemetry import start_ws_server
    start_ws_server()

    from roborun.ros_telemetry import get_bridge
    get_bridge().start()

    from roborun.trajectory import TrajectoryRecorder
    TrajectoryRecorder.get().start()

    # Vibecode runtime: behaviors/*.py hot-reload while the robot runs
    from roborun.behaviors import BehaviorRunner, write_examples
    created = write_examples()
    if created:
        announce(f"  Created {created}/ — edit follow_person.py and save. It reloads live.")
    BehaviorRunner.get().start()

    # Reach-a-human channel: forward notify events to an OpenClaw gateway
    from roborun.openclaw import start_bridge
    if start_bridge():
        announce(f"  OpenClaw bridge:  notify() → {os.environ['OPENCLAW_HOOKS_URL']}")

    if autostart and os.environ.get("ROBORUN_AUTOSTART", "1") != "0":
        threading.Thread(target=_run_autostart, daemon=True, name="Autostart").start()


def main() -> None:
    import sys
    if len(sys.argv) > 1 and sys.argv[1] in ("help", "--help", "-h"):
        print(_HELP)
        raise SystemExit(0)
    if len(sys.argv) > 1 and sys.argv[1] == "skill":
        from roborun.skills.manager import cli
        raise SystemExit(cli(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "connect":
        from roborun.connect import cli
        raise SystemExit(cli(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "search":
        from roborun.cli import search_cli
        raise SystemExit(search_cli(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "scenarios":
        from roborun.cli import scenarios_cli
        raise SystemExit(scenarios_cli(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "dataset":
        from roborun.cli import dataset_cli
        raise SystemExit(dataset_cli(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "demo":
        from roborun.cli import demo_cli
        raise SystemExit(demo_cli(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "ask":
        from roborun.cli import ask_cli
        raise SystemExit(ask_cli(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "status":
        from roborun.cli import status_cli
        raise SystemExit(status_cli(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "stop":
        from roborun.cli import stop_cli
        raise SystemExit(stop_cli(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "flag":
        from roborun.incidents import flag, list_incidents
        if len(sys.argv) > 2:
            r = flag(sys.argv[2], note=" ".join(sys.argv[3:]))
            print(f"flagged {r['id']} on {r['run_id']}")
        else:
            for i in list_incidents():
                print(f"  {i['run_id']}  ⚑ {i.get('note') or i['tag']}")
        raise SystemExit(0)
    # Headless + TUI run modes — drive the see/move/ask loop with no web UI
    if len(sys.argv) > 1 and sys.argv[1] in ("run", "headless"):
        from roborun.tui import run_headless
        raise SystemExit(run_headless(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "tui":
        from roborun.tui import run_tui
        raise SystemExit(run_tui(sys.argv[2:]))
    if not WEB_ROOT.exists():
        raise SystemExit(f"Missing web directory at {WEB_ROOT}")

    start_runtime(announce=print)

    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"\n  RoboRun is live: http://{HOST}:{PORT}")
    print(f"  MCP endpoint:    http://{HOST}:{PORT}/mcp   ·   Telemetry WS: ws://127.0.0.1:8766")
    # Quick-start: point new users at the dashboards + a one-command win.
    try:
        from roborun.spatial_memory import SpatialMemoryStore
        if SpatialMemoryStore().stats().get("total", 0) == 0:
            print(f"\n  New here? Run  roborun demo  in another terminal to load sample data,")
            print(f"  then open http://{HOST}:{PORT}/search to find anything your robots have seen.")
    except Exception:
        pass
    print(f"\n  Views: /search · /scenarios · /timeline · /analytics · /run   (or the cockpit's ▤ VIEWS menu)\n")
    from roborun.events import emit
    emit("system", "server", "roborun started", {"port": PORT})
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        try:
            from roborun.routes._singletons import get_webcam
            get_webcam().stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
