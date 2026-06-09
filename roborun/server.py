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

_FRAME_PATHS = [
    Path("/tmp/roborun_frame.jpg"),
    Path("/tmp/roborun_camera.jpg"),
]

# Import route modules — registering all @get/@post handlers
import roborun.routes.dashboard  # noqa: F401
import roborun.routes.fleet  # noqa: F401
import roborun.routes.tasks  # noqa: F401
import roborun.routes.webcam  # noqa: F401
import roborun.routes.simulator  # noqa: F401
import roborun.routes.agent  # noqa: F401
import roborun.routes.ros  # noqa: F401
import roborun.routes.memory  # noqa: F401
import roborun.routes.dataset  # noqa: F401
import roborun.routes.launch  # noqa: F401
import roborun.routes.skills  # noqa: F401
import roborun.routes.zk  # noqa: F401
from roborun.routes import dispatch_get, dispatch_post, read_json, send_json, ApiError
from roborun.routes.mcp import handle_mcp_request, handle_mcp_sse


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(WEB_ROOT), **kwargs)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[roborun] {self.address_string()} - {fmt % args}")

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        super().end_headers()

    def do_GET(self) -> None:
        path_only = self.path.split("?", 1)[0]

        # MCP SSE discovery
        if path_only in ("/mcp", "/mcp/ros"):
            handle_mcp_sse(self)
            return

        # MJPEG camera stream
        if path_only == "/api/camera/stream":
            self._mjpeg_stream()
            return

        # Route registry
        if dispatch_get(self.path, self):
            return

        # Static files
        if self.path == "/":
            self.path = "/index.html"
        super().do_GET()

    def do_OPTIONS(self) -> None:
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self) -> None:
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
                self.send_header("Access-Control-Allow-Origin", "*")
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

    def _mjpeg_stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        try:
            last_mtime = 0.0
            started = time.monotonic()
            try:
                self.connection.settimeout(30.0)
            except Exception:
                pass
            while time.monotonic() - started < 300:
                for p in _FRAME_PATHS:
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
    from roborun.routes._singletons import get_webcam, get_dataset
    while True:
        try:
            ds = get_dataset()
            wc = get_webcam()
            if ds.is_recording and wc.is_running:
                frame = wc.snapshot()
                if frame is not None:
                    ds.record_frame(frame, detections=wc.get_detections())
        except Exception:
            pass
        time.sleep(0.1)


def main() -> None:
    if not WEB_ROOT.exists():
        raise SystemExit(f"Missing web directory at {WEB_ROOT}")
    STATE_ROOT.mkdir(parents=True, exist_ok=True)

    # Load skills
    from roborun.skills import load_skills
    count = load_skills()
    if count:
        print(f"  Loaded {count} skill(s)")

    recorder = threading.Thread(target=_frame_recorder_loop, daemon=True, name="FrameRecorder")
    recorder.start()

    from roborun.telemetry import start_ws_server
    start_ws_server()

    from roborun.ros_telemetry import get_bridge
    get_bridge().start()

    from roborun.trajectory import TrajectoryRecorder
    TrajectoryRecorder.get().start()

    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"\n  RoboRun is live: http://{HOST}:{PORT}")
    print(f"  Telemetry WS:    ws://127.0.0.1:8766")
    print(f"  MCP endpoint:    http://{HOST}:{PORT}/mcp\n")
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
