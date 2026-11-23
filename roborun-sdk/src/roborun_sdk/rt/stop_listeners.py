"""K1 Stop sources that live in roborun-rt: LAN UDP, ``POST /stop`` (no login), SIGUSR1, HID pedal.

Defaults bind 127.0.0.1; ``bind="0.0.0.0"`` exposes the LAN Stop (no auth by design: Stop is always safe). Every
source calls ``runtime.stop(source)`` on its own thread; nothing here waits for the tick loop.
UDP: any datagram starting with ``STOP`` (optionally ``STOP <id>``). HTTP: ``POST /stop`` answers 200 with the
per-arm report (``GET /stop`` is 405 so a browser prefetch never stops the robot by accident). A web page gets no Stop here: a
request with an Origin or cross-site Fetch Metadata is refused, and on loopback the Host must name this listener.
"""
from __future__ import annotations

import json
import signal
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class _StopServer(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 128      # a Stop must never wait behind a full stdlib backlog of 5 (kernel SYN retry after 1 s)

DEFAULT_PORT = 8111


class StopListeners:
    def __init__(self, runtime, *, bind: str = "127.0.0.1", udp_port: int | None = DEFAULT_PORT,
                 http_port: int | None = DEFAULT_PORT, sigusr1: bool = False):
        self.runtime = runtime
        self.bind = bind
        self.udp_port = udp_port
        self.http_port = http_port
        self.sigusr1 = sigusr1
        self._udp: socket.socket | None = None
        self._http: ThreadingHTTPServer | None = None
        self._threads: list = []
        self._closing = threading.Event()
        self._signal_event = threading.Event()
        self.received: list = []

    def _stop(self, source: str, stop_id: str | None = None) -> dict:
        self.received.append(source)
        return self.runtime.stop(source, stop_id=stop_id)

    def start(self) -> "StopListeners":
        if self.udp_port is not None:
            self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp.bind((self.bind, self.udp_port))
            self._udp.settimeout(0.2)
            self.udp_port = self._udp.getsockname()[1]
            self._spawn(self._udp_loop, "manifest-stop-udp")
        if self.http_port is not None:
            listeners = self

            class Handler(BaseHTTPRequestHandler):
                def log_message(self, *args):
                    pass

                def _reply(self, code, data):
                    raw = json.dumps(data, default=str).encode()
                    self.send_response(code)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(raw)))
                    self.end_headers()
                    self.wfile.write(raw)

                def _from_a_web_page(self):
                    site = (self.headers.get("Sec-Fetch-Site") or "").strip().lower()
                    if self.headers.get("Origin") is not None or site not in ("", "none"):
                        return True
                    if listeners.bind not in ("127.0.0.1", "localhost", "::1"):
                        return False                      # the LAN Stop: reached under whatever name the network gives it
                    port = self.server.server_address[1]
                    return (self.headers.get("Host") or "").strip().lower() not in (f"127.0.0.1:{port}", f"localhost:{port}", f"[::1]:{port}")

                def do_POST(self):
                    if self._from_a_web_page():
                        return self._reply(403, {"error": "web pages cannot use this Stop"})
                    if self.path.split("?", 1)[0] != "/stop":
                        return self._reply(404, {"error": "only POST /stop lives here"})
                    report = listeners._stop("lan_http")
                    self._reply(200, {"stopped": True, "report": report})

                def do_GET(self):
                    self._reply(405, {"error": "Stop is POST /stop"})

            self._http = _StopServer((self.bind, self.http_port), Handler)
            self._http.daemon_threads = True
            self.http_port = self._http.server_address[1]
            self._spawn(self._http.serve_forever, "manifest-stop-http")
        if self.sigusr1:
            signal.signal(signal.SIGUSR1, lambda *_: self._signal_event.set())
            self._spawn(self._signal_loop, "manifest-stop-signal")
        return self

    def _spawn(self, fn, name):
        t = threading.Thread(target=fn, name=name, daemon=True)
        t.start()
        self._threads.append(t)

    def _udp_loop(self):
        while not self._closing.is_set():
            try:
                data, _ = self._udp.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                return
            if data.startswith(b"STOP"):
                parts = data.decode(errors="replace").split()
                self._stop("lan_udp", parts[1][:64] if len(parts) > 1 else None)

    def _signal_loop(self):
        while not self._closing.is_set():
            if self._signal_event.wait(0.2):
                self._signal_event.clear()
                self._stop("sigusr1")

    def close(self) -> None:
        self._closing.set()
        if self._http is not None:
            self._http.shutdown()
            self._http.server_close()
        if self._udp is not None:
            self._udp.close()
        for t in self._threads:
            t.join(1.0)


def send_udp_stop(host: str = "127.0.0.1", port: int = DEFAULT_PORT, stop_id: str | None = None, repeats: int = 3) -> None:
    payload = b"STOP" + (b" " + stop_id.encode() if stop_id else b"")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        for _ in range(repeats):
            s.sendto(payload, (host, port))
