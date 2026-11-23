"""HTTP front ends for the fakes, bound to 127.0.0.1 only (run tests under `unshare -rn` with lo up).

Faults act on the transport like the real failures: `link_loss` and a dead owner drop the connection without an
answer; `gc_pause` holds every reply (and the owner's ticks) until the pause ends; `slow_reply` delays one route.
Web pages get nothing here: a request must name this port on loopback in its Host, and one that shows it comes from a browser
(an Origin, cross-site Fetch Metadata, or a form-style POST from a browser's User-Agent) is refused before any route. Programs
on this computer send none of those. The supervisor routes require the station's local-client header, except POST
/supervisor/hold, which any local program may send (without the header it is obeyed and recorded as unverified); the Metal owner's POSTs require X-Manifest-Metal-Control: 1
and a JSON body.
"""
from __future__ import annotations

import json
import re
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .metal_owner import FakeMetalOwner, OwnerDead
from .yam_station import FakeYamStation

LOOPBACK_NAMES = ("127.0.0.1", "localhost", "[::1]")
FORM_TYPES = ("application/x-www-form-urlencoded", "multipart/form-data", "text/plain")
MAX_BODY = 16 << 20


def web_refusal(headers, port, method):
    """(status, message) when a request must not reach the station: wrong Host, a web page, or a body it cannot delimit."""
    hosts = headers.get_all("Host") or []
    if len(hosts) != 1 or hosts[0].strip().lower() not in [f"{name}:{port}" for name in LOOPBACK_NAMES]:
        return 421, "The station answers on its loopback address only"
    site = (headers.get("Sec-Fetch-Site") or "").strip().lower()
    browser_form = (method != "GET" and "mozilla/" in (headers.get("User-Agent") or "").lower()
                    and (headers.get("Content-Type") or "").split(";")[0].strip().lower() in FORM_TYPES)
    if headers.get_all("Origin") or site not in ("", "none") or browser_form:
        return 403, "Web pages cannot use the station"
    lengths = headers.get_all("Content-Length") or []
    if headers.get_all("Transfer-Encoding"):
        return 411, "Send a Content-Length"
    if lengths and (len(lengths) != 1 or not re.fullmatch(r"[0-9]{1,12}", lengths[0].strip())):
        return 400, "Invalid Content-Length"
    if lengths and int(lengths[0]) > MAX_BODY:
        return 413, "Request body too large"
    return None


class _Listener(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 128      # the stdlib backlog of 5 drops a Stop's SYN behind a few pollers (retried by the kernel after 1 s)


class SimServer:
    def __init__(self, fake, *, port: int = 0, tick_hz: float = 30.0):
        self.fake = fake
        self.tick_hz = tick_hz
        self._stop = threading.Event()
        handler = _handler_for(fake)
        self.httpd = _Listener(("127.0.0.1", port), handler)
        self.httpd.daemon_threads = True
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        self._threads = [threading.Thread(target=self.httpd.serve_forever, daemon=True, name="sim-http"),
                         threading.Thread(target=self._ticker, daemon=True, name="sim-tick")]

    def start(self):
        for t in self._threads:
            t.start()
        return self

    def _ticker(self):
        period = 1.0 / self.tick_hz
        while not self._stop.is_set():
            started = time.monotonic()
            self.fake.tick()
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    def close(self):
        self._stop.set()
        self.httpd.shutdown()
        self.httpd.server_close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()


def _handler_for(fake):
    metal = isinstance(fake, FakeMetalOwner)
    station = isinstance(fake, FakeYamStation)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def _drop(self):
            self.close_connection = True
            try:
                self.connection.shutdown(2)
            except OSError:
                pass

        def _refused(self) -> bool:
            refused = web_refusal(self.headers, self.server.server_address[1], self.command)
            if refused is None:
                return False
            self.close_connection = True
            raw = json.dumps({"error": refused[1]}).encode()
            self.send_response(refused[0])
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(raw)
            self.wfile.flush()
            try:                                   # what is in flight is discarded, so closing does not destroy the answer
                self.connection.shutdown(socket.SHUT_WR)
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    self.connection.settimeout(max(0.01, end - time.monotonic()))
                    if not self.connection.recv(65536):
                        break
            except OSError:
                pass
            return True

        def _gate(self) -> bool:
            now = fake.clock()
            if getattr(fake, "dead", False) or fake.faults.active("link_loss", now) is not None:
                self._drop()
                return False
            until = fake.faults.pause_until(now)
            if until is not None:
                time.sleep(max(0.0, min(5.0, until - now)))
            slow = fake.faults.active("slow_reply", now, route=self.path.split("?", 1)[0])
            if slow is not None:
                time.sleep(slow.get("delay_s", 0.0))
            return True

        def _reply(self, code, value):
            raw = json.dumps(value, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _call(self, fn, body):
            try:
                self._reply(200, fn(body))
            except OwnerDead:
                self._drop()
            except PermissionError as exc:
                self._reply(403, {"error": str(exc)})
            except LookupError as exc:
                self._reply(404, {"error": str(exc)})
            except ValueError as exc:
                self._reply(409, {"error": str(exc)})
            except Exception as exc:  # noqa: BLE001 - the real owners answer 503 for anything else
                self._reply(503, {"error": f"{type(exc).__name__}: {exc}"})

        def do_GET(self):
            if self._refused() or not self._gate():
                return
            get, _ = fake.routes()
            route = self.path.split("?", 1)[0]
            if route not in get:
                return self._reply(404, {"error": "Unknown route"})
            self._call(get[route], None)

        def do_OPTIONS(self):
            if not self._refused():
                self.close_connection = True
                self.send_error(405)

        def do_POST(self):
            if self._refused() or not self._gate():
                return
            route = self.path.split("?", 1)[0]
            length = int(self.headers.get("Content-Length", "0") or 0)
            raw = self.rfile.read(length) if length else b""
            try:
                body = json.loads(raw or b"{}")
                if not isinstance(body, dict):
                    raise ValueError("Expected JSON object")
            except ValueError as exc:
                return self._reply(409, {"error": str(exc)})
            if metal and (self.headers.get("X-Manifest-Metal-Control") != "1" or "Origin" in self.headers
                          or self.headers.get("Content-Type", "").split(";")[0].strip().lower() != "application/json"):
                return self._reply(403, {"error": "Metal control requires a trusted local JSON request without a browser Origin"})
            _, post = fake.routes()
            if route in ("/home", "/reset") and self.headers.get("X-Manifest-Supervisor") != "1":
                # every arm, not only the Metal: Home opens the jaws and Reset re-enables the motors, so only Roborun's own local
                # client (which checks a task is not running and the jaws are empty first) may ask for them
                return self._reply(403, {"error": "Use the local supervisor client (roborun's own backend) for Home and Reset"})
            if station and route.startswith("/supervisor/"):
                verified = self.headers.get("X-Manifest-Supervisor") == "1"
                if route == "/supervisor/hold" and route in post:
                    # never refused: an unverified Stop is obeyed and recorded as unverified
                    return self._call(lambda b: fake.hold(b, verified=verified), body)
                if not verified:
                    return self._reply(403, {"error": "Use the local supervisor client"})
            if route not in post:
                return self._reply(404, {"error": "Unknown supervisor route" if route.startswith("/supervisor/") else "Unknown route"})
            self._call(post[route], body)

    return Handler
