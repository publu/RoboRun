"""In-process fakes of the two native owners' HTTP surfaces, shaped like the real answers
(the YAM station's manifest-native-pose/1 pose and supervisor routes, the vendor station API, the Metal owner's
manifest-metal-owner/1).
They bind 127.0.0.1:0 inside ``unshare -rn``; they never stand for the real owners' behaviour beyond these routes."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class _Base:
    def __init__(self):
        self.requests = []
        self.lock = threading.Lock()
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _reply(self, code, data):
                raw = json.dumps(data).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self):
                with outer.lock:
                    outer.requests.append(("GET", self.path, dict(self.headers), None))
                code, data = outer.get(self.path)
                self._reply(code, data)

            def do_POST(self):
                n = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(n) or b"{}")
                with outer.lock:
                    outer.requests.append(("POST", self.path, dict(self.headers), body))
                code, data = outer.post(self.path, body, self.headers)
                self._reply(code, data)

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()

    def posts(self):
        return [p for m, p, _, _ in self.requests if m == "POST"]


class FakeYamStation(_Base):
    def __init__(self, *, hold_route=False, state="idle", lease=None, arm_state="fresh"):
        self.hold_route = hold_route
        self.state = state
        self.lease = lease            # {"id": ..., "active": True, "kind": "video_motion"}
        self.arm_state = arm_state
        self.q = [1.5967, -0.0013, 0.004, -0.0196, 0.0269, 0.2462]
        super().__init__()

    def pose(self):
        now = time.monotonic()
        row = {"body": "yam", "positions_rad": self.q, "gripper": 0.999, "commanded_rad": self.q,
               "commanded_gripper": 0.999, "source_monotonic_s": now - 0.004, "sample_id": f"{time.time():.9f}",
               "timestamp_source": "driver_state_cycle", "clock_uncertainty_s": 0.0007,
               "motor_temps_c": [41, 42, 43, 44, 40, 39, 38], "age_ms": 4.0, "fresh": True}
        out = {"schema": "manifest-native-pose/1", "version": 4, "server_monotonic_s": now,
               "measured": {"left": row} if self.arm_state in ("fresh", "stale") else {},
               "arms": {"left": {"body": "yam", "state": self.arm_state}}, "controller_state": self.state,
               "mixed_motion_left_version": 1}
        if self.lease and self.lease.get("active"):
            out["execution_lease"] = dict(self.lease)
        return out

    def get(self, path):
        if path == "/supervisor/pose":
            return 200, self.pose()
        if path == "/status":
            return 200, {"state": self.state, "arms": {"left": self.arm_state != "unavailable", "right": True},
                         "last_error": None, "max_ee_speed_m_s": 0.45}
        return 404, {"error": "no such route"}

    def post(self, path, body, headers):
        if path.startswith("/supervisor/") and headers.get("X-Manifest-Supervisor") != "1":
            return 403, {"error": "Use the local supervisor client"}
        if path == "/supervisor/hold":
            if not self.hold_route:
                return 404, {"error": "Unknown supervisor route"}
            if self.lease:
                self.lease["active"] = False
            self.state = "idle"                       # a hold interrupts a station Home / thermal blend
            row = {"arm": "left", "body": "yam", "state": "holding", "confirmed": True, "holding": True,
                   "positions_rad": self.q, "gripper": 0.999}
            return 200, {"schema": "manifest-station-hold/1", "version": 1, "hold": {"id": "h1", "generation": 1},
                         "actions": ["lease_ended"], "pending": False, "controller_state": "idle", "arms": {"left": row},
                         "parked_arms": [], "holding": True, "measured_monotonic_s": time.monotonic()}
        if path == "/supervisor/stop":
            if not self.lease or body.get("lease_id") != self.lease.get("id"):
                return 409, {"error": "ValueError: Unknown action lease; refusing to stop an unrelated run"}
            self.lease["active"] = False
            self.state = "idle"
            return 200, {"lease": self.lease}
        if path == "/stop":
            if self.state in ("homing", "resetting", "stopping"):
                return 409, {"error": f"stop refused: a {self.state} is in progress"}
            self.state = "idle"
            return 200, {"state": "idle"}
        return 404, {"error": "unexpected route in fake"}


class FakeMetalOwner(_Base):
    def __init__(self, *, state="holding", connected=True, motion=None, serial="SIMMETAL0001"):
        self.state, self.connected, self.motion, self.serial = state, connected, motion, serial
        self.q = [1.3669, -0.0005, -0.0256, 0.1507, -0.0563, 0.1022]
        super().__init__()

    def status(self):
        now = time.monotonic()
        row = {"positions_rad": self.q, "velocities_rad_s": [0.0024, 0, 0, 0, 0, 0], "motor_efforts_nm": [0.1] * 7,
               "motor_temps_c": [35] * 7, "motor_status": [1] * 7, "gripper_motor_rad": 1.9, "gripper": None,
               "commanded_rad": self.q if self.connected else None, "commanded_gripper_motor_rad": 1.9,
               "source_monotonic_s": now - 0.01, "timestamp_source": "can_query_roundtrip", "clock_uncertainty_s": 0.0,
               "body": "metal", "sample_id": f"metal:{self.serial}:{now:.9f}", "age_ms": 10.0, "fresh": True}
        return {"schema": "manifest-metal-owner/1", "body": "metal", "arm": "right", "state": self.state,
                "connected": self.connected, "jaw_connected": self.connected, "error": None, "maintenance": False,
                "adapter": {"channel": "can0", "usb_serial": self.serial}, "measured": {"right": row},
                "motion": self.motion, "motion_lease": None}

    def get(self, path):
        if path in ("/status", "/pose"):
            return 200, self.status()
        return 404, {"error": "Unknown route"}

    def post(self, path, body, headers):
        if headers.get("X-Manifest-Metal-Control") != "1" or "application/json" not in headers.get("Content-Type", ""):
            return 403, {"error": "Metal control requires a trusted local JSON request without a browser Origin"}
        if path == "/stop":
            self.motion = None
            if self.connected:
                self.state = "holding"
            return 200, self.status()
        if path == "/recover_hold":
            return 200, self.status()
        if path == "/jaw/set":
            return 200, {"lease_id": "x", "state": "running"}
        return 404, {"error": "Unknown route"}
