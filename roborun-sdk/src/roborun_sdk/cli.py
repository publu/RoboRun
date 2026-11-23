"""Developer commands: doctor, stop, run, rt, clear-stop. ``run`` and ``rt`` are simulated by default; ``--live`` targets the real owners.

  roborun doctor [--live]          checks: import guard, signing, gc, clock, sim self-test (+ live GET-only probes)
  roborun stop [--sim|--live]      this robot's Stop; --live: local roborun-rt Stop listener + every owner's hold in parallel
  roborun run SEGMENT [--live]     sim: play the segment on sim arms; live: acceptance on live samples (GET only)
  roborun rt [--live]              serve roborun-rt: tick loop, Stop listeners (UDP/HTTP :8111, SIGUSR1), link socket
  roborun clear-stop               clear a latched Stop in a running roborun-rt (operator + its console token)

``rt --live`` polls both owners with GETs. It POSTs their hold routes only on an explicit Stop (pedal, LAN UDP/HTTP,
SIGUSR1, ``roborun stop``, a relayed API Stop) or a ``hold`` control. Its automatic kernel holds (fault, tick
overrun, link loss) go out only while it commands an arm, and it cannot (the connectors do not support acquire/write),
so it never preempts the robot runtime's leases or the native thermal retreat on its own. Live rt never trusts the public
DEV keys: a DEV-signed pack may configure it (``--dev-keys-ok``), but no DEV-signed segment, control or pack offer
is accepted.

Exit codes: 0 ok, 1 usage/error, 2 refused/not confirmed, 3 segment ended without completing.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

from . import SDK_NAME, net

USER_HELP = """Roborun: run your robot arms (or a humanoid) from this computer.

Using an AI assistant? Have it read `roborun docs agents` first: everything it needs to run Roborun for you.

Start:
  roborun login                 sign this computer in: prints a link to approve on app.roborun.app
  roborun up --sim cup          no arms: two simulated YAM arms (scenes: cup, pickup-bench, mixed-objects, cluttered-bench,
                                low-light; a humanoid: roborun up --profile galaxea_r1 --sim room)
  roborun up                    your arms: a guided setup the first time, then Roborun starts. Open https://app.roborun.app
                                on this computer, or this robot's own viewer: roborun view
  roborun stop                  Stop now, like the Stop button;  roborun down: stop Roborun

Your arms: roborun setup (again any time), roborun hw detect | check | calibrate | motor | arms, roborun profiles.
Your own code: roborun new, roborun python FILE, the local API at http://127.0.0.1:8120/v1/docs, roborun docs.

Install: curl -fsSL https://roborun.app/install.sh | sh    Every command: roborun --help-all"""


def _manifest_install():
    """This machine's Roborun install: $MANIFEST_DREAMAPI, else the one `roborun setup` recorded, else ~/roborun/robot/runtime."""
    import json, os
    if os.environ.get("MANIFEST_DREAMAPI"):
        return Path(os.environ["MANIFEST_DREAMAPI"])
    cfg = net.roborun_home() / "robot.json"
    try:
        return Path(json.loads(cfg.read_text())["manifest"])
    except (OSError, ValueError, KeyError):
        return Path.home() / "roborun"


DREAMAPI = _manifest_install()
YAM_URL = "http://127.0.0.1:8091"
METAL_URL = "http://127.0.0.1:8093"
FIRST_POLL_TIMEOUT_S = 1.0
RT_MODULES = ("roborun_sdk.rt.runtime", "roborun_sdk.rt.server", "roborun_sdk.rt.stop_listeners",
              "roborun_sdk.connectors.yam", "roborun_sdk.connectors.metal", "roborun_sdk.connectors.sim",
              "roborun_sdk.connectors.hid")


def _print(obj, as_json: bool) -> None:
    if as_json:
        print(json.dumps(obj, indent=1, default=str))
        return
    def walk(o, indent=0):
        pad = "  " * indent
        if isinstance(o, dict):
            for k, v in o.items():
                if isinstance(v, (dict, list)) and v:
                    print(f"{pad}{k}:")
                    walk(v, indent + 1)
                else:
                    print(f"{pad}{k}: {v}")
        elif isinstance(o, list):
            for v in o:
                if isinstance(v, (dict, list)):
                    print(f"{pad}-")
                    walk(v, indent + 1)
                else:
                    print(f"{pad}- {v}")
        else:
            print(f"{pad}{o}")
    walk(obj)


# ---------------------------------------------------------------- doctor

def import_guard_check() -> dict:
    """Import every rt module in a fresh interpreter with the guard installed; report forbidden modules loaded."""
    src = str(Path(__file__).resolve().parents[1])
    code = ("import json,sys;sys.path.insert(0,%r);from roborun_sdk.rt import guard;guard.install();"
            "import importlib;[importlib.import_module(m) for m in %r];print(json.dumps(guard.loaded()))") % (src, RT_MODULES)
    proc = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        return {"ok": False, "error": proc.stderr.strip()[-400:]}
    loaded = json.loads(proc.stdout.strip().splitlines()[-1])
    return {"ok": not loaded, "forbidden_loaded": loaded, "modules": list(RT_MODULES)}


def _optional(name: str):
    try:
        mod = __import__(name)
        return getattr(mod, "__version__", "present")
    except Exception:
        return None


def sim_selftest() -> dict:
    """Accept and play a short generated move on sim arms, then Stop: both arms must report holding."""
    from .connectors.sim import SimWorld, drive
    from .dev import selftest_segment, sim_pack
    from .rt.clock import SimClock
    from .rt.runtime import Runtime
    pack = sim_pack()
    clock = SimClock()
    world = SimWorld.yam_metal(clock)
    rt = Runtime(world.arms, pack, clock=clock, armed_local=True, armed_server=True, trust_unknown_plan_basis=True)
    rt.session_started("doctor")
    drive(rt, world, 0.5)
    left = world.arms["left"]
    ack = rt.submit(selftest_segment(pack, "left", left.q.tolist(), left.jaw))
    drive(rt, world, 3.0, until=lambda r: r.playback is not None and r.playback.state == "ended")
    outcome = rt.playback.outcome if rt.playback else None
    stop = rt.stop("doctor")
    return {"ok": ack.get("result") == "accepted" and outcome == "complete" and not stop["not_confirmed"],
            "ack": ack.get("result"), "reason": ack.get("reason"), "outcome": outcome,
            "stop_holding": {a: v["holding"] for a, v in stop["arms"].items()}}


def _hold_route(yam) -> dict:
    """GET /supervisor/hold (the station's hold route, read-only): installed or not."""
    from .connectors.local_http import OwnerError
    from .rt.datatypes import ConnectorError
    try:
        st = yam.http.get("/supervisor/hold", timeout_s=0.3)
        return {"installed": st.get("schema") == "manifest-station-hold-status/1", "version": st.get("version"),
                "generation": st.get("generation"), "installed_wraps": st.get("installed")}
    except OwnerError as exc:
        return {"installed": False, "status": exc.status, "why": exc.message[:120]}
    except ConnectorError as exc:
        return {"installed": None, "why": str(exc)[:120]}


def live_connectors(clock, *, allow_post: bool, yam_url: str = YAM_URL, metal_url: str = METAL_URL,
                    sysroot: Path = Path("/sys"), first_poll_timeout_s: float = FIRST_POLL_TIMEOUT_S) -> dict:
    """Both owners' connectors with their pollers started. Returns once each first poll has answered (or the timeout
    passed), so the first tick never judges an owner that was never asked. Shared by ``rt --live``, ``run --live``
    and the tests."""
    from .connectors.metal import MetalOwnerConnector
    from .connectors.yam import YamStationConnector
    conns = {"left": YamStationConnector(yam_url, allow_post=allow_post, clock=clock, sysroot=sysroot),
             "right": MetalOwnerConnector(metal_url, allow_post=allow_post, clock=clock)}
    for c in conns.values():
        c.start()
    deadline = time.monotonic() + first_poll_timeout_s
    for c in conns.values():
        c.wait_first_poll(deadline - time.monotonic())
    return conns


def live_runtime(conns: dict, pack, *, clock, proximity=None, interlocks=None):
    """The runtime ``rt --live`` serves: WSS link rules, both arming halves off, the DEV keys refused."""
    from .dev import DEV_KIDS
    from .rt.runtime import Runtime
    return Runtime(conns, pack, clock=clock, proximity=proximity, interlocks=interlocks, transport="wss",
                   refused_kids=DEV_KIDS)


def live_probe(dreamapi: Path) -> dict:
    """GET-only probes of the owners and a read-only camera broker status."""
    from .connectors.metal import MetalOwnerConnector
    from .connectors.realsense import CameraServiceClient, DEFAULT_SESSION_FILE
    from .connectors.yam import YamStationConnector, active_leases
    out = {}
    yam = YamStationConnector(allow_post=False, offline_after_s=0.0)      # one-shot probe: a failed poll is offline
    s = yam.poll_once()
    pose = yam.last_status() or {}
    out["left_yam"] = {"health": yam.health().__dict__, "controller_state": pose.get("controller_state"),
                       "sample_age_ms": None if s is None else round((time.monotonic() - s.t) * 1000, 1),
                       "q": None if s is None else [round(v, 4) for v in s.q], "jaw": None if s is None else s.jaw,
                       "temp_max_c": None if s is None or not s.temps_c else max(s.temps_c),
                       "leases": active_leases(pose), "vendor_status": {k: (yam._vendor or {}).get(k) for k in ("state", "arms", "last_error", "max_ee_speed_m_s")},
                       "lease_free_hold_route": _hold_route(yam),
                       "posts_sent": yam.http.posts()}
    metal = MetalOwnerConnector(allow_post=False, offline_after_s=0.0)
    s = metal.poll_once()
    st = metal.last_status() or {}
    out["right_metal"] = {"health": metal.health().__dict__, "state": st.get("state"), "connected": st.get("connected"),
                          "jaw_connected": st.get("jaw_connected"), "adapter": st.get("adapter"), "error": st.get("error"),
                          "sample_age_ms": None if s is None else round((time.monotonic() - s.t) * 1000, 1),
                          "q": None if s is None else [round(v, 4) for v in s.q],
                          "temp_max_c": None if s is None or not s.temps_c else max(s.temps_c),
                          "posts_sent": metal.http.posts()}
    try:
        cam = CameraServiceClient(session_file=DEFAULT_SESSION_FILE).status()
        out["camera_broker"] = {"state": cam.get("state"), "roles": cam.get("roles"),
                                "camera_session_id": cam.get("camera_session_id"), "errors": cam.get("errors")}
    except Exception as exc:
        out["camera_broker"] = {"error": f"{type(exc).__name__}: {exc}"[:200]}
    out["streaming"] = {"wired": False, "why": "these connectors do not command motion"}
    out["mixed_safety"] = {"available": (dreamapi / "mixed_safety.py").is_file()}
    return out


def cmd_doctor(args) -> int:
    import gc
    from .rt import signing
    from .rt.clock import Clock
    report = {"sdk": SDK_NAME, "python": sys.version.split()[0],
              "deps": {"numpy": _optional("numpy"), "msgpack": _optional("msgpack"), "websockets": _optional("websockets"),
                       "zstandard": _optional("zstandard"), "PIL": _optional("PIL"), "cryptography": _optional("cryptography")},
              "import_guard": import_guard_check(), "signing": {"backend": signing.BACKEND}}
    from .rt import wire
    key = signing.SigningKey("doctor", os.urandom(32))
    blob = os.urandom(4096)
    t0 = time.perf_counter()
    sig = key.sign_bytes(blob)
    t1 = time.perf_counter()
    ok = signing._verify_raw(key.public, blob, sig) and not signing._verify_raw(key.public, blob + b"x", sig)
    report["signing"].update(roundtrip=ok, sign_ms=round((t1 - t0) * 1000, 2),
                             verify_ms=round((time.perf_counter() - t1) * 1000, 2),
                             payload="manifest_wire.signing_payload" if wire.available() else
                             "canonical JSON of the raw message (wire schema package not importable)")
    report["gc"] = {"freeze": hasattr(gc, "freeze")}
    report["clock"] = Clock().describe()
    report["sim_selftest"] = sim_selftest()
    if args.live:
        report["live"] = live_probe(Path(args.dreamapi))
    checks = {"safety runtime imports": report["import_guard"]["ok"], "signature check": report["signing"]["roundtrip"],
              "simulated Stop self-test": report["sim_selftest"]["ok"]}
    failed = [name for name, passed in checks.items() if not passed]
    report = {"summary": "OK: " + ", ".join(checks) + " passed" if not failed else
              f"{len(failed)} problem(s): " + ", ".join(failed) + " failed (details below)", **report}
    if args.json:
        _print(report, True)
    else:
        print(report["summary"])
        if failed or args.verbose:
            _print({k: v for k, v in report.items() if k != "summary"}, False)
        else:
            print("details: roborun doctor --verbose")
    return 0 if not failed else 2


# ---------------------------------------------------------------- stop

def _rt_stop(port: int, timeout_s: float = 0.3) -> dict:
    """Signal a local roborun-rt (UDP fire-and-forget, then POST /stop). Absent rt is not an error."""
    import http.client
    from .rt.stop_listeners import send_udp_stop
    out = {}
    try:
        send_udp_stop("127.0.0.1", port)
        out["udp"] = "sent"
    except OSError as exc:
        out["udp"] = f"error: {exc}"
    try:
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=timeout_s)
        conn.request("POST", "/stop", body=b"{}", headers={"Content-Type": "application/json"})
        resp = conn.getresponse()
        out["http"] = {"status": resp.status, "body": json.loads(resp.read() or b"{}")}
        conn.close()
    except (OSError, ValueError) as exc:
        out["http"] = f"no roborun-rt listening ({type(exc).__name__})"
    return out


def _stop_this_robot(args) -> int:
    """`roborun stop`: the same Stop as the button in the app, sent to the robot on this computer."""
    import urllib.error
    import urllib.request
    from . import robot
    backend = "http://127.0.0.1:8120"
    if robot.CONFIG.exists():
        backend = json.loads(robot.CONFIG.read_text()).get("backend") or backend
    request = urllib.request.Request(backend + "/api/stop", data=b"{}", method="POST", headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=10) as reply:
            body = json.loads(reply.read() or b"{}")
    except urllib.error.HTTPError as exc:
        print(f"the robot refused or could not confirm Stop (HTTP {exc.code}). If the arm is moving, use its power switch.")
        return 2
    except (OSError, ValueError):
        print("Roborun is not running on this computer, so there is nothing to stop.")
        return 0
    _print(body, True) if args.json else print("Stop sent: the robot is holding where it is.")
    return 0


def cmd_stop(args) -> int:
    from .rt.clock import Clock
    from .rt.kernel.stop import StopLatch
    if not args.live and not args.sim:
        return _stop_this_robot(args)
    if not args.live:
        from .connectors.sim import SimWorld
        from .dev import sim_pack
        from .rt.clock import SimClock
        from .rt.runtime import Runtime
        clock = SimClock()
        world = SimWorld.yam_metal(clock)
        rt = Runtime(world.arms, sim_pack(), clock=clock)
        report = rt.stop("cli")
        _print({"mode": "SIM: no robot was contacted (use --live)", "report": report}, args.json)
        return 0
    from .connectors.metal import MetalOwnerConnector
    from .connectors.yam import YamStationConnector
    connectors = {"left": YamStationConnector(allow_post=True), "right": MetalOwnerConnector(allow_post=True)}
    latch = StopLatch(Clock())
    started = time.perf_counter()
    rt_report = None
    rt_holder: dict = {}
    if not args.no_rt:
        t = threading.Thread(target=lambda: rt_holder.update(_rt_stop(args.rt_port)), daemon=True)
        t.start()
    report = latch.trigger("cli", connectors=connectors)
    late = {}
    if report["not_confirmed"] and args.wait_s > 0:
        # owners answer after the 300 ms budget sometimes (a supervisor stop waits for the loop): report them late
        deadline = time.perf_counter() + args.wait_s
        for arm in report["not_confirmed"]:
            job = latch._inflight.get(arm)
            if job is not None:
                job.done.wait(max(0.0, deadline - time.perf_counter()))
                if job.done.is_set() and job.ack is not None:
                    late[arm] = {**job.ack.public(), "late": True}
    if not args.no_rt:
        t.join(1.0)
        rt_report = rt_holder
    out = {"mode": "LIVE", "elapsed_ms": round((time.perf_counter() - started) * 1000, 1), "arms": report["arms"],
           "not_confirmed": report["not_confirmed"], "late_acks": late, "manifest_rt": rt_report}
    _print(out, args.json)
    confirmed = all(v.get("confirmed") or late.get(a, {}).get("holding") for a, v in report["arms"].items())
    return 0 if confirmed else 2


# ---------------------------------------------------------------- run

def _load_json(path: str) -> dict:
    from .rt.canonical import from_jsonable
    return from_jsonable(json.loads(Path(path).read_text()))


def _load_pack(args, live: bool):
    from .dev import DEV_KIDS, dev_root_keys, sim_pack, sim_pack_raw
    from .rt.pack import load_pack
    from .rt.signing import keys_from_trusted
    raw = _load_json(args.pack) if args.pack else sim_pack_raw()
    if args.root_keys:
        keys = keys_from_trusted(json.loads(Path(args.root_keys).read_text()))
    else:
        keys = dev_root_keys()
    if live and not args.root_keys and not args.dev_keys_ok:
        raise SystemExit("run --live needs --root-keys (the API's pinned root key) or an explicit --dev-keys-ok")
    if live and set(keys) & set(DEV_KIDS) and not args.dev_keys_ok:
        raise SystemExit("refusing DEV keys in live mode without --dev-keys-ok")
    return load_pack(raw, keys, allow_unsigned=not live and args.allow_unsigned)


def cmd_run(args) -> int:
    from .rt.clock import Clock, SimClock
    from .rt.kernel.proximity import CapsuleProximity, MixedSafetyProximity
    from .rt.runtime import Runtime
    if (args.segment is None) == (args.from_current is None):
        raise SystemExit("give a SEGMENT file or --from-current ARM")
    seg = _load_json(args.segment) if args.segment else None
    pack = _load_pack(args, args.live)
    if args.geometry == "mixed-safety" or (args.geometry == "auto" and args.live):
        proximity = MixedSafetyProximity(args.dreamapi)
    else:
        proximity = CapsuleProximity(pack)
    if not args.live:
        from .connectors.sim import SimArm, SimWorld, drive
        clock = SimClock()
        if seg is None:
            from .dev import selftest_segment
            arm = args.from_current
            homes = {n: (list(a.home_q) if a.home_q is not None else None) for n, a in pack.arms.items()}
            seg = selftest_segment(pack, arm, homes[arm], 0.999 if arm == "left" else 1.9)
        arm = seg.get("arm")
        vi = seg.get("valid_if") or {}
        homes = {n: (list(a.home_q) if a.home_q is not None else None) for n, a in pack.arms.items()}
        start = {n: q for n, q in homes.items()}
        if vi.get("start_q") is not None:
            start[arm] = vi["start_q"]
        world = SimWorld.yam_metal(clock, left_q=start.get("left"), right_q=start.get("right"),
                                   left_jaw=vi.get("jaw", 0.999) if arm == "left" else 0.999,
                                   right_jaw=vi.get("jaw", 1.9) if arm == "right" else 1.9)
        rt = Runtime(world.arms, pack, clock=clock, proximity=proximity, armed_local=True, armed_server=True,
                     trust_unknown_plan_basis=True, allow_unsigned_segments=args.allow_unsigned)
        rt.session_started("cli-sim")
        drive(rt, world, 0.5)
        basis = (seg.get("context") or {}).get("plan_basis_obs")
        if basis:
            rt.register_observation(basis, clock.now())
            drive(rt, world, 0.04)
        ack = rt.submit(seg)
        if ack.get("result") == "accepted":
            if args.realtime:
                for _ in range(int(args.seconds * rt.rate_hz)):
                    clock.advance(rt.dt)
                    world.step(rt.dt)
                    rt.link_heartbeat()
                    rt.tick()
                    time.sleep(rt.dt)
                    if rt.playback is not None and rt.playback.state == "ended":
                        break
            else:
                drive(rt, world, args.seconds, until=lambda r: r.playback is not None and r.playback.state == "ended")
        events = rt.drain_events()
        gate = [e for e in events if e.get("type") == "gate_report"]
        out = {"mode": "SIM", "pack": pack.digest, "geometry": proximity.name, "ack": ack,
               "events": [e for e in events if e.get("type") != "gate_report"],
               "gate_report": gate[-1] if gate else None,
               "final": {n: [round(v, 4) for v in a.q] for n, a in world.arms.items()}}
        _print(out, args.json)
        if ack.get("result") != "accepted":
            return 2
        return 0 if gate and gate[-1]["outcome"] == "complete" else 3
    # ---- live: GET-only acceptance against the real samples; the connectors cannot acquire an arm
    clock = Clock()
    conns = live_connectors(clock, allow_post=False)
    try:
        rt = Runtime(conns, pack, clock=clock, proximity=proximity, armed_local=True, armed_server=True,
                     trust_unknown_plan_basis=True)
        if hasattr(proximity, "warm"):
            proximity.warm()
        rt.session_started("cli-live")
        stop = threading.Event()
        th = threading.Thread(target=rt.run, args=(stop,), kwargs={"feed_local_link": True}, daemon=True)
        th.start()
        time.sleep(0.8)                         # 0.3 s rest window + freshness
        if seg is None:
            # a 0.02 rad joint-1 move from where the arm rests now: exercises sampling, the rest window, K3 and
            # K4 (the runtime's mixed-arm check) on live samples; nothing moves (the connectors cannot acquire an arm)
            from .dev import selftest_segment
            from .rt.acceptance import jaw_reference
            s_now = conns[args.from_current].sample()
            # no jaw reading: a placeholder row value; acceptance refuses the start as jaw_unknown
            jaw_now = 0.0 if s_now.jaw is None else jaw_reference(pack.arm(args.from_current), s_now, None)
            seg = selftest_segment(pack, args.from_current, list(s_now.q), jaw_now, delta_rad=0.02,
                                   measured_jaw=s_now.jaw)
        ack = rt.submit(seg)
        stop.set()
        th.join(2.0)
        out = {"mode": "LIVE (acceptance only; these connectors do not move the arms)", "pack": pack.digest,
               "geometry": proximity.name, "ack": ack, "plan_basis": "not checked (segment file)",
               "posts_sent": {n: c.http.posts() for n, c in conns.items()}, "events": rt.drain_events()}
        _print(out, args.json)
        return 0 if ack.get("acceptance") == "passed" else 2
    finally:
        for c in conns.values():
            c.close()


# ---------------------------------------------------------------- rt (the roborun-rt process)

def cmd_rt(args) -> int:
    """Serve roborun-rt: runtime tick loop + Stop listeners (UDP/HTTP/SIGUSR1, optional pedal) + the link socket.
    Live: connectors poll the owners (GET). They POST their hold routes only for an explicit Stop or a hold control,
    plus automatic holds while rt commands an arm (never: the connectors do not support acquire/write)."""
    import secrets
    import signal as _signal
    from .rt import guard
    guard.install()
    from .rt.clock import Clock
    from .rt.kernel.interlocks import Interlocks
    from .rt.kernel.proximity import CapsuleProximity, MixedSafetyProximity
    from .rt.runtime import Runtime
    from .rt.server import RtServer
    from .rt.stop_listeners import StopListeners
    pack = _load_pack(args, args.live)
    clock = Clock()
    world = None
    if args.live:
        conns = live_connectors(clock, allow_post=True, yam_url=args.yam_url, metal_url=args.metal_url,
                                sysroot=Path(args.sysroot))
    else:
        from .connectors.sim import SimWorld
        world = SimWorld.yam_metal(clock)
        conns = world.arms
    proximity = (MixedSafetyProximity(args.dreamapi) if args.geometry == "mixed-safety" or (args.geometry == "auto" and args.live)
                 else CapsuleProximity(pack))
    interlocks = Interlocks(args.interlocks, conns)
    seeded = interlocks.seed_from_jaw_memory(args.seed_jaw_memory) if args.seed_jaw_memory else []
    if args.live:
        rt = live_runtime(conns, pack, clock=clock, proximity=proximity, interlocks=interlocks)
    else:
        rt = Runtime(conns, pack, clock=clock, proximity=proximity, interlocks=interlocks, transport="wss")
    if args.cpu is not None and hasattr(os, "sched_setaffinity"):
        os.sched_setaffinity(0, {args.cpu})
    console_token = secrets.token_hex(18)          # printed on this console only: `roborun clear-stop --token`
    server = RtServer(rt, args.socket, console_token=console_token).start()
    listeners = StopListeners(rt, bind=args.stop_bind, udp_port=args.stop_port, http_port=args.stop_port,
                              sigusr1=True).start()
    pedal = None
    if args.pedal:
        from .connectors.hid import HidPedal
        pedal = HidPedal(args.pedal)
        pedal.on_press(lambda src: rt.stop(src))
        pedal.start()
    stop_event = threading.Event()
    for sig in (_signal.SIGINT, _signal.SIGTERM):
        _signal.signal(sig, lambda *_: stop_event.set())
    if world is not None:
        def step_world():
            while not stop_event.is_set():
                world.step(rt.dt)
                time.sleep(rt.dt)
        threading.Thread(target=step_world, daemon=True).start()
    if args.seconds:
        threading.Timer(args.seconds, stop_event.set).start()
    if hasattr(proximity, "warm"):
        proximity.warm()
    guard.freeze_heap()
    print(json.dumps({"manifest_rt": "running", "mode": "LIVE" if args.live else "SIM", "pack": pack.digest,
                      "socket": args.socket, "stop_udp": listeners.udp_port, "stop_http": listeners.http_port,
                      "stop_bind": args.stop_bind, "geometry": proximity.name, "interlocks_seeded": seeded,
                      "refused_kids": sorted(rt.refused_kids), "console_token": console_token,
                      "forbidden_modules": guard.loaded()}), flush=True)
    try:
        rt.run(stop_event)
    finally:
        listeners.close()
        server.close()
        if pedal is not None:
            pedal.close()
        for c in conns.values():
            if hasattr(c, "close"):
                c.close()
    print(json.dumps({"manifest_rt": "exited", "ticks": rt.tick_count, "stops": rt.stop_latch.count,
                      "events": len(rt.events)}), flush=True)
    return 0


# ---------------------------------------------------------------- clear-stop

def cmd_clear_stop(args) -> int:
    """A local operator clears a latched Stop in a running roborun-rt (needs its console token). Starts nothing."""
    from .link.rtclient import RtClient
    from .rt.datatypes import Refused
    client = RtClient(args.socket)
    try:
        rec = client.clear_stop(args.operator, args.token)
    except Refused as r:
        _print({"cleared": False, "reason": r.reason, "message": r.message}, args.json)
        return 2
    except OSError as exc:
        _print({"cleared": False, "reason": "no_manifest_rt", "message": str(exc)[:200]}, args.json)
        return 1
    finally:
        client.close()
    _print({"cleared": rec}, args.json)
    return 0


# what `roborun --help` lists; every other command works as before (Roborun's own services start some of them), and only
# `roborun --help-all` lists them
USER_COMMANDS = ("login", "up", "down", "status", "stop", "setup", "view", "new", "docs", "python", "upgrade", "uninstall",
                 "hw", "profiles")
DEVELOPER_COMMANDS = ("doctor", "run", "rt", "clear-stop", "local", "sync", "station", "hardware-station", "body", "sim",
                      "cameras", "settings", "profile", "arms", "arm-check", "arm-calibrate", "calibrate-gripper", "motor",
                      "adapters")
# roborun hw NAME: the hardware commands, each the same parser as its older spelling (which keeps working)
HW = (("detect", None, "what is connected (adapters with their ADDRESS, cameras) and which arms it could be; asks and moves nothing"),
      ("check", "arm-check", "read-only check of an arm: nothing is enabled or moved (unless --move)"),
      ("calibrate", "arm-calibrate", "record an arm's zero, ranges and gripper by hand, with torque off (YAM jaw: --gripper)"),
      ("motor", "motor", "motors on a bus: list them (read-only), change an id, set a zero (each asks first)"),
      ("arms", "arms", "the arms Roborun can drive"))


def _alias(action, name, parser, text, prog):
    """Make `parser` also answer as `name` under the subcommands `action` (its help listed there; its usage says `prog`)."""
    action._name_parser_map[name] = parser
    action._choices_actions.append(argparse._SubParsersAction._ChoicesPseudoAction(name, (), text))
    old = parser.prog
    parser.prog = prog
    for sub in getattr(parser, '_subparsers', None) and parser._subparsers._group_actions or ():
        for child in sub.choices.values():
            child.prog = child.prog.replace(old, prog, 1)


def add_hw_and_profiles(sub):
    hw = sub.add_parser("hw", help="the hardware checks: detect, check, calibrate, motor, arms",
                        description="Your arms and cameras, one check at a time (roborun setup runs them for you).")
    hws = hw.add_subparsers(dest="hw_cmd", required=True, metavar="CHECK")
    d = hws.add_parser("detect", help=HW[0][2])
    d.add_argument("--json", action="store_true")
    d.set_defaults(func=lambda a: __import__("roborun_sdk.guided", fromlist=["detect"]).detect(a.json))
    for name, old, text in HW[1:]:
        _alias(hws, name, sub.choices[old], text, f"roborun hw {name}")
    profiles = sub.choices["profiles"]
    ps = profiles.add_subparsers(dest="profiles_cmd", metavar="create")
    _alias(ps, "create", sub.choices["profile"]._subparsers._group_actions[0].choices["create"],
           "a profile for your own arm, from its URDF", "roborun profiles create")


def main(argv=None) -> int:
    from roborun_sdk import devkit
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv[:1] == ["python"]:                       # everything after it is Python's own arguments
        return devkit.run_python(argv[1:])
    show_all = "--help-all" in argv
    if show_all:
        argv = ["--help"]
    ap = argparse.ArgumentParser(prog="roborun", description=USER_HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", action=devkit.VersionAction)
    sub = ap.add_subparsers(dest="cmd", required=True, metavar="COMMAND")
    d = sub.add_parser("doctor", help="health and safety-rule checks")
    d.add_argument("--live", action="store_true", help="also probe the real owners (GET only)")
    d.add_argument("--runtime", dest="dreamapi", metavar="DIR", default=str(DREAMAPI), help="this robot's Roborun install")
    d.add_argument("--dreamapi", dest="dreamapi", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    d.add_argument("--json", action="store_true")
    d.add_argument("--verbose", action="store_true", help="print every check's details")
    s = sub.add_parser("stop", help="stop the robot on this computer now, like the Stop button in the app")
    s.add_argument("--live", action="store_true", help="developer: hold the arm owners directly (POST their hold/stop routes)")
    s.add_argument("--sim", action="store_true", help="developer: exercise Stop on simulated arms; no robot is contacted")
    s.add_argument("--rt-port", type=int, default=8111)
    s.add_argument("--no-rt", action="store_true", help="do not signal the local safety runtime")
    s.add_argument("--wait-s", type=float, default=1.5, help="wait this long for late acks after the 300 ms budget")
    s.add_argument("--json", action="store_true")
    r = sub.add_parser("run", help="play a segment file (sim by default)")
    r.add_argument("segment", nargs="?")
    r.add_argument("--from-current", choices=("left", "right"),
                   help="instead of a file: a small DEV-signed move from the arm's present pose (acceptance check)")
    r.add_argument("--pack", help="safety pack JSON (default: the bundled DEV sim pack)")
    r.add_argument("--live", action="store_true", help="acceptance against the real owners' samples (GET only)")
    r.add_argument("--root-keys", help="trusted root keys JSON list [{kid, alg, pub}]")
    r.add_argument("--dev-keys-ok", action="store_true", help="accept DEV-signed packs in live mode")
    r.add_argument("--allow-unsigned", action="store_true", help="sim only: accept unsigned packs/segments")
    r.add_argument("--geometry", choices=("auto", "capsule", "mixed-safety"), default="auto")
    r.add_argument("--runtime", dest="dreamapi", metavar="DIR", default=str(DREAMAPI), help="this robot's Roborun install")
    r.add_argument("--dreamapi", dest="dreamapi", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    r.add_argument("--seconds", type=float, default=20.0)
    r.add_argument("--realtime", action="store_true", help="sim at wall-clock pace")
    r.add_argument("--json", action="store_true")
    t = sub.add_parser("rt", help="serve the local safety runtime (sim by default)")
    t.add_argument("--live", action="store_true",
                   help="real owners: GET polling; hold routes are POSTed only on an explicit Stop or hold control")
    t.add_argument("--pack")
    t.add_argument("--root-keys")
    t.add_argument("--dev-keys-ok", action="store_true")
    t.add_argument("--allow-unsigned", action="store_true")
    t.add_argument("--geometry", choices=("auto", "capsule", "mixed-safety"), default="auto")
    t.add_argument("--runtime", dest="dreamapi", metavar="DIR", default=str(DREAMAPI), help="this robot's Roborun install")
    t.add_argument("--dreamapi", dest="dreamapi", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    t.add_argument("--socket", default=os.path.join(os.environ.get("XDG_RUNTIME_DIR", "/tmp"), "manifest-rt.sock"))
    t.add_argument("--stop-bind", default="127.0.0.1", help="0.0.0.0 exposes the LAN Stop (no auth, by design)")
    t.add_argument("--stop-port", type=int, default=8111)
    t.add_argument("--pedal", help="input event device of a USB Stop pedal")
    t.add_argument("--interlocks", help="persisted K8 bits (JSON)")
    t.add_argument("--seed-jaw-memory", help="the robot runtime's jaw memory file to seed K8 from (read-only)")
    t.add_argument("--cpu", type=int, help="pin the safety runtime to this CPU")
    t.add_argument("--seconds", type=float, help="exit after this long (tests)")
    t.add_argument("--yam-url", default=YAM_URL, help=argparse.SUPPRESS)          # drills against fakes only
    t.add_argument("--metal-url", default=METAL_URL, help=argparse.SUPPRESS)
    t.add_argument("--sysroot", default="/sys", help=argparse.SUPPRESS)
    c = sub.add_parser("clear-stop", help="clear a latched Stop in the running safety runtime (local operator)")
    c.add_argument("--operator", required=True, help="who clears it (logged)")
    c.add_argument("--token", required=True, help="the console_token `roborun rt` printed when it started")
    c.add_argument("--socket", default=os.path.join(os.environ.get("XDG_RUNTIME_DIR", "/tmp"), "manifest-rt.sock"))
    c.add_argument("--json", action="store_true")
    from roborun_sdk import robot, embodiment, profile_create, mujoco_sim
    robot.add_parsers(sub)
    embodiment.add_parser(sub)
    profile_create.add_parser(sub)
    mujoco_sim.add_parser(sub)
    from roborun_sdk.body import server as body_server
    body_server.add_parser(sub)
    from roborun_sdk import scaffold
    scaffold.add_parser(sub)
    devkit.add_parsers(sub)
    from roborun_sdk import login
    login.add_parser(sub)
    from roborun_sdk import cameras
    cameras.add_parser(sub)
    from roborun_sdk.arms import hardware as arm_hardware
    arm_hardware.add_parsers(sub)
    from roborun_sdk.arms import calibrate as arm_calibrate
    arm_calibrate.add_parser(sub)
    from roborun_sdk.arms import connect as arm_connect
    arm_connect.add_parsers(sub)                      # last: it wraps the arm commands and `up` with plain failure messages
    add_hw_and_profiles(sub)
    if not show_all:                                  # the other commands still run; --help-all lists them
        sub._choices_actions[:] = sorted((c for c in sub._choices_actions if c.dest in USER_COMMANDS),
                                         key=lambda c: USER_COMMANDS.index(c.dest))
    args = ap.parse_args(argv)
    run = args.func if getattr(args, "func", None) is not None else {
        "doctor": cmd_doctor, "stop": cmd_stop, "run": cmd_run, "rt": cmd_rt, "clear-stop": cmd_clear_stop}[args.cmd]
    try:
        return run(args)
    except ModuleNotFoundError as exc:               # a light install (pip install roborun-sdk): name the extra to add
        from roborun_sdk._optional import explain
        hint = explain(exc)
        if hint is None:
            raise
        print(f"roborun {args.cmd}: {hint}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
