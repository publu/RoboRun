"""roborun-station: the owner semantics, the lease-free hold, every fault kind, the HTTP front ends."""
import json
import math
import re
import subprocess
import sys
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from roborun_station import (VELOCITY_QUANTUM, CameraLost, FakeCamera, FakeMetalOwner, FakeYamStation,  # noqa: E402
                                    Faults, OwnerDead, OwnerError, SimClock, SimServer, StationError)
from roborun_station import drills  # noqa: E402


def leased(station, seconds=8):
    session = station.register_session(seconds=seconds)
    oid = station.observe()["observation_id"]
    return station.action({"observation_id": oid, "seconds": seconds, "instruction": "x",
                           "reference": {"kind": "video_motion", "session_id": session}})["lease"]["id"]


class Arms(unittest.TestCase):
    def test_at_rest_velocity_is_quantised_noise_but_positions_are_still(self):
        """The reason the rest test is a position window, not a velocity threshold."""
        clock = SimClock(); st = FakeYamStation(clock=clock)
        window, speeds = [], []
        for _ in range(12):                                    # 0.4 s of driver cycles at 30 Hz
            st.advance(1/30)
            row = st.left.row(clock())
            window.append(row["positions_rad"]); speeds.append(max(abs(v) for v in row["velocities_rad_s"]))
        joint_range = max(max(c) - min(c) for c in zip(*window))
        self.assertLessEqual(joint_range, 0.0015)
        self.assertGreater(max(speeds), 1e-4)                  # the old |v| <= 1e-4 rule would refuse
        for v in speeds:
            self.assertAlmostEqual(v / VELOCITY_QUANTUM, round(v / VELOCITY_QUANTUM), places=6)

    def test_rows_carry_the_safety_check_provenance(self):
        clock = SimClock(); st = FakeYamStation(clock=clock); st.advance(0.1)
        row = st.pose()["measured"]["left"]
        for key in ("body", "positions_rad", "gripper", "commanded_rad", "commanded_gripper", "source_monotonic_s",
                    "sample_id", "timestamp_source", "clock_uncertainty_s", "motor_temps_c"):
            self.assertIn(key, row)
        self.assertEqual((row["body"], row["timestamp_source"]), ("yam", "driver_state_cycle"))
        m = FakeMetalOwner(clock=clock); m.advance(0.1)
        r = m.status()["measured"]["right"]
        self.assertEqual((r["body"], r["timestamp_source"]), ("metal", "can_query_roundtrip"))
        self.assertTrue(r["sample_id"].startswith("metal:SIMMETAL0001:"))


class Faulted(unittest.TestCase):
    def setUp(self):
        self.clock = SimClock(); self.faults = Faults()
        self.st = FakeYamStation(clock=self.clock, faults=self.faults)
        self.st.advance(0.2)

    def test_stale_and_frozen_samples(self):
        self.faults.add("stale", self.clock(), 1.0, arm="left", extra_s=0.12)
        self.st.advance(0.1)
        self.assertGreaterEqual(self.st.pose()["measured"]["left"]["age_ms"], 119)
        self.faults.clear("stale")
        self.faults.add("freeze", self.clock(), 0.5, arm="left")
        stamp = self.st.pose()["measured"]["left"]["source_monotonic_s"]
        self.st.advance(0.3)
        row = self.st.pose()["measured"]["left"]
        self.assertEqual(row["source_monotonic_s"], stamp)
        self.assertFalse(row["fresh"])
        self.st.advance(0.3)
        self.assertGreater(self.st.pose()["measured"]["left"]["source_monotonic_s"], stamp)

    def test_gc_pause_stops_ticks(self):
        lease = leased(self.st)
        self.faults.add("gc_pause", self.clock(), 3.0)
        self.st.advance(3.0)                                   # 3 s without a heartbeat, but the process was paused
        self.assertTrue(self.st.lease["active"])
        self.st.advance(2.6)                                   # now the watchdog runs: no heartbeat for 5.6 s
        self.assertEqual(self.st.lease["reason"], "client_disconnected")
        self.assertIsNotNone(lease)

    def test_usb_drop_faults_the_station(self):
        leased(self.st)
        self.faults.add("usb_drop", self.clock(), None, arm="left")
        self.st.advance(0.1)
        self.assertEqual(self.st.state, "error")
        self.assertEqual(self.st.lease["reason"], "station_error")
        self.assertIn("left", self.st.status()["dead_arms"])
        self.assertIn(self.st.pose()["arms"]["left"]["state"], ("stale", "unavailable"))
        with self.assertRaises(StationError):
            self.st.home()

    def test_stall_blocks_the_joints(self):
        self.faults.add("stall", self.clock(), None, arm="left")
        q0 = list(self.st.left.q)
        self.st.left.command(q=[q0[0] + 0.2] + q0[1:])
        self.st.advance(0.5)
        self.assertLess(abs(self.st.left.q[0] - q0[0]), 1e-9)
        self.assertGreater(self.st.left.q_cmd[0] - self.st.left.q[0], 0.19)


class StationDefaults(unittest.TestCase):
    """A station without the lease-free hold: the gaps its Stop must close."""

    def test_supervisor_stop_needs_the_lease_id(self):
        clock = SimClock(); st = FakeYamStation(clock=clock, predict=drills.sweeping_predict(clock))
        lease = leased(st)
        with self.assertRaisesRegex(StationError, "Unknown action lease"):
            st.supervisor_stop({})
        st.supervisor_stop({"lease_id": lease})
        self.assertEqual(st.lease["reason"], "supervisor_stop")

    def test_vendor_stop_is_refused_while_homing_and_resetting(self):
        for state, text in (("home", "home move"), ("thermal_blend", "home move"), ("reset_home", "home move"),
                            ("reset_reopen", "reset")):
            clock, st, _ = drills.yam_in_state(state, lease_free_hold=False)
            with self.assertRaisesRegex(StationError, f"stop refused: a {text}"):
                st.stop()
            self.assertFalse(drills.judge_hold(st, [st.left])["left"]["holding"], state)

    def test_home_opens_the_grippers_and_auto_home_is_suppressed_under_a_lease(self):
        clock = SimClock(); st = FakeYamStation(clock=clock, predict=drills.sweeping_predict(clock))
        st.left.command(jaw=0.6); st.advance(1.0)
        leased(st)
        self.assertFalse(st.auto_home("budget_cap"))
        self.assertEqual(st.state, "running")
        st.supervisor_stop({"lease_id": st.lease["id"]})
        st.home()
        self.assertEqual(st.left.jaw_cmd, 1.0)                 # Home opens the gripper
        st.advance(3.5)
        self.assertEqual(st.state, "idle")
        self.assertTrue(st.auto_home("key_refused"))           # unsupervised: it homes, grippers open

    def test_motion_lease_streams_and_refuses_an_excessive_answer(self):
        clock = SimClock(); st = FakeYamStation(clock=clock, predict=drills.sweeping_predict(clock))
        lease = leased(st)
        q0 = st.left.q[0]
        for _ in range(10):
            st.advance(0.2); st.heartbeat({"lease_id": lease})
        self.assertGreater(st.left.q[0] - q0, 0.3)
        self.assertGreater(st.lease["answers"], 20)
        st.predict = lambda body: {"target": [v + 0.05 for v in body["commanded"]["q_l_chunk"][:6]] + [1.0],
                                   "acquired_at": clock()}
        st.advance(0.1)
        self.assertFalse(st.lease["active"])
        self.assertEqual(st.lease["reason"], "prediction_error")
        self.assertIn("excessive", st.lease["stop_reason"])

    def test_stale_answers_end_the_lease(self):
        clock = SimClock(); st = FakeYamStation(clock=clock)
        st.predict = lambda body: {"target": body["commanded"]["q_l_chunk"], "acquired_at": clock() - 0.3}
        leased(st); st.advance(0.1)
        self.assertIn("expired", st.lease["stop_reason"])


class LeaseFreeHold(unittest.TestCase):
    def test_every_drill_state_holds_within_300_ms(self):
        for state in drills.YAM_STATES:
            for seed in range(5):
                clock, st, _ = drills.yam_in_state(state, seed=seed)
                fast = max(abs(v) for v in st.left.dq) > 0.25     # the station's MOVING_RAD_S
                answer = st.hold({})
                row = answer["arms"]["left"]
                if state == "reset_reopen":
                    # the reopen cannot be interrupted: the hold says so at once, and the Home after it is cancelled
                    self.assertFalse(row["holding"])
                    self.assertEqual(row["state"], "resetting")
                    self.assertIn("reset_home_cancelled", answer["actions"])
                    st.advance(2.5)
                    self.assertEqual(st.state, "idle")
                    self.assertTrue(drills.judge_hold(st, [st.left])["left"]["holding"])
                    continue
                self.assertTrue(row["holding"], (state, row))
                self.assertEqual(row["state"], "holding")
                self.assertLessEqual(answer["elapsed_ms"], 300, state)
                if fast:                            # a fast arm coasts: the hold waits for it (the budget is used)
                    self.assertGreaterEqual(answer["elapsed_ms"], 33, state)
                # the answer is the newest SAMPLE, not the simulator's ground truth
                self.assertEqual(row["sample_id"], st.left.last_row["sample_id"])
                self.assertEqual(row["positions_rad"], st.left.last_row["positions_rad"])
                self.assertLessEqual(row["sample_age_ms"], 50)
                verdict = drills.judge_hold(st, [st.left])["left"]
                self.assertTrue(verdict["holding"], (state, verdict))
                self.assertEqual(st.state, "idle", state)
                self.assertFalse(st.lease and st.lease["active"])
                if state in ("motion_streaming", "jog"):
                    self.assertEqual(st.lease["reason"], "supervisor_stop")   # what every client reads as a Stop

    def test_a_dead_or_unconfirmed_arm_is_never_reported_held(self):
        """Under usb_drop / freeze / stale the fake never says holding:true with ground-truth q."""
        cases = {
            "usb_drop": ("fault", lambda f, now: f.add("usb_drop", now, None, arm="left")),
            "dfu": ("fault", lambda f, now: f.add("dfu", now, None, arm="left")),
            "freeze": ("stale", lambda f, now: f.add("freeze", now, None, arm="left")),
            "stale": ("stale", lambda f, now: f.add("stale", now, None, arm="left", extra_s=0.3)),
        }
        for name, (want, inject) in cases.items():
            for state in ("idle", "motion_streaming"):
                clock, st, _ = drills.yam_in_state(state)
                inject(st.faults, clock())
                st.advance(0.3)                     # the fault has lasted 300 ms
                answer = st.hold({})
                row = answer["arms"]["left"]
                self.assertIs(row["holding"], False, (name, state, row))
                self.assertIs(answer["holding"], False)
                self.assertEqual(row["state"], want, (name, state, row))
                self.assertTrue(row["cause"], (name, state))
                if want == "stale":                 # positions are the newest sample's, never arm.q
                    self.assertEqual(row["positions_rad"], st.left.last_row["positions_rad"])
                    self.assertGreater(row["sample_age_ms"], 250)
                else:
                    self.assertNotIn("positions_rad", row)

    def test_a_fast_arm_coasts_and_the_hold_waits_for_it(self):
        clock, st, _ = drills.yam_in_state("home")               # homing at the full 0.6 rad/s
        self.assertGreater(max(abs(v) for v in st.left.dq), 0.5)
        q_at_stop = list(st.left.q)
        answer = st.hold({})
        row = answer["arms"]["left"]
        self.assertTrue(row["holding"], row)
        self.assertGreaterEqual(answer["elapsed_ms"], 66)        # at least two driver cycles of coasting
        self.assertLessEqual(answer["elapsed_ms"], 300)
        coast = max(abs(a - b) for a, b in zip(st.left.q, q_at_stop))
        self.assertGreater(coast, 0.02)                           # ~30 mrad: v^2/2a from 0.6 rad/s at 6 rad/s^2
        self.assertLess(coast, 0.04)
        self.assertLessEqual(max(abs(v) for v in row["velocities_rad_s"]), 0.25)

    def test_a_freeze_shorter_than_the_stale_bound_still_confirms_from_its_last_sample(self):
        """The station confirms on a sample up to 0.25 s old; the answer says how old, and its positions are that
        sample's (the hold never invents a pose)."""
        clock, st, _ = drills.yam_in_state("idle")
        st.faults.add("freeze", clock(), None, arm="left")
        st.advance(0.1)
        frozen = st.left.last_row
        answer = st.hold({})
        row = answer["arms"]["left"]
        self.assertTrue(row["holding"], row)
        self.assertEqual(row["positions_rad"], frozen["positions_rad"])
        self.assertGreater(row["sample_age_ms"], 90)
        self.assertLessEqual(row["sample_age_ms"], 250)

    def test_the_latch_refuses_new_leases_for_0_75_s(self):
        clock, st, _ = drills.yam_in_state("idle")
        st.hold({})
        session = st.register_session(seconds=6)
        oid = st.observe()["observation_id"]
        body = {"observation_id": oid, "seconds": 6, "instruction": "x",
                "reference": {"kind": "video_motion", "session_id": session}}
        with self.assertRaisesRegex(StationError, "nothing new starts within 0.75 s"):
            st.action(body)
        st.advance(0.8)
        self.assertTrue(st.action(body)["lease"]["active"])

    def test_hold_is_harmless_to_repeat(self):
        clock, st, _ = drills.yam_in_state("motion_streaming")
        first = st.hold({})
        for _ in range(5):
            again = st.hold({})
        self.assertEqual(first["arms"]["left"]["holding"], again["arms"]["left"]["holding"])
        self.assertEqual(again["actions"], ["nothing_moving"])
        self.assertEqual(again["hold"]["generation"], first["hold"]["generation"] + 5)

    def test_not_available_unless_enabled(self):
        clock, st, _ = drills.yam_in_state("idle", lease_free_hold=False)
        with self.assertRaises(LookupError):
            st.hold({})


class MetalOwner(unittest.TestCase):
    def test_stop_preempts_home_and_motion_in_any_state(self):
        for state in drills.METAL_STATES:
            clock, owner, lease = drills.metal_in_state(state)
            if state != "idle":
                self.assertEqual(owner.status()["state"], "moving")
            epoch = owner.epoch
            st = owner.stop()
            self.assertEqual(owner.epoch, epoch + 1)
            self.assertEqual(st["state"], "holding")
            self.assertTrue(drills.judge_hold(owner, [owner.arm])["right"]["holding"], state)
            if lease:
                self.assertEqual(owner.motion_lease["reason"], "supervisor_stop")

    def test_motion_heartbeat_and_bounds(self):
        clock, owner, lease = drills.metal_in_state("motion_streaming")
        owner.advance(1.0)                                     # no heartbeat for > 0.8 s
        self.assertEqual(owner.motion_lease["reason"], "client_disconnected")
        clock, owner, lease = drills.metal_in_state("motion_streaming")
        owner.predict = lambda body: {"target": [v + 0.2 for v in body["commanded"]["q_r_chunk"][:6]] + [1.0],
                                      "acquired_at": clock()}
        owner.advance(0.1)
        self.assertEqual(owner.motion_lease["reason"], "prediction_error")

    def test_usb_drop_dfu_and_the_0207_crash(self):
        clock = SimClock(); faults = Faults()
        owner = FakeMetalOwner(clock=clock, faults=faults); owner.advance(0.2)
        faults.add("dfu", clock(), None, arm="right")
        owner.advance(0.1)
        self.assertIn("not the confirmed right Metal candleLight adapter", owner.status()["error"])
        with self.assertRaises(OwnerError):
            owner.home()
        clock = SimClock(); faults = Faults()
        owner = FakeMetalOwner(clock=clock, faults=faults, crash_on_double_fault=True); owner.advance(0.2)
        faults.add("usb_drop", clock(), None, arm="right")
        owner.advance(0.2)
        self.assertTrue(owner.dead)
        with self.assertRaises(OwnerDead):
            owner.stop()
        clock = SimClock(); faults = Faults()
        fixed = FakeMetalOwner(clock=clock, faults=faults); fixed.advance(0.2)
        faults.add("usb_drop", clock(), None, arm="right")
        fixed.advance(0.5)
        self.assertFalse(fixed.dead)
        self.assertEqual(fixed.status()["state"], "fault_hold")
        self.assertEqual(fixed.status()["measured"], {})


class Cameras(unittest.TestCase):
    def test_identity_frames_loss_and_restart(self):
        clock = SimClock(); faults = Faults()
        cam = FakeCamera(clock=clock, faults=faults)
        ident = cam.identity()
        self.assertEqual(ident["serial"], "000000000000")
        self.assertRegex(ident["intrinsics_sha256"], r"^[0-9a-f]{64}$")
        clock.advance(0.1)
        f1 = cam.frame(); clock.advance(0.05); f2 = cam.frame()
        self.assertGreater(f2["frame_id"], f1["frame_id"])
        self.assertLessEqual(f1["acquired_at_s"], clock())
        cam.restart()
        self.assertNotEqual(cam.identity()["session"], ident["session"])
        faults.add("camera_lost", clock(), 1.0, camera="top")
        with self.assertRaises(CameraLost):
            cam.frame()

    def test_bracketed_capture_rejects_motion(self):
        clock = SimClock(); st = FakeYamStation(clock=clock); st.advance(0.5)
        cam = FakeCamera(clock=clock)

        def arms():
            st.advance(1/30)
            return {"left": st.left.row(clock())}
        ok, bracket = cam.capture_bracketed(arms)
        self.assertTrue(ok, bracket)
        st.left.command(q=[st.left.q[0] + 0.3] + st.left.q[1:])
        st.advance(0.1)
        ok, bracket = cam.capture_bracketed(arms)
        self.assertFalse(ok)
        self.assertEqual(bracket["reason"], "moved")

    def test_bracket_needs_0_3_s_of_quiet_before_the_exposure_and_a_sample_after_it(self):
        """K7: still over >= 0.3 s up to the exposure (not two samples ~66 ms apart), and before <= exposure <= after."""
        clock = SimClock(); st = FakeYamStation(clock=clock); st.advance(0.5)
        cam = FakeCamera(clock=clock)

        def arms():
            st.advance(1/30)
            return {"left": st.left.row(clock())}
        ok, b = cam.capture_bracketed(arms)
        self.assertTrue(ok, b)
        self.assertGreaterEqual(b["quiet_s"], 0.3 - 1e-9)
        self.assertLessEqual(b["before"], b["exposure"])
        self.assertLessEqual(b["exposure"], b["after"])
        self.assertGreaterEqual(b["samples"], 10)
        ok, b = cam.capture_bracketed(arms, max_reads=4)          # ~0.13 s of samples: never quiet long enough
        self.assertFalse(ok)
        self.assertIn("not quiet for 0.3 s", b["reason"])
        frozen = {"left": st.left.row(clock())}
        ok, b = cam.capture_bracketed(lambda: frozen)              # samples that never advance cover no span
        self.assertFalse(ok)
        creep = {"q": list(st.left.q)}

        def creeping():                                            # 6 mrad/s: 1.8 mrad over 0.3 s
            st.advance(1/30)
            creep["q"][0] += 0.006/30
            row = dict(st.left.row(clock())); row["positions_rad"] = list(creep["q"])
            return {"left": row}
        ok, b = cam.capture_bracketed(creeping)
        self.assertFalse(ok)
        self.assertEqual(b["reason"], "moved")


class Http(unittest.TestCase):
    def post(self, url, body, headers=None, timeout=2.0):
        h = {"Content-Type": "application/json"}; h.update(headers or {})
        req = urllib.request.Request(url, data=json.dumps(body).encode(), headers=h)
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.load(resp)

    def test_station_over_http(self):
        faults = Faults()
        st = FakeYamStation(faults=faults, lease_free_hold=True)
        st.predict = lambda body: {"target": body["commanded"]["q_l_chunk"], "acquired_at": time.monotonic()}
        sup = {"X-Manifest-Supervisor": "1"}
        with SimServer(st) as srv:
            time.sleep(0.15)
            pose = json.load(urllib.request.urlopen(srv.url + "/supervisor/pose", timeout=2))
            self.assertEqual(pose["schema"], "manifest-native-pose/1")
            with self.assertRaises(urllib.error.HTTPError) as err:
                self.post(srv.url + "/supervisor/observe", {})            # no supervisor header
            self.assertEqual(err.exception.code, 403)
            oid = self.post(srv.url + "/supervisor/observe", {}, sup)["observation_id"]
            session = st.register_session()
            lease = self.post(srv.url + "/supervisor/action", {"observation_id": oid, "seconds": 6, "instruction": "x",
                              "reference": {"kind": "video_motion", "session_id": session}}, sup)["lease"]["id"]
            with self.assertRaises(urllib.error.HTTPError) as err:
                self.post(srv.url + "/supervisor/stop", {}, sup)
            self.assertEqual(err.exception.code, 409)
            started = time.monotonic()
            faults.add("gc_pause", time.monotonic(), 0.3)
            self.post(srv.url + "/supervisor/heartbeat", {"lease_id": lease}, sup)
            self.assertGreaterEqual(time.monotonic() - started, 0.25)
            held = self.post(srv.url + "/supervisor/hold", {}, sup)
            self.assertTrue(held["arms"]["left"]["holding"], held["arms"]["left"])
            self.assertEqual(held["schema"], "manifest-station-hold/1")
            self.assertTrue(held["hold"]["verified"])
            # any local program may hold: without the supervisor header it is obeyed and recorded as unverified
            unverified = self.post(srv.url + "/supervisor/hold", {"source": "local_script"})
            self.assertFalse(unverified["hold"]["verified"])
            self.assertEqual(unverified["hold"]["source"], "local_script")
            self.assertTrue(unverified["arms"]["left"]["holding"])
            # a web page may not, whatever it sends
            for headers in ({"Origin": "https://x"}, {"Origin": "http://127.0.0.1:9011"}, {"Origin": "null"}, {"Sec-Fetch-Site": "cross-site"},
                            {"Sec-Fetch-Site": "same-site"}, {"User-Agent": "Mozilla/5.0", "Content-Type": "text/plain"}):
                with self.assertRaises(urllib.error.HTTPError) as err:
                    self.post(srv.url + "/supervisor/hold", {}, headers)
                self.assertEqual(err.exception.code, 403, headers)
            with self.assertRaises(urllib.error.HTTPError) as err:
                self.post(srv.url + "/supervisor/hold", {}, {"Host": "rebind.test:%d" % srv.httpd.server_address[1]})
            self.assertEqual(err.exception.code, 421)
            with self.assertRaises(urllib.error.HTTPError) as err:
                urllib.request.urlopen(urllib.request.Request(srv.url + "/status", headers={"Sec-Fetch-Site": "cross-site"}), timeout=2)
            self.assertEqual(err.exception.code, 403)
            faults.add("usb_drop", time.monotonic(), 0.3, arm="left")
            time.sleep(0.05)
            dropped = self.post(srv.url + "/supervisor/hold", {}, sup)
            self.assertEqual((dropped["arms"]["left"]["state"], dropped["arms"]["left"]["holding"]), ("fault", False))
            time.sleep(0.3)
            faults.add("link_loss", time.monotonic(), 0.5)
            with self.assertRaises((urllib.error.URLError, ConnectionError, OSError)):
                urllib.request.urlopen(srv.url + "/status", timeout=1)
            time.sleep(0.55)
            self.assertEqual(json.load(urllib.request.urlopen(srv.url + "/status", timeout=2))["state"], "idle")

    def test_metal_over_http(self):
        owner = FakeMetalOwner()
        with SimServer(owner) as srv:
            time.sleep(0.1)
            with self.assertRaises(urllib.error.HTTPError) as err:
                self.post(srv.url + "/stop", {})
            self.assertEqual(err.exception.code, 403)
            st = self.post(srv.url + "/stop", {}, {"X-Manifest-Metal-Control": "1"})
            self.assertEqual(st["state"], "holding")
            self.assertEqual(json.load(urllib.request.urlopen(srv.url + "/status", timeout=2))["schema"],
                             "manifest-metal-owner/1")


class Imports(unittest.TestCase):
    def test_standard_library_only(self):
        code = ("import sys; import roborun_station, roborun_station.drills;"
                "print(sorted(m for m in ('torch','transformers','cv2','numpy','mujoco','pydantic') if m in sys.modules))")
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60,
                             env={"PYTHONPATH": str(SRC), "PATH": "/usr/bin:/bin"})
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(out.stdout.strip(), "[]")
        pattern = re.compile(r"^\s*(import|from)\s+(torch|transformers|cv2|numpy|mujoco)\b", re.M)
        for path in (SRC / "roborun_station").glob("*.py"):
            self.assertIsNone(pattern.search(path.read_text()), path.name)


if __name__ == "__main__":
    unittest.main()
