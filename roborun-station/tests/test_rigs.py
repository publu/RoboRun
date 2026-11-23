"""The dual-YAM station (a REAL second YAM, never a parked stand-in) and the parameterised, live-like Metal
owner (arm / port / channel / serial; 5 deg/s, sag stalls 0.29-2.1 deg short, the joint-5 arrival bias, a jaw that stalls on
the cup). The defaults stay as they are (left-only station, ideal Metal follower): the SDK drills must not change under them."""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from roborun_station import (METAL_J5_ARRIVAL_DEG, FakeMetalOwner, FakeYamStation, Faults, SimClock,  # noqa: E402
                                    StationError, metal_shortfall_rad, yam_right_spec)
from roborun_station import drills  # noqa: E402


def lease(station, arm, seconds=8):
    session = station.register_session(seconds=seconds, arm=arm)
    oid = station.observe()["observation_id"]
    return station.action({"observation_id": oid, "seconds": seconds, "instruction": "x",
                           "reference": {"kind": "video_motion", "session_id": session}})["lease"]


class DualYam(unittest.TestCase):
    def setUp(self):
        self.clock = SimClock()
        self.bodies = []

        def predict(body):
            self.bodies.append(body)
            arm = body["enabled_arms"][0]
            chunk = list(body["commanded"]["q_l_chunk" if arm == "left" else "q_r_chunk"])
            chunk[0] += 0.03
            return {"target": chunk, "acquired_at": self.clock(), "steps": 1}
        self.st = FakeYamStation(clock=self.clock, right="yam", lease_free_hold=True, predict=predict)
        self.st.advance(0.3)

    def test_the_right_arm_is_a_second_yam_not_a_parked_stand_in(self):
        pose = self.st.pose()
        self.assertEqual(pose["arms"]["right"]["body"], "yam")
        self.assertEqual(pose["arms"]["right"]["state"], "fresh")
        self.assertEqual(pose["measured"]["right"]["body"], "yam")
        self.assertEqual(self.st.status()["parked_arms"], [])
        self.assertEqual(tuple(self.st.right.spec.home), yam_right_spec().home)

    def test_a_right_lease_streams_only_the_right_yam(self):
        left0, right0 = list(self.st.left.q), list(self.st.right.q)
        out = lease(self.st, "right")
        self.assertEqual(out["arm"], "right")
        self.st.advance(1.0)
        self.assertTrue(self.bodies and all(b["enabled_arms"] == ["right"] and "q_r_chunk" in b["commanded"]
                                            and b["robot_body"] == "yam" for b in self.bodies))
        self.assertGreater(self.st.right.q[0] - right0[0], 0.2)                 # the right YAM moved
        self.assertLess(max(abs(a - b) for a, b in zip(self.st.left.q, left0)), 1e-3)   # the left one did not

    def test_a_left_lease_still_streams_the_left_yam(self):
        right0 = list(self.st.right.q)
        lease(self.st, "left")
        self.st.advance(1.0)
        self.assertTrue(all(b["enabled_arms"] == ["left"] for b in self.bodies))
        self.assertLess(max(abs(a - b) for a, b in zip(self.st.right.q, right0)), 1e-3)

    def test_a_leased_move_home_takes_one_arm_home_with_its_grip_kept(self):
        arm = self.st.right
        arm.command(q=[v + 0.3 for v in arm.spec.home[:1]] + list(arm.spec.home[1:]), jaw=0.4)
        self.st.advance(2.0)
        oid = self.st.observe()["observation_id"]
        out = self.st.action({"observation_id": oid, "seconds": 8, "reference": {"kind": "move_home", "arm": "right"}})
        self.assertEqual(out["lease"]["arm"], "right")
        self.st.advance(2.0)
        self.assertEqual(self.st.lease["reason"], "action_goal_reached")
        self.assertTrue(arm.at(arm.spec.home, tol=0.01))
        self.assertAlmostEqual(arm.jaw_cmd, 0.4, places=6)                     # the grip is kept

    def test_home_and_the_hold_cover_both_arms(self):
        for arm in (self.st.left, self.st.right):
            arm.command(q=[arm.spec.home[0] + 0.3] + list(arm.spec.home[1:]))
        self.st.advance(1.0)
        self.st.home()
        self.st.advance(3.5)
        for arm in (self.st.left, self.st.right):
            self.assertTrue(arm.at(arm.spec.home, tol=0.01))
        reply = self.st.hold({"source": "test"})
        self.assertEqual(set(reply["arms"]), {"left", "right"})
        self.assertTrue(reply["holding"])

    def test_a_dead_right_driver_refuses_its_lease(self):
        faults = self.st.faults
        faults.add("usb_drop", self.clock(), 5.0, arm="right")
        self.st.advance(0.1)
        with self.assertRaises(StationError):
            lease(self.st, "right")

    def test_the_default_station_is_unchanged(self):
        st = FakeYamStation(clock=SimClock())
        self.assertTrue(st.parked_right)
        self.assertEqual(st.pose()["arms"]["right"], {"body": "yam", "state": "parked"})
        with self.assertRaises(StationError):
            st.arm("right")


class MetalOwnerIdentity(unittest.TestCase):
    def test_arm_port_channel_and_serial_are_parameters(self):
        clock = SimClock()
        m = FakeMetalOwner(clock=clock, arm="left", port=8094, channel="can_metal_left", serial="LEFT0001")
        m.advance(0.1)
        st = m.status()
        self.assertEqual((st["arm"], st["port"], st["adapter"]), ("left", 8094, {"channel": "can_metal_left",
                                                                               "usb_serial": "LEFT0001"}))
        self.assertEqual(list(st["measured"]), ["left"])
        self.assertTrue(st["measured"]["left"]["sample_id"].startswith("metal:LEFT0001:"))
        self.assertIn("mixed_motion_left_version", m.motion_observe()["capabilities"])

    def test_faults_are_keyed_by_the_owners_own_arm(self):
        clock = SimClock(); faults = Faults()
        m = FakeMetalOwner(clock=clock, faults=faults, arm="left", channel="can7")
        m.advance(0.1)
        faults.add("dfu", clock(), 1.0, arm="right")                         # another arm's fault: not this owner
        m.advance(0.1)
        self.assertIsNone(m.status()["error"])
        faults.add("dfu", clock(), 1.0, arm="left")
        m.advance(0.1)
        self.assertEqual(m.status()["error"], "can7 is not the confirmed left Metal candleLight adapter")

    def test_the_default_owner_is_the_right_metal(self):
        m = FakeMetalOwner(clock=SimClock())
        st = m.status()
        self.assertEqual((st["arm"], st["port"], st["adapter"]["channel"], st["physics"]), ("right", 8093, "can0", "ideal"))


class LiveMetal(unittest.TestCase):
    def setUp(self):
        self.clock = SimClock()
        self.cup = False
        self.m = FakeMetalOwner(clock=self.clock, physics="live", cup_between=lambda: self.cup)
        self.m.advance(0.2)

    def test_it_moves_at_five_degrees_per_second(self):
        arm = self.m.arm
        q0 = arm.q[0]
        arm.command(q=[q0 + math.radians(20)] + list(arm.q[1:]))
        self.m.advance(1.0)
        self.assertAlmostEqual(math.degrees(arm.q[0] - q0), 5.0, delta=0.2)

    def test_joints_two_and_three_stall_short_from_sag_more_when_extended(self):
        arm = self.m.arm
        folded = list(arm.q)
        target = list(folded); target[1] -= math.radians(5)
        arm.command(q=target)
        self.m.advance(3.0)
        short_folded = math.degrees(abs(arm.q[1] - target[1]))
        self.assertGreaterEqual(short_folded, 0.28)
        self.assertLess(short_folded, 0.6)
        extended = list(folded); extended[1] = math.radians(-60); extended[2] = math.radians(60)
        arm.q = list(extended); arm.q_cmd = list(extended)
        target = list(extended); target[2] += math.radians(5)
        arm.command(q=target)
        self.m.advance(3.0)
        short_ext = math.degrees(abs(arm.q[2] - target[2]))
        self.assertGreater(short_ext, 1.8)
        self.assertLessEqual(short_ext, 2.11)
        self.assertIn(2, arm.stalled)

    def test_joint_five_arrives_short_by_its_bias(self):
        arm = self.m.arm
        target = list(arm.q); target[4] += math.radians(4)
        arm.command(q=target)
        self.m.advance(2.0)
        self.assertAlmostEqual(math.degrees(target[4] - arm.q[4]), METAL_J5_ARRIVAL_DEG, places=3)

    def test_the_jaw_stalls_on_the_cup_and_reports_its_effort(self):
        self.cup = True
        self.m.arm.command(jaw=self.m.arm.spec.jaw_closed)
        self.m.advance(12.0)
        st = self.m.status()
        row = st["measured"]["right"]
        self.assertAlmostEqual(row["gripper"], 0.045, places=3)            # stopped on the wall, not closed
        self.assertTrue(row["jaw_blocked"])
        self.assertGreater(row["jaw_effort_nm"], 0.25)
        self.cup = False
        self.m.arm.command(jaw=self.m.arm.spec.jaw_closed)
        self.m.advance(2.0)
        self.assertLess(self.m.status()["measured"]["right"]["gripper"], 0.01)

    def test_home_arrives_at_the_paired_measured_pose_despite_the_stalls(self):
        arm = self.m.arm
        away = list(arm.spec.home); away[0] += math.radians(8); away[2] += math.radians(6)
        arm.q = list(away); arm.q_cmd = list(away)
        self.m.home()
        self.m.advance(4.0)
        self.assertIsNone(self.m.motion)                                     # Home completed (0.5 deg arrival)
        self.assertTrue(arm.at(arm.spec.home, tol=math.radians(0.5)))

    def test_shortfall_model_bounds(self):
        home = self.m.arm.spec.home
        self.assertAlmostEqual(math.degrees(metal_shortfall_rad(home, 1)), 0.29, delta=0.03)   # folded: 0.31
        far = list(home); far[1] = math.radians(-60); far[2] = math.radians(60)
        self.assertAlmostEqual(math.degrees(metal_shortfall_rad(far, 2)), 2.1, places=6)
        self.assertEqual(metal_shortfall_rad(home, 0), 0.0)


class DrillsUnchanged(unittest.TestCase):
    def test_metal_drills_still_hold_at_one_milliradian(self):
        for state in drills.METAL_STATES:
            clock, owner, lease_id = drills.metal_in_state(state)
            owner.stop()
            result = drills.judge_hold(owner, [owner.arm])
            self.assertTrue(result["right"]["holding"], (state, result))


if __name__ == "__main__":
    unittest.main()
