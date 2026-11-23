"""The Stop drill matrix in sim, through the real connectors and the shared fakes of
roborun-station: YAM station states x {the lease-free hold, a
station without it}, and the Metal owner's states. Pass: the connector's per-arm ack never says "holding" unless the
fake's physics shows the arm held (judge_hold), and with the lease-free hold every powered arm is held."""
import sys
from pathlib import Path

import pytest

SIM_SRC = Path(__file__).resolve().parents[2] / "roborun-station/src"
pytestmark = pytest.mark.skipif(not SIM_SRC.is_dir(), reason="roborun-station not present")
if SIM_SRC.is_dir() and str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

YAM_STATES = ("idle", "motion_streaming", "jog", "home", "thermal_blend", "reset_home", "reset_reopen")


def _drill_yam(state, lease_free_hold):
    from roborun_station import SimServer, drills

    from roborun_sdk.connectors.yam import YamStationConnector
    clock, station, lease = drills.yam_in_state(state, lease_free_hold=lease_free_hold)
    with SimServer(station) as srv:
        conn = YamStationConnector(srv.url, allow_post=True, sysroot=Path("/nonexistent"))
        ack = conn.hold("stop:drill")
    judged = drills.judge_hold(station, [station.left])["left"]
    return ack, judged


@pytest.mark.parametrize("state", YAM_STATES)
def test_yam_drill_with_lease_free_hold(state):
    ack, judged = _drill_yam(state, True)
    if ack.holding:
        assert judged["holding"], (state, ack.detail, judged)
    if state == "reset_reopen":
        assert ack.holding is not True                     # arms let go while the drivers reopen: support them
    else:
        assert ack.holding is True and judged["holding"], (state, ack.detail, judged)


@pytest.mark.parametrize("state", YAM_STATES)
def test_yam_drill_on_todays_station_never_claims_a_false_hold(state):
    ack, judged = _drill_yam(state, False)
    if ack.holding:
        assert judged["holding"], (state, ack.detail, judged)
    if state in ("home", "thermal_blend", "reset_home", "reset_reopen"):
        assert ack.holding is not True                     # the vendor refuses Stop there: reported, never hidden


@pytest.mark.parametrize("state", ("idle", "motion_streaming", "home"))
def test_metal_drill(state):
    from roborun_station import SimServer, drills

    from roborun_sdk.connectors.metal import MetalOwnerConnector
    clock, owner, lease = drills.metal_in_state(state)
    with SimServer(owner) as srv:
        ack = MetalOwnerConnector(srv.url, allow_post=True).hold("stop:drill")
    judged = drills.judge_hold(owner, [owner.arm])
    assert ack.holding is True and all(v["holding"] for v in judged.values()), (state, ack.detail, judged)
