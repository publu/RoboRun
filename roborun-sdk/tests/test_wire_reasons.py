"""The uplink names its refusals with the wire's own reasons: a passed acceptance on an owner the SDK cannot stream yet
is `connector_not_wired`, a refusal while the link is late is `link_late` (never `busy`)."""
import pytest

from roborun_sdk.link import wire_compat
from roborun_sdk.rt import wire


@pytest.mark.parametrize("local", ["connector_not_wired", "link_late"])
def test_reason_travels_as_itself(local):
    msg = wire_compat.ack({"ref": "seg_1", "result": "refused", "reason": local, "sdk_t": 2.0,
                           "message": "x", "acceptance": "passed"}, seq=4)
    assert msg["type"] == "ack" and msg["reason"] == local
    if wire.available():
        assert wire.module().parse_message(dict(msg, wire=1)).reason == local


def test_the_others_still_map_as_before():
    assert wire_compat.REASONS["another_arm_moving"] == "busy"
    assert wire_compat.REASONS["sample_stale"] == "stale_sample"
