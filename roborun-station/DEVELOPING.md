# Developing roborun-station

`roborun-station` is the station logic the Roborun SDK runs around your arm drivers (motion leases, Stop and hold, Home),
plus simulated YAM / Metal owners and cameras for tests. It is pure Python (standard library only), needs no hardware and
opens no network port beyond 127.0.0.1. You normally use it through `roborun-sdk`; this guide is for changing it.

## Set up and test

```sh
python -m venv .venv && . .venv/bin/activate
pip install -e .
python -m pytest -q tests/
```

The tests run in simulated time (`SimClock`) and over HTTP on 127.0.0.1. To keep their ports private, run them in their own
network namespace on Linux: `unshare -rn sh -c 'ip link set lo up; python -m pytest -q tests/'`.

## What is in the package

- `FakeYamStation` (:8091 semantics): supervisor leases with a 2.5 s heartbeat, `POST /supervisor/stop` (needs the lease
  id), the vendor `POST /stop` (refused while HOMING / RESETTING / STOPPING), `POST /home`, `POST /reset`, predict pulls with
  per-answer bounds, and `GET /supervisor/pose`. `lease_free_hold=True` adds `POST /supervisor/hold`: never
  refused, it ends any lease, waits up to 0.25 s for the arms to settle and answers each
  arm's measured state (`holding`, `moving`, `stale`, `fault`, `resetting`). `right="yam"` simulates a second YAM on the right.
- `FakeMetalOwner`: a simulated MakerMods Metal owner with lease-free `/stop`, `/home`, `/recover_hold`, `/lease/observe|action|heartbeat|stop`,
  and adapter identity / DFU refusal. `FakeMetalOwner(arm=, port=, channel=, serial=)` is any Metal of a rig;
  `physics="live"` adds sag, stiction and a jaw that stops on an object, `physics="ideal"` (default) follows exactly.
- `FakeCamera`: serial, session id, intrinsics hash, timed exposures, and captures bracketed by still arm samples.
- Faults (`Faults.add(kind, start, duration, **params)`): `stale`, `freeze`, `gc_pause`, `link_loss`, `stall`, `usb_drop`,
  `dfu`, `slow_reply`, `camera_lost`.
- `drills.yam_in_state(state)` / `metal_in_state` / `judge_hold`: put a fake owner in a state (idle, streaming, jog, Home,
  thermal blend, Reset), issue a hold and judge it from the motion that follows.

```python
from roborun_station import SimClock, Faults, FakeYamStation, SimServer
clock = SimClock(); station = FakeYamStation(clock=clock, lease_free_hold=True); station.advance(0.5)   # in-process
with SimServer(FakeYamStation()) as srv:                      # real time over HTTP on 127.0.0.1
    ...  # srv.url
```

## Versions

The source keeps `version = "0.1.0"`; release builds stamp the real version, and `roborun-sdk` requires the matching
`roborun-station` release. Change the two packages together.
