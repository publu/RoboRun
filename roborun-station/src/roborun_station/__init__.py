"""roborun-station: the station logic around your arm adapters (leases, Stop and hold, Home), plus simulated arms, owners and
cameras with fault injection for tests.

    from roborun_station import SimClock, Faults, FakeYamStation, FakeMetalOwner, FakeCamera, SimServer

In-process (deterministic): build a simulated station with a SimClock and call `station.advance(seconds)`.
Over HTTP (real time, 127.0.0.1 only; run under `unshare -rn`): `with SimServer(fake) as srv: srv.url`.
Faults: `faults.add(kind, start, duration, **params)`, kinds stale / freeze / gc_pause / link_loss / stall /
usb_drop / dfu / slow_reply / camera_lost (see core.Faults). Stop drill states: roborun_station.drills.
Standard library only.
"""
from .camera import CameraLost, FakeCamera
from .core import (METAL_J5_ARRIVAL_DEG, METAL_JAW_ON_CUP, METAL_SAG_SHORT_DEG, VELOCITY_QUANTUM, ArmSpec, Faults, RealClock,
                   SimArm, SimClock, metal_extension, metal_right_spec, metal_shortfall_rad, metal_spec, yam_left_spec,
                   yam_right_spec)
from .http import SimServer
from .metal_owner import FakeMetalOwner, OwnerDead, OwnerError
from .yam_station import FakeYamStation, StationError

__version__ = "0.1.0"
__all__ = ["SimClock", "RealClock", "Faults", "SimArm", "ArmSpec", "yam_left_spec", "yam_right_spec", "metal_right_spec",
           "metal_spec", "metal_shortfall_rad", "metal_extension", "METAL_SAG_SHORT_DEG", "METAL_J5_ARRIVAL_DEG",
           "METAL_JAW_ON_CUP",
           "FakeYamStation", "StationError", "FakeMetalOwner", "OwnerError", "OwnerDead", "FakeCamera", "CameraLost",
           "SimServer", "VELOCITY_QUANTUM"]
