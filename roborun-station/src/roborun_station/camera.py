"""Fake cameras: identity (serial, session, intrinsics hash), timed exposures, a bracketed capture, faults.

Frames are deterministic bytes (a JPEG-shaped RGB payload and little-endian z16 depth), stamped on the owner clock
at the exposure. A restart changes the session id: the identity change the SDK must report (calibration suspect).
"""
from __future__ import annotations

import hashlib
import json
import struct
import uuid

from .core import Faults, RealClock


class CameraLost(RuntimeError):
    pass


class FakeCamera:
    def __init__(self, name="top", model="D435i", serial="000000000000", *, depth=True, width=640, height=480,
                 fx=605.0, fy=605.0, cx=320.0, cy=240.0, fps=30.0,
                 depth_scale_m=0.001, clock=None, faults: Faults | None = None):
        self.name, self.model, self.serial, self.depth = name, model, serial, depth
        self.intrinsics = {"w": width, "h": height, "fx": fx, "fy": fy, "cx": cx, "cy": cy}
        self.fps, self.depth_scale_m = fps, depth_scale_m
        self.clock = clock or RealClock()
        self.faults = faults or Faults()
        self.session = "cs_" + uuid.uuid4().hex[:8]
        self.started_at = self.clock()

    def identity(self) -> dict:
        digest = hashlib.sha256(json.dumps(self.intrinsics, sort_keys=True).encode()).hexdigest()
        return {"id": self.name, "model": self.model, "serial": self.serial, "session": self.session,
                "intrinsics": dict(self.intrinsics), "intrinsics_sha256": digest,
                "depth_scale_m": self.depth_scale_m if self.depth else None, "depth": self.depth}

    def restart(self):
        """A USB re-enumeration or service restart: same serial, new session (calibration suspect)."""
        self.session = "cs_" + uuid.uuid4().hex[:8]
        self.started_at = self.clock()

    def frame(self) -> dict:
        now = self.clock()
        if self.faults.active("camera_lost", now, camera=self.name) is not None:
            raise CameraLost(f"camera {self.name} delivers no frames")
        index = int((now - self.started_at) * self.fps + 1e-6)
        exposure = self.started_at + index / self.fps
        seed = hashlib.sha256(f"{self.session}:{index}".encode()).digest()
        rgb = b"\xff\xd8\xff\xe0" + seed * 64 + b"\xff\xd9"
        depth = None
        if self.depth:
            depth = struct.pack(f"<{64}H", *[(b*7 + 300) for b in seed*2])
        return {"camera": self.name, "frame_id": index, "acquired_at_s": exposure, "camera_session_id": self.session,
                "acquisition_timestamp_verified": True, "rgb": rgb, "depth_z16": depth,
                "sha256": hashlib.sha256(rgb).hexdigest()}

    def capture_bracketed(self, read_arms, *, quiet_s=0.3, max_joint_range_rad=0.0015, max_jaw_range=0.003,
                          max_reads=64):
        """K7's bracket: every arm still -- each joint within `max_joint_range_rad` and the jaw within
        `max_jaw_range` over all samples -- for at least `quiet_s` up to the exposure, and still in a sample taken
        after it (video_capture's chain of still observes spanning QUIET_S before the exposure).

        read_arms() -> {arm: row} returns each arm's newest driver sample (rows carry source_monotonic_s,
        positions_rad and gripper; under a SimClock the caller's read_arms advances time). Samples are read until they
        span `quiet_s`, then one exposure is taken, then samples until one is stamped after the exposure. Any motion
        refuses at once (the caller tries again); running out of `max_reads` refuses as not quiet.
        Returns (accepted, bracket)."""
        chain: list[dict] = []
        frame = None

        def ranges():
            joint = jaw = 0.0
            for arm in chain[0]:
                rows = [c[arm] for c in chain]
                joint = max(joint, max(max(col) - min(col) for col in zip(*(r["positions_rad"] for r in rows))))
                jaws = [r["gripper"] for r in rows]
                jaw = max(jaw, max(jaws) - min(jaws))
            return joint, jaw

        def verdict(ok, reason, after=None):
            joint, jaw = ranges() if chain else (0.0, 0.0)
            start = max(r["source_monotonic_s"] for r in chain[0].values()) if chain else None
            exposure = None if frame is None else frame["acquired_at_s"]
            return ok, {"accepted": ok, "reason": reason, "before": start, "after": after, "exposure": exposure,
                        "quiet_s": None if exposure is None or start is None else exposure - start,
                        "quiet_limit_s": quiet_s, "joint_range_rad": joint, "jaw_range": jaw, "samples": len(chain),
                        "frame": frame}

        for _ in range(max_reads):
            rows = read_arms()
            if not rows or (chain and set(rows) != set(chain[0])):
                return verdict(False, "an arm sample is missing")
            chain.append(rows)
            joint, jaw = ranges()
            if joint > max_joint_range_rad or jaw > max_jaw_range:
                return verdict(False, "moved")
            start = max(r["source_monotonic_s"] for r in chain[0].values())
            newest = min(r["source_monotonic_s"] for r in rows.values())
            if frame is None:
                if newest - start < quiet_s - 1e-9:
                    continue
                frame = self.frame()
                if frame["acquired_at_s"] - start < quiet_s - 1e-9:
                    frame = None                   # exposed before the quiet span was covered: take a later one
                continue
            if newest >= frame["acquired_at_s"] - 1e-9:   # the after-sample is stamped after the exposure
                return verdict(True, None, after=newest)
        return verdict(False, f"not quiet for {quiet_s:g} s within {max_reads} samples")
