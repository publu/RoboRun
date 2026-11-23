"""connector-realsense: read-only access to the robot runtime's camera broker. roborun-link only.

The broker owns the D435i top and D405 wrist cameras and serves already-acquired frames over an owner-only Unix
socket (path from ``depth/live-camera-session.json``): 4-byte big-endian length + JSON request; the reply is JSON
(status or error) or an ``np.savez`` packet with ``rgb`` (uint8 HxWx3), ``depth_z16`` (uint16 HxW) and ``metadata``
(serial, intrinsics, depth_scale_m, frame_id, acquired_at_s on CLOCK_MONOTONIC, camera_session_id, ...).

This client only ever sends ``status`` and ``frame``. It never sends ``stop`` or ``restart`` and never opens a
device; the broker keeps ownership. numpy only (no cv2, no pyrealsense2).
"""
from __future__ import annotations

import os
import hashlib
import io
import json
import socket
import struct
from pathlib import Path

import numpy as np

ALLOWED_OPERATIONS = ("status", "frame")
ROLES = ("top", "left", "right")
MAX_PACKET = 8 * 1024 * 1024
# the camera service's session file (which socket the live camera serves on): $MANIFEST_CAMERA_SESSION, else beside the
# Roborun install (<install>/../../depth/live-camera-session.json, its layout today)
DEFAULT_SESSION_FILE = Path(os.environ.get("MANIFEST_CAMERA_SESSION")
                            or Path(os.environ.get("MANIFEST_DREAMAPI", Path.home() / "roborun" / "robot" / "runtime")).parent.parent / "depth" / "live-camera-session.json")


class CameraError(RuntimeError):
    pass


def session_socket(session_file: Path = DEFAULT_SESSION_FILE, tmp: str = "/tmp") -> str:
    data = json.loads(Path(session_file).read_text())
    job = Path(data["job"]).resolve()
    if job.parent != Path(tmp) or not job.name.startswith("fieldwork-rgbd-"):
        raise CameraError("invalid camera session directory")
    return str(job / "camera.sock")


def _recv_exact(conn, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        block = conn.recv(n - len(out))
        if not block:
            raise CameraError("camera connection ended")
        out.extend(block)
    return bytes(out)


def decode_frame(payload: bytes) -> dict:
    with np.load(io.BytesIO(payload), allow_pickle=False) as data:
        if set(data.files) != {"rgb", "depth_z16", "metadata"}:
            raise CameraError("invalid camera frame packet")
        frame = json.loads(data["metadata"].tobytes())
        frame["rgb"] = data["rgb"].copy()
        frame["depth_z16"] = data["depth_z16"].copy()
    intr = frame.get("intrinsics") or {}
    h, w = intr.get("height"), intr.get("width")
    if (not isinstance(h, int) or not isinstance(w, int) or frame["rgb"].shape != (h, w, 3)
            or frame["rgb"].dtype != np.uint8 or frame["depth_z16"].shape != (h, w) or frame["depth_z16"].dtype != np.uint16):
        raise CameraError("camera frame encoding/profile mismatch")
    return frame


class CameraServiceClient:
    def __init__(self, socket_path: str | None = None, *, session_file: Path = DEFAULT_SESSION_FILE, timeout_s: float = 2.0):
        self._socket_path = socket_path
        self.session_file = session_file
        self.timeout_s = timeout_s
        self.sent: list[str] = []

    def socket_path(self) -> str:
        return self._socket_path or session_socket(self.session_file)

    def request(self, operation: str, role: str | None = None, *, after_id: int | None = None):
        if operation not in ALLOWED_OPERATIONS:
            raise CameraError(f"camera operation {operation!r} is not allowed from the SDK (read-only)")
        body = {"operation": operation}
        if role is not None:
            if role not in ROLES:
                raise CameraError("unknown camera role")
            body["role"] = role
        if after_id is not None:
            body["after_id"] = int(after_id)
        raw = json.dumps(body).encode()
        self.sent.append(operation)
        with socket.socket(socket.AF_UNIX) as conn:
            conn.settimeout(self.timeout_s)
            conn.connect(self.socket_path())
            conn.sendall(struct.pack("!I", len(raw)) + raw)
            size = struct.unpack("!I", _recv_exact(conn, 4))[0]
            if size > MAX_PACKET:
                raise CameraError("camera packet exceeds limit")
            payload = _recv_exact(conn, size)
        if payload.startswith(b"{"):
            result = json.loads(payload)
            if result.get("error"):
                raise CameraError(result["error"])
            return result
        return decode_frame(payload)

    def status(self) -> dict:
        return self.request("status")

    def frame(self, role: str, after_id: int | None = None) -> dict:
        return self.request("frame", role, after_id=after_id)


def intrinsics_digest(intr: dict) -> str:
    return "sha256:" + hashlib.sha256(json.dumps(intr, sort_keys=True).encode()).hexdigest()


class RealSenseCamera:
    """CameraConnector over the broker for one role (top D435i, left wrist D405)."""

    def __init__(self, client: CameraServiceClient, role: str = "top"):
        if role not in ROLES:
            raise ValueError("unknown camera role")
        self.client = client
        self.role = role
        self._identity: dict | None = None

    def latest(self, after_id: int | None = None) -> dict:
        frame = self.client.frame(self.role, after_id)
        if not frame.get("acquisition_timestamp_verified", False):
            raise CameraError("frame has no verified acquisition timestamp")
        self._identity = {"role": self.role, "serial": frame.get("serial"), "session": frame.get("camera_session_id"),
                          "intrinsics_sha256": intrinsics_digest(frame.get("intrinsics") or {}),
                          "depth_scale_m": frame.get("depth_scale_m"), "depth": True,
                          "reference_stream": frame.get("reference_stream")}
        return frame

    def identity(self) -> dict:
        if self._identity is None:
            self.latest()
        return dict(self._identity)
