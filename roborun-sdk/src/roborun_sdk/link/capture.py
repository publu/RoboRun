"""Observation capture: one exposure bracketed by still arm samples on one clock, encoded and content-addressed.

Bracket (K7): joints move <= 1.5 mrad and the jaw <= 0.003 over >= 0.3 s around the exposure (the window
video_capture uses); scene flow p90 <= 0.35 px when an optical-flow backend is available (cv2, roborun-link only;
otherwise reported as not measured, never assumed). Encoding: RGB to JPEG q95 (Pillow, or cv2 when present), depth
z16 to zstd level 3. Upload order rgb, state, depth, so the API can submit to motion while the depth is in flight.

Privacy: an observation is what leaves this computer, so its image and depth pass the face blur here (camera_privacy.py) and it
is marked ``face_blur: true``. The uploader refuses to send anything without that mark to a non-loopback API.
"""
from __future__ import annotations

import hashlib
import io
import uuid
from dataclasses import dataclass

import numpy as np


class CaptureRefused(RuntimeError):
    pass


@dataclass(frozen=True)
class BracketSpec:
    quiet_s: float = 0.3
    max_joint_range_rad: float = 0.0015
    max_jaw_range: float = 0.003
    max_flow_p90_px: float = 0.35
    deadline_s: float = 2.0

    @classmethod
    def from_directive(cls, msg: dict) -> "BracketSpec":
        b = msg.get("bracket") or {}
        return cls(quiet_s=float(b.get("quiet_s", 0.3)), max_joint_range_rad=float(b.get("max_joint_range_rad", 0.0015)),
                   max_flow_p90_px=float(b.get("max_flow_p90_px", 0.35)), deadline_s=float(msg.get("deadline_s", 2.0)))


def _arms_vec(snap: dict) -> tuple[np.ndarray, np.ndarray]:
    names = sorted(snap["arms"])
    q = np.concatenate([np.asarray(snap["arms"][n]["q"], float) for n in names])
    jaw = np.array([np.nan if snap["arms"][n]["jaw"] is None else snap["arms"][n]["jaw"] for n in names], float)
    return q, jaw


def bracket(camera, samples_fn, clock, spec: BracketSpec, *, poll_s: float = 1 / 30, flow=None) -> dict:
    """Collect still samples for ``quiet_s``, take the next frame, then one more sample. Raises CaptureRefused."""
    start = clock.now()
    window: list = []
    while True:
        snap = samples_fn()
        window.append(snap)
        while len(window) > 2 and window[1]["now"] <= snap["now"] - spec.quiet_s:
            window.pop(0)
        if snap["now"] - window[0]["now"] + 1e-9 >= spec.quiet_s:
            break
        if clock.now() - start > spec.deadline_s:
            raise CaptureRefused("arms did not stay still long enough before the exposure")
        clock.sleep(poll_s)
    before = window[0]
    frame = camera.latest()
    after = samples_fn()
    rows = window + [after]
    qs = np.stack([_arms_vec(r)[0] for r in rows])
    jaws = np.stack([_arms_vec(r)[1] for r in rows])
    joint_range = float(np.max(np.ptp(qs, axis=0)))
    jaw_range = float(np.nanmax(np.ptp(jaws, axis=0))) if np.isfinite(jaws).any() else 0.0
    exposure = float(frame["acquired_at_s"])
    if not before["now"] <= exposure <= after["now"]:
        raise CaptureRefused("the exposure is not inside the still bracket")
    if joint_range > spec.max_joint_range_rad or jaw_range > spec.max_jaw_range:
        raise CaptureRefused(f"arms moved during the bracket ({joint_range * 1000:.2f} mrad, jaw {jaw_range:.4f})")
    flow_p90 = None
    if flow is not None:
        flow_p90 = flow(frame)
        if flow_p90 is not None and flow_p90 > spec.max_flow_p90_px:
            raise CaptureRefused(f"scene moved ({flow_p90:.2f} px p90)")
    return {"frame": frame, "before": before, "after": after,
            "bracket": {"before": round(before["now"], 6), "after": round(after["now"], 6),
                        "stable_span_s": round(after["now"] - before["now"], 4),
                        "stable_joint_range_rad": round(joint_range, 6), "max_jaw_change": round(jaw_range, 6),
                        "scene_motion_p90_px": flow_p90, "clock_uncertainty_s": 0.0}}


def encode_jpeg(rgb: np.ndarray, quality: int = 95) -> bytes:
    try:
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(np.ascontiguousarray(rgb, dtype=np.uint8), "RGB").save(buf, "JPEG", quality=int(quality))
        return buf.getvalue()
    except ImportError:
        import cv2  # roborun-link only
        ok, enc = cv2.imencode(".jpg", np.ascontiguousarray(rgb[..., ::-1]), [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if not ok:
            raise CaptureRefused("JPEG encoding failed")
        return enc.tobytes()


def encode_depth(depth_z16: np.ndarray, level: int = 3) -> bytes:
    import zstandard
    return zstandard.ZstdCompressor(level=level).compress(np.ascontiguousarray(depth_z16, dtype="<u2").tobytes())


def decode_depth(blob: bytes, shape) -> np.ndarray:
    import zstandard
    return np.frombuffer(zstandard.ZstdDecompressor().decompress(blob), dtype="<u2").reshape(shape)


def observation(capture_msg: dict, camera_id: str, br: dict, *, jpeg_quality: int = 95, with_depth: bool = True) -> tuple[dict, dict]:
    """(manifest, blobs). ``blobs`` maps role -> (sha256, bytes, content type)."""
    from ..camera_privacy import clear_arrays, outbound_enabled
    frame = br["frame"]
    clear_rgb, clear_depth = clear_arrays(frame["rgb"], frame.get("depth_z16") if with_depth else None)
    frame = dict(frame, rgb=clear_rgb, depth_z16=clear_depth)
    rgb = encode_jpeg(frame["rgb"], jpeg_quality)
    blobs = {"rgb": (hashlib.sha256(rgb).hexdigest(), rgb, "image/jpeg")}
    if frame.get("depth_z16") is not None:
        d = encode_depth(frame["depth_z16"])
        blobs["depth"] = (hashlib.sha256(d).hexdigest(), d, "application/zstd")
    intr = frame.get("intrinsics") or {}
    arms = {name: {"sample": a["sample"], "q": [round(v, 6) for v in a["q"]], "jaw": a["jaw"]}
            for name, a in br["after"]["arms"].items()}
    manifest = {"type": "observation", "wire": 1, "obs": "obs_" + uuid.uuid4().hex[:12],
                "capture": capture_msg.get("capture"), "kind": "bracketed",
                "camera": {"id": camera_id, "serial": frame.get("serial"), "session": frame.get("camera_session_id"),
                           "intrinsics": {k: intr.get(k) for k in ("width", "height", "fx", "fy", "ppx", "ppy") if k in intr},
                           "depth_scale_m": frame.get("depth_scale_m")},
                "t_exposure": round(float(frame["acquired_at_s"]), 6), "bracket": br["bracket"], "arms": arms,
                "blobs": {role: {"sha256": sha, "codec": "jpeg" if role == "rgb" else "z16+zstd3", "bytes": len(data),
                                 **({"quality": jpeg_quality} if role == "rgb" else {})}
                          for role, (sha, data, _) in blobs.items()},
                "privacy": {"face_blur": outbound_enabled()}}
    return manifest, blobs


class Capturer:
    """Answers ``capture`` directives: bracket, encode, upload (rgb first), return the observation manifest."""

    def __init__(self, cameras: dict, samples_fn, clock, uploader=None, flow=None):
        self.cameras = cameras
        self.samples_fn = samples_fn
        self.clock = clock
        self.uploader = uploader
        self.flow = flow

    def __call__(self, msg: dict) -> dict:
        cams = msg.get("cameras") or [{"id": "top"}]
        spec = BracketSpec.from_directive(msg)
        cam = cams[0]
        camera = self.cameras.get(cam.get("id"))
        if camera is None:
            raise CaptureRefused(f"camera {cam.get('id')!r} is not connected")
        br = bracket(camera, self.samples_fn, self.clock, spec, flow=self.flow)
        quality = int((cam.get("rgb") or {}).get("quality", 95))
        manifest, blobs = observation(msg, cam["id"], br, jpeg_quality=quality, with_depth=cam.get("depth") is not None)
        if self.uploader is not None:
            self.uploader.upload_observation(manifest, blobs, order=msg.get("upload_order") or ("rgb", "state", "depth"))
        return manifest
