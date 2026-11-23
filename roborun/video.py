"""H.264 encoding for the camera MCAP channel (RECORD_EVERYTHING/PERF #5).

Per-frame JPEG is 10–50× larger than a codec stream. This wraps PyAV's libx264
into a tiny push API that yields encoded packets, so the recorder can write a
`foxglove.CompressedVideo` channel instead of `CompressedImage`. Optional dep
(`av`); callers fall back to JPEG when it's absent.
"""
from __future__ import annotations

from fractions import Fraction
from typing import Iterator


def available() -> bool:
    try:
        import av  # noqa: F401
        return True
    except Exception:
        return False


class H264Encoder:
    """Push BGR frames, get H.264 packet bytes. `flush()` drains the tail."""

    def __init__(self, width: int, height: int, fps: int = 30) -> None:
        import av
        self._cc = av.CodecContext.create("libx264", "w")
        self._cc.width = width
        self._cc.height = height
        self._cc.pix_fmt = "yuv420p"
        self._cc.time_base = Fraction(1, fps)
        self._cc.options = {"preset": "veryfast", "tune": "zerolatency",
                            "g": str(fps * 2)}  # ~2s GOP for excerptable clips
        self._av = av
        self._pts = 0
        self.width, self.height = width, height

    def add(self, frame_bgr) -> Iterator[bytes]:
        f = self._av.VideoFrame.from_ndarray(frame_bgr, format="bgr24")
        f.pts = self._pts
        self._pts += 1
        for pkt in self._cc.encode(f):
            yield bytes(pkt)

    def flush(self) -> Iterator[bytes]:
        for pkt in self._cc.encode():  # None flushes
            yield bytes(pkt)
