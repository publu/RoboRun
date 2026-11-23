"""manifest.wire.v1 framing; the binary state matches the wire schema package byte for byte.

Control messages are msgpack maps. The ``state`` uplink is a fixed 272-byte little-endian record (8.2 KB/s at 30 Hz):

  header    24 B  type u8, wire u8, flags u16, seq u32, sdk_t f64, api_seq_seen u32, hb_gap_p999_ms u16, reserved u16
  execution 16 B  segment ref u32, tau f32, scale f32, mode u8, branch u8, guard flags u16
  per arm  116 B x 2  sample id u32, src_t f64, q f32x6, q_cmd f32x6, dq f32x6, jaw f32, jaw_cmd f32, jaw effort f32,
                      temps u8x7, status u8, health u8, interlocks u8, min gap mm u16, sample age ms u16, reserved 6
"""
from __future__ import annotations

import math
import struct
import zlib

from .._optional import require

msgpack = require('msgpack', 'link')

# Constants shared with manifest_wire.state_binary (the two must produce identical bytes; tests cross-decode).
STATE_TYPE = 3
STATE_SIZE = 272
MAX_FRAME = 4 * 1024 * 1024
_HEADER = struct.Struct("<BBHIdIHH")
_EXEC = struct.Struct("<IffBBH")
_ARM = struct.Struct("<Id6f6f6ffff7BBBBHH6x")
assert _HEADER.size == 24 and _EXEC.size == 16 and _ARM.size == 116

MODES = ("IDLE", "ACCEPTING", "EXECUTING", "PAUSED", "HOLDING", "STOPPED", "FAULT")
STATUSES = ("idle", "moving", "holding", "parked", "fault_hold", "offline")
HEALTHS = ("ok", "degraded", "fault", "offline")
FLAG_BITS = ("stopped", "link_lost", "fault", "armed", "link_late", "pack_staged")
GUARD_BITS = ("fresh_pose_hold", "proximity_hold", "clamp", "pause", "thermal", "tick_overrun", "bias", "press")
HEALTH_MAP = {"ok": "ok", "degraded": "degraded", "fault": "fault", "offline": "offline", "stale": "degraded",
              "unknown": "degraded"}
NO_TEMP = 0xFF
NAN = float("nan")


def encode(msg: dict) -> bytes:
    raw = msgpack.packb(msg, use_bin_type=True)
    if len(raw) > MAX_FRAME:
        raise ValueError("frame too large")
    return raw


def decode(raw: bytes):
    if len(raw) == STATE_SIZE and raw[0] == STATE_TYPE:
        return unpack_state(raw)
    if len(raw) > MAX_FRAME:
        raise ValueError("frame too large")
    msg = msgpack.unpackb(raw, raw=False, strict_map_key=False)
    if not isinstance(msg, dict) or not isinstance(msg.get("type"), str):
        raise ValueError("wire frames are maps with a type")
    return msg


def tag32(identifier: str | None) -> int:
    """manifest_wire.tag32: CRC-32 of the UTF-8 id (0 for none)."""
    return 0 if not identifier else zlib.crc32(str(identifier).encode()) & 0xFFFFFFFF


def _bits(names, table) -> int:
    return sum(1 << table.index(n) for n in names if n in table)


def pack_state(state: dict, *, seq: int, api_seq_seen: int = 0, hb_gap_p999_ms: int = 0, arm_order=("left", "right")) -> bytes:
    """The runtime's ``state()`` as the 272-byte frame (byte-identical to manifest_wire.pack_state)."""
    k = state.get("kernel") or {}
    ex = state.get("exec") or {}
    link = k.get("link") or {}
    flags = set()
    if k.get("stopped"):
        flags.add("stopped")
    if link.get("latched_lost"):
        flags.add("link_lost")
    if link.get("state") == "LATE":
        flags.add("link_late")
    if k.get("faults"):
        flags.add("fault")
    if k.get("armed"):
        flags.add("armed")
    guards = state.get("guards") or {}
    running = ex.get("state") not in (None, "ended")
    out = [_HEADER.pack(STATE_TYPE, 1, _bits(flags, FLAG_BITS), seq & 0xFFFFFFFF, float(state.get("sdk_t", 0.0)),
                        api_seq_seen & 0xFFFFFFFF, max(0, min(int(hb_gap_p999_ms), 0xFFFF)), 0),
           _EXEC.pack(tag32(ex.get("segment")) if running else 0, float(ex.get("tau", 0.0)) if running else 0.0,
                      1.0 if running else 0.0, MODES.index(k.get("mode", "IDLE")) if k.get("mode") in MODES else 0, 0,
                      _bits(guards.get("flags") or (), GUARD_BITS))]
    gap = guards.get("min_gap_mm")
    for name in arm_order:
        a = (state.get("arms") or {}).get(name)
        if a is None or a.get("q") is None:
            out.append(_ARM.pack(0, NAN, *(NAN,) * 18, NAN, NAN, NAN, *(NO_TEMP,) * 7,
                                 STATUSES.index("offline"), HEALTHS.index("offline"), 0, 0xFFFF, 0xFFFF))
            continue
        tmax = a.get("temp_max_c")
        temps = [NO_TEMP if tmax is None else max(0, min(254, int(round(tmax))))] + [NO_TEMP] * 6
        inter = a.get("interlocks") or {}
        bits = (1 if inter.get("held") else 0) | (2 if inter.get("pending_release") else 0)
        q_cmd = a.get("q_cmd")
        dq = a.get("dq")
        out.append(_ARM.pack(tag32(a.get("sample")), float(a.get("src_t", NAN)), *[float(v) for v in a["q"]],
                             *([float(v) for v in q_cmd] if q_cmd else [NAN] * 6),
                             *([float(v) for v in dq] if dq else [NAN] * 6),
                             NAN if a.get("jaw") is None else float(a["jaw"]),
                             NAN if a.get("jaw_cmd") is None else float(a["jaw_cmd"]), NAN, *temps,
                             STATUSES.index(a.get("status")) if a.get("status") in STATUSES else STATUSES.index("offline"),
                             HEALTHS.index(HEALTH_MAP.get((a.get("health") or {}).get("state", "unknown"), "degraded")),
                             bits, 0xFFFF if gap is None or not math.isfinite(gap) else max(0, min(0xFFFE, int(round(gap)))),
                             0xFFFF if a.get("age_ms") is None else max(0, min(0xFFFE, int(round(a["age_ms"]))))))
    raw = b"".join(out)
    assert len(raw) == STATE_SIZE
    return raw


def unpack_state(raw: bytes, arm_order=("left", "right")) -> dict:
    if len(raw) != STATE_SIZE or raw[0] != STATE_TYPE:
        raise ValueError("not a state record")
    t, wire_v, flags, seq, sdk_t, api_seen, gap, _ = _HEADER.unpack_from(raw, 0)
    ref, tau, scale, mode, branch, gflags = _EXEC.unpack_from(raw, 24)
    arms = {}
    for i, name in enumerate(arm_order):
        f = _ARM.unpack_from(raw, 40 + i * 116)
        arms[name] = {"sample_tag": f[0], "src_t": f[1], "q": list(f[2:8]), "q_cmd": list(f[8:14]), "dq": list(f[14:20]),
                      "jaw": f[20], "jaw_cmd": f[21], "jaw_effort": f[22], "temps_c": [t for t in f[23:30] if t != NO_TEMP],
                      "status": STATUSES[f[30]], "health": HEALTHS[f[31]],
                      "interlocks": {"held": bool(f[32] & 1), "pending_release": bool(f[32] & 2)},
                      "min_gap_mm": None if f[33] == 0xFFFF else f[33], "age_ms": None if f[34] == 0xFFFF else f[34]}
    return {"type": "state", "wire": wire_v, "seq": seq, "sdk_t": sdk_t, "api_seq_seen": api_seen, "hb_gap_p999_ms": gap,
            "flags": {n: bool(flags >> i & 1) for i, n in enumerate(FLAG_BITS)},
            "exec": {"segment_ref": ref, "tau": tau, "scale": scale, "mode": MODES[mode], "branch": branch,
                     "guards": [n for i, n in enumerate(GUARD_BITS) if gflags >> i & 1]}, "arms": arms}
