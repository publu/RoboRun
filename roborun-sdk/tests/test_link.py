"""roborun-link: wire codec, session client over an in-memory transport, capture/encode/upload, pack cache,
and the read-only camera broker client."""
import hashlib
import io
import json
import os
import socket
import struct
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pytest

from roborun_sdk import dev
from roborun_sdk.connectors.realsense import CameraError, CameraServiceClient, RealSenseCamera
from roborun_sdk.link import codec
from roborun_sdk.link.capture import BracketSpec, CaptureRefused, Capturer, bracket, decode_depth, observation
from roborun_sdk.link.pack_cache import PackCache
from roborun_sdk.link.rtclient import InProcessRt, RtClient
from roborun_sdk.link.session import SessionClient, SessionRefused
from roborun_sdk.link.transport import MemoryTransport
from roborun_sdk.link.upload import BlobUploader, UploadRefused
from roborun_sdk.rt.datatypes import Refused
from roborun_sdk.rt.server import RtServer
from roborun_sdk.rt.signing import SigningKey

from conftest import Rig

API_KEY = SigningKey("api-test", hashlib.sha256(b"test api key").digest())


# ---------------------------------------------------------------- codec

def test_state_record_is_272_bytes_and_round_trips(rig):
    rig.rt.submit(rig.segment(seq=1, sid="c", seconds=2.0))
    rig.drive(0.2)
    st = rig.rt.state()
    raw = codec.pack_state(st, seq=7, api_seq_seen=3, hb_gap_p999_ms=120)
    assert len(raw) == 272
    back = codec.decode(raw)
    assert back["seq"] == 7 and back["exec"]["mode"] == "EXECUTING" and back["flags"]["armed"]
    assert np.allclose(back["arms"]["left"]["q"], st["arms"]["left"]["q"], atol=1e-6)
    assert back["arms"]["right"]["status"] == "holding"
    from roborun_sdk.rt import wire
    if wire.available():                                   # byte layout = manifest_wire.state_binary
        w = wire.module()
        frame = w.unpack_state(raw)
        assert frame.seq == 7 and frame.mode == "EXECUTING" and "armed" in frame.flags
        assert frame.segment_ref == w.tag32("c") and frame.arms[0].status == "moving"
        assert w.pack_state(frame) == raw


# ---------------------------------------------------------------- session

class FakeAPI:
    def __init__(self, end):
        self.end = end
        self.inbox = []

    def recv_all(self, timeout=0.05):
        out = []
        while True:
            raw = self.end.recv(timeout)
            if raw is None:
                return out
            out.append(codec.decode(raw))
            self.inbox.append(out[-1])

    def send(self, msg, key=None):
        if key is not None:
            msg = key.sign(msg, msg["type"])
        self.end.send(codec.encode(msg))


def _admit(session="ses_1", **kw):
    msg = {"type": "admit", "wire": 1, "seq": 1, "session": session, "result": "accepted", "transport": "wss",
           "heartbeat_ms": 100, "link_lost_ms": 1000, "state_hz": {"executing": 30, "idle": 2},
           "speed_grant_cap_m_s": 0.10, "pack": {"required": dev.sim_pack().digest, "status": "current"},
           "calibration": {}, "arms": {"left": {"admitted": True}, "right": {"admitted": True}}}
    msg.update(kw)
    return API_KEY.sign(msg, "admit")


def _connect(rig, admit=None):
    sdk_end, api_end = MemoryTransport.pair()
    api = FakeAPI(api_end)
    client = SessionClient(InProcessRt(rig.rt), rig="rig_sim_yam_metal", device="dev_test", clock=rig.clock,
                           admit_keys={API_KEY.kid: API_KEY.public})
    api.send(admit or _admit())
    client.connect(sdk_end, admit_timeout_s=0.5)
    rig.drive(0.05)
    rig.observe()                                               # a (re)connect marks the scene: plan again
    return client, api


def test_hello_admit_segment_ack_state_and_events(rig):
    client, api = _connect(rig)
    hello = api.recv_all()[0]
    assert hello["type"] == "hello" and hello["wire"] == 1 and set(hello["arms"]) == {"left", "right"}
    assert hello["arms"]["right"]["semantics"].startswith("makermods-metal") and len(hello["kernel_sha256"]) == 64
    assert hello["arms"]["left"]["connector"]["kind"] == "yam-station"
    seg = rig.segment(seq=3, sid="via_link")
    api.send(seg)
    client.pump()
    acks = [m for m in api.recv_all() if m.get("type") == "ack"]
    assert acks and acks[0]["result"] == "accepted" and acks[0]["ref"] == "via_link"
    api.send(seg)                                             # duplicate: re-acked, never re-run
    client.pump()
    dup = [m for m in api.recv_all() if m.get("type") == "ack"]
    assert dup[0]["result"] == "duplicate"
    states = [m for m in api.inbox if m.get("type") == "state"]
    assert states and states[-1]["arms"]["left"]["status"] in ("moving", "holding")
    rig.drive(2.0)
    client.pump()
    reports = [m for m in api.recv_all() if m.get("type") == "gate_report"]
    assert reports and reports[0]["outcome"] == "complete" and reports[0]["evidence"]["sdk_outcome"] == "complete"


def test_every_uplink_message_is_wire_valid(rig):
    from roborun_sdk.rt import wire
    if not wire.available():
        pytest.skip("wire schema package not importable")
    w = wire.module()
    client, api = _connect(rig)
    api.send(rig.segment(seq=3, sid="ok"))
    api.send(rig.segment(seq=2, sid="old"))                     # refused (seq regression)
    client.pump(); client.pump()
    rig.drive(0.3)
    rig.right.inject(frozen_stamps=True)                         # fresh-pose hold, then budget end
    rig.drive(1.0)
    rig.right.faults.clear()
    rig.drive(0.3)
    api.send({"type": "stop", "id": "stp_1", "source": "hosted_ui", "user_role": "operator", "scope": "rig"})
    for _ in range(4):
        client.pump()
    msgs = api.recv_all()
    kinds = set()
    for m in api.inbox:
        if m.get("type") == "state":
            continue
        w.parse_message(m)                                       # raises on any schema mismatch
        kinds.add(m["type"] + ":" + str(m.get("kind") or m.get("result") or m.get("outcome") or ""))
    assert {"hello:", "ack:accepted", "ack:refused", "event:fresh_pose_hold", "event:stopped"} <= kinds
    refused = [m for m in api.inbox if m.get("type") == "ack" and m.get("result") == "refused"]
    assert refused[0]["reason"] == "seq_regression"


def test_unsigned_or_foreign_admit_is_refused(rig):
    sdk_end, api_end = MemoryTransport.pair()
    client = SessionClient(InProcessRt(rig.rt), rig="r", device="d", clock=rig.clock, admit_keys={API_KEY.kid: API_KEY.public})
    bad = dict(_admit())
    bad.pop("sig")
    FakeAPI(api_end).send(bad)
    with pytest.raises(SessionRefused):
        client.connect(sdk_end, admit_timeout_s=0.3)


def test_relayed_stop_unsigned_is_obeyed_and_reported(rig):
    client, api = _connect(rig)
    rig.rt.submit(rig.segment(seq=1, sid="s", seconds=2.0))
    api.send({"type": "stop", "id": "stp_9d0e", "source": "hosted_ui", "user_role": "operator", "scope": "rig"})
    client.pump()
    client.pump()
    stopped = [m for m in api.recv_all() if m.get("kind") == "stopped"]
    assert rig.rt.stop_latch.latched and stopped and stopped[0]["ref"] == "stp_9d0e"
    assert stopped[0]["detail"]["verified"] is False and stopped[0]["arms"]["left"]["holding"] is True


def test_reconnect_never_resumes_motion(rig):
    client, api = _connect(rig)
    rig.rt.register_observation("obs_before_loss", rig.clock.now())
    rig.rt.submit(rig.segment(seq=1, sid="r", seconds=3.0))
    rig.drive(0.2)
    rig.drive(1.2, feed_link=False)                          # the link drops: rt holds, latches
    assert rig.rt.playback.outcome == "link_lost"
    n = len(rig.left.writes)
    client2, api2 = _connect(rig, _admit(session="ses_2"))
    hello = api2.recv_all()[0]
    assert hello["latched"]["link_lost"] is True              # the new hello reports the latch ...
    rig.drive(0.5)
    assert len(rig.left.writes) == n                          # ... and nothing resumed
    api2.send(rig.segment(seq=9, sid="old_plan", plan_basis_obs="obs_before_loss"))
    client2.pump()
    ack = [m for m in api2.recv_all() if m.get("type") == "ack"][0]
    assert ack["reason"] == "plan_basis_stale" and ack["detail"]["sdk"]["reason"] == "plan_basis_stale"


def test_rt_socket_round_trip(rig, tmp_path):
    path = str(tmp_path / "rt.sock")
    server = RtServer(rig.rt, path).start()
    try:
        assert oct(os.stat(path).st_mode & 0o777) == "0o600"
        c = RtClient(path)
        info = c.info()
        assert set(info["arms"]) == {"left", "right"} and info["latched"]["stopped"] is False
        ack = c.submit(rig.segment(seq=1, sid="sock"))
        assert ack["result"] == "accepted"
        with pytest.raises(Refused):
            c._call("nope")
        rep = c.stop({"type": "stop", "id": "x"})
        assert rep["source"] == "api:relay" and rig.rt.stop_latch.latched
        c.close()
    finally:
        server.close()


# ---------------------------------------------------------------- capture / upload

class FakeCamera:
    def __init__(self, clock, h=48, w=64):
        self.clock, self.h, self.w = clock, h, w

    def latest(self, after_id=None):
        rgb = (np.arange(self.h * self.w * 3) % 251).astype(np.uint8).reshape(self.h, self.w, 3)
        depth = (np.arange(self.h * self.w) % 4000).astype(np.uint16).reshape(self.h, self.w)
        return {"rgb": rgb, "depth_z16": depth, "acquired_at_s": self.clock.now(), "serial": "123456789012",
                "camera_session_id": "cs_test", "intrinsics": {"width": self.w, "height": self.h, "fx": 600.0, "fy": 600.0},
                "depth_scale_m": 0.001, "acquisition_timestamp_verified": True}


def _samples_fn(rig):
    from roborun_sdk.rt.server import samples

    def fn():
        rig.world.step(1 / 30)
        return samples(rig.rt) | {"now": rig.clock.now()}
    return fn


def test_bracket_accepts_still_arms_and_refuses_motion(rig):
    rig.rt.tick()
    cam = FakeCamera(rig.clock)
    br = bracket(cam, _samples_fn(rig), rig.clock, BracketSpec())
    assert br["bracket"]["stable_span_s"] >= 0.3 and br["bracket"]["stable_joint_range_rad"] <= 0.0015
    manifest, blobs = observation({"capture": "cap_1"}, "top", br)
    assert manifest["privacy"]["face_blur"] is True and set(blobs) == {"rgb", "depth"}
    sha, data, _ = blobs["depth"]
    assert hashlib.sha256(data).hexdigest() == sha
    assert np.array_equal(decode_depth(data, (48, 64)), cam.latest()["depth_z16"])
    rig.left.q_cmd = rig.left.q + 0.01

    def moving():
        rig.world.step(1 / 30)
        rig.rt.tick()
        from roborun_sdk.rt.server import samples
        return samples(rig.rt) | {"now": rig.clock.now()}
    with pytest.raises(CaptureRefused):
        bracket(cam, moving, rig.clock, BracketSpec(deadline_s=0.5))


class BlobSink(BaseHTTPRequestHandler):
    received = []

    def log_message(self, *a):
        pass

    def do_PUT(self):
        n = int(self.headers["Content-Length"])
        BlobSink.received.append((self.path, self.rfile.read(n), self.headers.get("Content-Type")))
        self.send_response(201)
        self.end_headers()


def test_uploader_puts_content_addressed_blobs_in_order(rig):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), BlobSink)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        BlobSink.received.clear()
        up = BlobUploader(f"http://127.0.0.1:{srv.server_address[1]}", token="t", max_bytes_per_s=10e6)
        cam = FakeCamera(rig.clock)
        cap = Capturer({"top": cam}, _samples_fn(rig), rig.clock, uploader=up)
        directive = {"type": "capture", "wire": 1, "seq": 88, "capture": "cap_41", "purpose": "motion_source",
                     "cameras": [{"id": "top", "rgb": {"codec": "jpeg", "quality": 95}, "depth": {"codec": "z16+zstd3"}}],
                     "bracket": {"quiet_s": 0.3, "max_joint_range_rad": 0.0015, "max_flow_p90_px": 0.35}, "upload_order": ["rgb", "state", "depth"],
                     "deadline_s": 2.0}
        manifest = cap(directive)
        from roborun_sdk.link import wire_compat
        from roborun_sdk.rt import wire
        if wire.available():
            wire.module().parse_message(directive)                 # the directive is the wire's own form
            obs = wire_compat.observation(manifest, 5)
            assert obs["type"] == "observation" and obs["camera"]["intrinsics"]["w"] == 64
        paths = [p for p, _, _ in BlobSink.received]
        assert paths == [f"/v1/blobs/{manifest['blobs']['rgb']['sha256']}", f"/v1/blobs/{manifest['blobs']['depth']['sha256']}"]
        assert BlobSink.received[0][2] == "image/jpeg"
        with pytest.raises(UploadRefused):
            up.put("0" * 64, b"not that", "application/octet-stream")
    finally:
        srv.shutdown()
        srv.server_close()
    remote = BlobUploader("https://api.example.invalid")
    with pytest.raises(UploadRefused):                         # frames leave the site only blurred
        remote.upload_observation({"privacy": {"face_blur": False}}, {"rgb": ("x", b"", "image/jpeg")})


# ---------------------------------------------------------------- pack cache

def test_pack_cache_verifies_and_detects_tampering(tmp_path):
    cache = PackCache(tmp_path, dev.dev_root_keys())
    raw = dev.sim_pack_raw()
    dg = cache.put(raw)
    assert cache.get(dg)["pack"] == raw["pack"] and cache.digests() == [dg]
    path = tmp_path / (dg.split(":")[1] + ".json")
    path.write_text(path.read_text().replace("pk_sim_yam_metal_v1", "pk_evil"))
    with pytest.raises(Refused):
        cache.get(dg)
    with pytest.raises(Refused):
        PackCache(tmp_path / "b", {"other": dev.DEV_SEGMENTS.public}).put(raw)


# ---------------------------------------------------------------- camera broker (read-only)

def _fake_broker(path, frames):
    srv = socket.socket(socket.AF_UNIX)
    srv.bind(path)
    srv.listen(4)
    seen = []

    def serve():
        while True:
            try:
                conn, _ = srv.accept()
            except OSError:
                return
            with conn:
                size = struct.unpack("!I", conn.recv(4))[0]
                req = json.loads(conn.recv(size))
                seen.append(req["operation"])
                if req["operation"] == "status":
                    payload = json.dumps({"state": "running", "roles": ["top", "left"], "camera_session_id": "cs1"}).encode()
                else:
                    frame = frames()
                    buf = io.BytesIO()
                    meta = {k: v for k, v in frame.items() if k not in ("rgb", "depth_z16")}
                    np.savez(buf, rgb=frame["rgb"], depth_z16=frame["depth_z16"],
                             metadata=np.frombuffer(json.dumps(meta).encode(), dtype=np.uint8))
                    payload = buf.getvalue()
                conn.sendall(struct.pack("!I", len(payload)) + payload)
    threading.Thread(target=serve, daemon=True).start()
    return srv, seen


def test_realsense_client_reads_status_and_frames_only(rig):
    path = os.path.join(tempfile.mkdtemp(prefix="cam"), "camera.sock")   # short: tmp_path can exceed the socket path limit
    cam = FakeCamera(rig.clock)
    srv, seen = _fake_broker(path, cam.latest)
    try:
        client = CameraServiceClient(path)
        assert client.status()["state"] == "running"
        rs = RealSenseCamera(client, "top")
        frame = rs.latest()
        assert frame["rgb"].shape == (48, 64, 3) and rs.identity()["serial"] == "123456789012"
        for op in ("stop", "restart"):
            with pytest.raises(CameraError):
                client.request(op)
        assert seen == ["status", "frame"]
    finally:
        srv.close()
