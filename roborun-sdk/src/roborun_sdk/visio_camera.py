"""Visio stereo cameras (General Intelligence Labs) as a Roborun camera: RGB + depth from one calibrated pair.

A Visio device (head rig, gripper or wrist unit) streams H.265 camera video and its own calibration over USB serial, and
records the same streams to MCAP. This class reads either, pairs the two eyes by capture time, rectifies them with the
device's calibration and computes depth with a stereo matcher. The broker serves it like any other camera. It reads the
wire itself and decodes each stream from the schema the device announces, so it needs no Visio package and keeps working
across their firmware updates. Read-only: it never writes to the device.

    pip install 'roborun-sdk[visio]'

    cameras:
      top:
        source: roborun_sdk.visio_camera:VisioCamera
        port: /dev/ttyACM0        # a live device (or tcp://HOST:50001 over its USB network); or  mcap: run.mcap
        width: 640                # output size of the rectified left eye; give height too, or it follows the
        height: 360               # calibration's aspect
        root: ego                 # which device root (ego, gripper_left, ...); default: the first calibrated pair
        pair: [1, 0]              # left and right camera indices; Visio's stereo rigs put camera 1 on the left
        min_depth_m: 0.15         # nearest depth the matcher searches for
        matcher: sgbm             # or package.module:function(left_gray, right_gray, max_disparity) -> disparity (px)
        align_rows: true          # fit the small row misalignment a unit's factory calibration leaves (on open)

What it serves: the RECTIFIED left eye as a pinhole camera (distortion "none"), with depth in millimetres aligned to it
and 0 where the matcher found no match. Depth runs at the matcher's speed; each frame carries the capture time of the
pair it came from.

Capture time: a live device stamps frames on its own clock. The offset to this computer's monotonic clock is the lowest
(arrival - capture) seen so far, so a frame's time can be late by the link's smallest delay, never early. A recording
is replayed in real time from when it is opened.
"""
from __future__ import annotations

import importlib
import queue
import re
import threading
import time
from pathlib import Path

import numpy as np

TOPIC = re.compile(r'^/(?:[^/]+/)?(?P<root>[^/]+)/camera/(?P<index>\d+)(?:/(?P<artifact>intrinsics|extrinsics))?$')
FISHEYE = ('equidistant', 'kannala_brandt', 'fisheye')
PINHOLE = ('plumb_bob', 'rational_polynomial', 'none', '')
PAIR_TOLERANCE_NS = 2_000_000          # the two eyes are hardware-triggered together; anything further apart is not a pair
OPEN_WAIT_S = 15.0
ROW_SEARCH_PX = 12                     # matches further apart in rows than this are not the same point
MIN_ROW_MATCHES = 150
ALIGN_PAIRS = 4
CLOCK_JUMP_NS = 1_000_000_000          # a frame this much later than the best seen means the device clock restarted


def _needs(what):
    raise ImportError(f'the Visio camera needs {what}: pip install "roborun-sdk[visio]"')


def _cv2():
    try:
        import cv2
    except ImportError:
        _needs('OpenCV')
    return cv2


def _pose(translation, rotation):
    """foxglove Vector3 + Quaternion -> 4x4, child point -> parent frame."""
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    n = (x * x + y * y + z * z + w * w) ** .5
    if not n:
        raise ValueError('calibration rotation is empty')
    x, y, z, w = x / n, y / n, z / n, w / n
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = translation.x, translation.y, translation.z
    return T


class Calibration:
    """What one device root has announced: per-camera intrinsics and each camera's pose in camera 0."""

    def __init__(self):
        self.intrinsics = {}            # index -> (width, height, model, K 3x3, D)
        self.in_cam0 = {0: np.eye(4)}   # index -> 4x4, p_cam0 = T @ p_cami

    def add(self, index, artifact, msg):
        if artifact == 'intrinsics':
            self.intrinsics[index] = (int(msg.width), int(msg.height), str(msg.distortion_model).lower(),
                                      np.array(msg.K, float).reshape(3, 3), np.array(msg.D, float))
        elif index and msg.ListFields():
            self.in_cam0[index] = _pose(msg.translation, msg.rotation)

    def has(self, left, right):
        return all(i in self.intrinsics and i in self.in_cam0 for i in (left, right))


class Rectifier:
    """Rectification maps for one pair and the pinhole camera they produce."""

    def __init__(self, calibration, left, right, out_width=640, frame_size=None, out_height=None):
        cv2 = _cv2()
        (wl, hl, model, Kl, Dl), (wr, hr, model_r, Kr, Dr) = calibration.intrinsics[left], calibration.intrinsics[right]
        if model != model_r:
            raise ValueError(f'cameras {left} and {right} use different lens models ({model}, {model_r})')
        if (wl, hl) != (wr, hr):
            raise ValueError(f'cameras {left} and {right} are calibrated at different sizes')
        size = frame_size or (wl, hl)
        if size != (wl, hl):                       # the device captures at another resolution than it was calibrated at
            if abs(size[0] / size[1] - wl / hl) > 1e-3:
                raise ValueError(f'frames are {size[0]}x{size[1]} but the calibration is {wl}x{hl}, another aspect')
            s = size[0] / wl
            Kl, Kr = Kl * [[s], [s], [1]], Kr * [[s], [s], [1]]
        out = (int(out_width), int(out_height or round(out_width * size[1] / size[0])))
        # p_right = R @ p_left + T, what OpenCV's stereoRectify takes
        right_from_left = np.linalg.inv(calibration.in_cam0[right]) @ calibration.in_cam0[left]
        R, T = np.ascontiguousarray(right_from_left[:3, :3]), np.ascontiguousarray(right_from_left[:3, 3].reshape(3, 1))
        if model in FISHEYE:
            D1, D2 = Dl[:4].reshape(4, 1), Dr[:4].reshape(4, 1)

            # OpenCV's rotations are used; its projections are not (for a wide lens and an output size other than the
            # input they come out unusable). The output is a centred pinhole keeping the lens's own resolution at the
            # centre: the calibrated focal length scaled to the output, rather than all ~180 degrees squeezed in.
            R1, R2, *_ = cv2.fisheye.stereoRectify(Kl, D1, Kr, D2, size, R, T, cv2.fisheye.CALIB_ZERO_DISPARITY,
                                                   newImageSize=size, balance=0.0, fov_scale=1.0)
            f = (Kl[0, 0] + Kr[0, 0]) / 2 * out[0] / size[0]
            P1 = np.array([[f, 0, out[0] / 2, 0], [0, f, out[1] / 2, 0], [0, 0, 1, 0]])
            P2 = P1.copy()
            P2[0, 3] = f * float((R2 @ T)[0, 0])        # right camera's x in the rectified frame: negative = right of left
            maps = [cv2.fisheye.initUndistortRectifyMap(K, D, Ri, P, out, cv2.CV_32FC1)
                    for K, D, Ri, P in ((Kl, D1, R1, P1), (Kr, D2, R2, P2))]
        elif model in PINHOLE:
            R1, R2, P1, P2, *_ = cv2.stereoRectify(Kl, Dl, Kr, Dr, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
                                                   newImageSize=out)
            maps = [cv2.initUndistortRectifyMap(K, D, Ri, P, out, cv2.CV_32FC1)
                    for K, D, Ri, P in ((Kl, Dl, R1, P1), (Kr, Dr, R2, P2))]
        else:
            raise ValueError(f'lens model {model!r} is not supported (fisheye: {FISHEYE}, pinhole: {PINHOLE})')
        if P2[0, 3] >= 0:
            raise ValueError(f'camera {left} is not left of camera {right}: set pair: [{right}, {left}]')
        self.maps, self.size, self.out = maps, size, out
        self.factory_right = maps[1]
        self.row_fix = None                  # (offset, per x, per y) px the right eye's rows are moved by
        self.fx, self.fy, self.cx, self.cy = P1[0, 0], P1[1, 1], P1[0, 2], P1[1, 2]
        self.baseline_m = -P2[0, 3] / P2[0, 0]

    def intrinsics(self):
        return dict(width=self.out[0], height=self.out[1], fx=float(self.fx), fy=float(self.fy), ppx=float(self.cx),
                    ppy=float(self.cy), distortion_model='none', coeffs=[0.0] * 5)

    def rectify(self, left_rgb, right_rgb):
        cv2 = _cv2()
        return tuple(cv2.remap(img, m1, m2, cv2.INTER_LINEAR) for img, (m1, m2) in zip((left_rgb, right_rgb), self.maps))

    def align_rows(self, pairs):
        """Fit the small row misalignment a unit's factory calibration leaves (left_y - right_y = a + b*x + c*y, from
        features matched between the rectified eyes) and fold it into the right eye's map. Returns what it found; keeps
        the factory maps when the views lack texture or the fit is not clearly better."""
        cv2 = _cv2()
        self.maps[1], self.row_fix = self.factory_right, None
        orb, rows = cv2.ORB_create(4000), []
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        for left_rgb, right_rgb in pairs:
            left, right = (cv2.cvtColor(i, cv2.COLOR_RGB2GRAY) for i in self.rectify(left_rgb, right_rgb))
            kl, dl = orb.detectAndCompute(left, None)
            kr, dr = orb.detectAndCompute(right, None)
            if dl is None or dr is None or len(kr) < 2:
                continue
            for best in matcher.knnMatch(dl, dr, k=2):
                if len(best) == 2 and best[0].distance < 0.75 * best[1].distance:
                    (xl, yl), (xr, yr) = kl[best[0].queryIdx].pt, kr[best[0].trainIdx].pt
                    if 0 < xl - xr < self.out[0] / 3 and abs(yl - yr) < ROW_SEARCH_PX:
                        rows.append((xl, yl, yl - yr))
        found = dict(matches=len(rows), applied=False)
        if len(rows) < MIN_ROW_MATCHES:
            return dict(found, why='too little texture to check the rows')
        x, y, dy = np.array(rows, float).T
        A = np.c_[np.ones_like(x), x - self.out[0] / 2, y - self.out[1] / 2]
        keep = np.ones(len(dy), bool)
        for _ in range(4):
            coef = np.linalg.lstsq(A[keep], dy[keep], rcond=None)[0]
            keep = np.abs(A @ coef - dy) < 1.5
        before, after = float(np.median(np.abs(dy[keep]))), float(np.median(np.abs(dy[keep] - A[keep] @ coef)))
        found.update(before_px=round(before, 2), after_px=round(after, 2), used=int(keep.sum()))
        if keep.sum() < MIN_ROW_MATCHES or before < 0.3 or after > 0.7 * before:
            return dict(found, why='the factory rows already line up' if before < 0.3 else 'no consistent misalignment')
        h, w = self.out[1], self.out[0]
        v, u = np.mgrid[0:h, 0:w].astype(np.float32)
        shift = (coef[0] + coef[1] * (u - w / 2) + coef[2] * (v - h / 2)).astype(np.float32)
        mx, my = self.factory_right       # a feature at right row y belongs on row y + shift: sample from y - shift
        self.maps[1] = (cv2.remap(mx, u, v - shift, cv2.INTER_LINEAR), cv2.remap(my, u, v - shift, cv2.INTER_LINEAR))
        self.row_fix = tuple(float(c) for c in coef)
        return dict(found, applied=True, offset_px=round(coef[0], 2), per_100px_x=round(coef[1] * 100, 3),
                    per_100px_y=round(coef[2] * 100, 3))

    def max_disparity(self, min_depth_m):
        return int(min(256, max(16, -(-self.fx * self.baseline_m / min_depth_m // 16) * 16)))

    def depth_mm(self, disparity):
        disparity = np.asarray(disparity, np.float32)
        depth = np.zeros(disparity.shape, np.float32)
        valid = disparity > 0
        depth[valid] = self.fx * self.baseline_m * 1000.0 / disparity[valid]
        return np.clip(np.rint(depth), 0, 65535).astype(np.uint16)


def sgbm(left_gray, right_gray, max_disparity):
    """OpenCV semi-global matching: disparity in pixels, <= 0 where nothing matched."""
    cv2 = _cv2()
    block = 5
    matcher = cv2.StereoSGBM_create(minDisparity=0, numDisparities=max_disparity, blockSize=block,
                                    P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1, uniquenessRatio=10,
                                    speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    return matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0


def load_matcher(name):
    if name in (None, 'sgbm'):
        return sgbm
    module, _, attr = str(name).partition(':')
    if not attr:
        raise ValueError(f'matcher {name!r}: use sgbm or package.module:function')
    return getattr(importlib.import_module(module), attr)


class _Decoder:
    def __init__(self):
        try:
            import av
        except ImportError:
            _needs('PyAV')
        self.av, self.codec = av, av.CodecContext.create('hevc', 'r')
        self.codec.thread_type = 'SLICE'         # slice threads only: frame threading would hold frames back (latency)

    def decode(self, data):
        try:
            frames = self.codec.decode(self.av.Packet(data))
        except self.av.error.InvalidDataError:   # before the first keyframe
            return None
        return frames[-1].to_ndarray(format='rgb24') if frames else None


# -- the Visio wire, read directly ------------------------------------------------------------------------------------
# A frame on the serial link is COBS(HEADER_LEN u8 | Header protobuf | payload | CRC-16/CCITT-FALSE u16 le), ended by
# 0x00. Header field 1 is the stream id. Stream 1 is the DeviceInfo announce (about once a second): it names every data
# stream's topic and carries its schema as a protobuf FileDescriptorSet, so payloads are decoded from what the device
# says they are. Spec: github.com/General-Intelligence-Labs/visio-schema docs/protocol/framing.md.

DEVICE_INFO = 1
FIRST_DATA_STREAM = 16
SILENT_S = 10.0                        # no bytes for this long, across hangups and reopens: the device has stopped
MAX_FRAME = 8 << 20
CHUNK_QUEUE = 512                      # ~0.5 s of the link at full rate


def _crc_table():
    table = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        table.append(c)
    return table


CRC_TABLE = _crc_table()


def crc16(data):
    c = 0xFFFF
    for b in data:
        c = ((c << 8) & 0xFFFF) ^ CRC_TABLE[(c >> 8) ^ b]
    return c


def cobs_decode(data):
    out, i, n = bytearray(), 0, len(data)
    while i < n:
        code = data[i]
        if code == 0 or i + code > n + (code == 1):
            raise ValueError('bad COBS run')
        out += data[i + 1:i + code]
        i += code
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def _fields(buf):
    """Protobuf wire fields as (number, value): ints for varints and fixed, bytes for length-delimited."""
    i, n = 0, len(buf)
    while i < n:
        key, i = _varint(buf, i)
        number, kind = key >> 3, key & 7
        if kind == 0:
            value, i = _varint(buf, i)
        elif kind == 2:
            size, i = _varint(buf, i)
            value, i = buf[i:i + size], i + size
        elif kind in (1, 5):
            width = 8 if kind == 1 else 4
            value, i = int.from_bytes(buf[i:i + width], 'little'), i + width
        else:
            raise ValueError(f'protobuf wire type {kind}')
        yield number, value


def _varint(buf, i):
    value = shift = 0
    while True:
        b = buf[i]
        i += 1
        value |= (b & 0x7F) << shift
        if b < 0x80:
            return value, i
        shift += 7


def unframe(frame):
    """One COBS-encoded frame (without its 0x00) -> (stream_id, payload), or None when it is damaged."""
    try:
        core = cobs_decode(frame)
    except ValueError:
        return None
    if len(core) < 3 or crc16(core[:-2]) != int.from_bytes(core[-2:], 'little'):
        return None
    header_len = core[0]
    stream_id = next((v for k, v in _fields(core[1:1 + header_len]) if k == 1), 0)
    return stream_id, core[1 + header_len:-2]


def device_info(payload):
    """DeviceInfo -> (device_name, {stream_id: (topic, schema_name, FileDescriptorSet bytes)})."""
    name, channels = '', {}
    for number, value in _fields(payload):
        if number == 1:
            name = value.decode()
        elif number == 2:
            ch = dict(_fields(value))
            channels[ch.get(1, 0)] = (ch.get(2, b'').decode(), ch.get(4, b'').decode(), ch.get(5, b''))
    return name, channels


class Schemas:
    """Message classes built from the schemas the device (or the recording) carries."""

    def __init__(self):
        try:
            from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
        except ImportError:
            _needs('protobuf')
        self.descriptor_pb2, self.factory = descriptor_pb2, message_factory
        self.pool, self.files, self.classes = descriptor_pool.DescriptorPool(), set(), {}

    def decode(self, schema_name, schema, payload):
        cls = self.classes.get(schema_name)
        if cls is None:
            fds = self.descriptor_pb2.FileDescriptorSet.FromString(schema)
            for f in fds.file:
                if f.name not in self.files:
                    self.pool.Add(f)
                    self.files.add(f.name)
            cls = self.classes[schema_name] = self.factory.GetMessageClass(self.pool.FindMessageTypeByName(schema_name))
        return cls.FromString(payload)


def _drain(port, stopping, chunks):
    """Read the port as fast as it delivers, in its own thread: the kernel's serial buffer is small, and at ~2 MB/s any
    pause (decoding video, matching stereo) overflows it and corrupts frames. Reopens through hangups and USB blips;
    after SILENT_S without a byte it hands the consumer a ConnectionError."""
    import os
    import select
    import termios
    import tty
    heard = time.monotonic()
    while not stopping.is_set():
        fd = None
        try:
            fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            tty.setraw(fd)
            attrs = termios.tcgetattr(fd)
            attrs[2] &= ~termios.HUPCL        # closing must not drop the line: the next open would read a hangup
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            while not stopping.is_set() and time.monotonic() - heard < SILENT_S:
                if not select.select([fd], [], [], 0.5)[0]:
                    continue
                chunk = os.read(fd, 1 << 20)
                if not chunk:
                    break                          # hangup (a USB blip, or the previous close): reopen
                heard = time.monotonic()
                try:
                    chunks.put_nowait(chunk)
                except queue.Full:                 # the consumer is far behind: drop the oldest, keep the newest
                    try:
                        chunks.get_nowait()
                    except queue.Empty:
                        pass
                    chunks.put_nowait(chunk)
        except OSError:                            # not there (re-enumerating) or a read error: reopen
            pass
        finally:
            if fd is not None:
                os.close(fd)
        if time.monotonic() - heard >= SILENT_S:
            chunks.put(ConnectionError(f'Visio device at {port} sent nothing for {SILENT_S:.0f} s'))
            return
        stopping.wait(0.3)


def _drain_tcp(host, port, stopping, chunks):
    """The same stream over the device's USB network link (or Wi-Fi): TCP, same framing. Sends one empty frame (a single
    0x00) after connecting, as the protocol asks, to resync the device's reader; nothing else is ever written."""
    import socket
    heard = time.monotonic()
    while not stopping.is_set():
        sock = None
        try:
            sock = socket.create_connection((host, int(port)), timeout=3)
            sock.sendall(b'\x00')
            sock.settimeout(0.5)
            while not stopping.is_set() and time.monotonic() - heard < SILENT_S:
                try:
                    chunk = sock.recv(1 << 20)
                except socket.timeout:
                    continue
                if not chunk:
                    break                              # closed by the device: reconnect
                heard = time.monotonic()
                try:
                    chunks.put_nowait(chunk)
                except queue.Full:
                    try:
                        chunks.get_nowait()
                    except queue.Empty:
                        pass
                    chunks.put_nowait(chunk)
        except OSError:
            pass
        finally:
            if sock is not None:
                sock.close()
        if time.monotonic() - heard >= SILENT_S:
            chunks.put(ConnectionError(f'Visio device at {host}:{port} sent nothing for {SILENT_S:.0f} s'))
            return
        stopping.wait(0.3)


def header_time_ns(frame_core_header):
    """The capture time a message's wire header carries (Header field 3, a protobuf Timestamp), in ns; None if absent."""
    for number, value in _fields(frame_core_header):
        if number == 3 and isinstance(value, (bytes, bytearray)):
            f = dict(_fields(value))
            return int(f.get(1, 0)) * 1_000_000_000 + int(f.get(2, 0))
    return None


def device_rows(address, stopping, with_time=False):
    """A live device at a serial port (/dev/ttyACM0) or tcp://HOST:PORT: yields (device_name, topic, schema_name, schema,
    payload) for every data message it announces; with_time adds (arrival_monotonic_ns, header_capture_ns)."""
    chunks = queue.Queue(maxsize=CHUNK_QUEUE)
    if str(address).startswith('tcp://'):
        host, _, port = str(address)[6:].rpartition(':')
        threading.Thread(target=_drain_tcp, args=(host, port or 50001, stopping, chunks), daemon=True, name='visio-tcp').start()
    else:
        threading.Thread(target=_drain, args=(address, stopping, chunks), daemon=True, name='visio-serial').start()
    name, channels, pending = '', {}, b''
    while not stopping.is_set():
        try:
            chunk = chunks.get(timeout=0.5)
        except queue.Empty:
            continue
        if isinstance(chunk, Exception):
            raise chunk
        arrival = time.monotonic_ns()
        *frames, pending = (pending + chunk).split(b'\x00')
        if len(pending) > MAX_FRAME:
            pending = b''
        for frame in frames:
            if not frame:
                continue
            try:
                core = cobs_decode(frame)
            except ValueError:
                continue
            if len(core) < 3 or crc16(core[:-2]) != int.from_bytes(core[-2:], 'little'):
                continue
            header = core[1:1 + core[0]]
            stream_id = next((v for k, v in _fields(header) if k == 1), 0)
            payload = core[1 + core[0]:-2]
            if stream_id == DEVICE_INFO:
                name, announced = device_info(payload)
                channels.update(announced)
            elif stream_id >= FIRST_DATA_STREAM and stream_id in channels:
                row = (name, *channels[stream_id], payload)
                yield (*row, arrival, header_time_ns(header)) if with_time else row


def serial_rows(port, stopping):
    """A live device on a serial port (kept for callers that name it so)."""
    return device_rows(port, stopping)


def mcap_rows(path):
    """A recording: the same rows, from MCAP's own channel and schema records."""
    try:
        from mcap.reader import make_reader
    except ImportError:
        _needs('mcap')
    with open(path, 'rb') as f:
        for schema, channel, message in make_reader(f).iter_messages(log_time_order=True):
            if schema is not None:
                yield '', channel.topic, schema.name, schema.data, message.data


class VisioCamera:
    def __init__(self, role='top', port=None, mcap=None, width=640, height=None, root=None, pair=(1, 0),
                 min_depth_m=0.15, matcher='sgbm', align_rows=True, fps=None):
        if bool(port) == bool(mcap):
            raise ValueError('Visio camera: give exactly one of port (a live device) or mcap (a recording)')
        if mcap and not Path(mcap).expanduser().is_file():
            raise ValueError(f'Visio camera: no recording at {mcap}')
        self.role, self.port = role, port
        self.mcap = str(Path(mcap).expanduser()) if mcap else None
        self.width, self.height, self.root, self.min_depth_m = int(width), height, root, float(min_depth_m)
        self.root_given = root
        self.align, self.alignment = bool(align_rows), None
        self.left, self.right = (int(i) for i in pair)
        self.matcher = load_matcher(matcher)
        self.calibrations = {}                  # root -> Calibration
        self.frames = {}                        # (root, index) -> (capture_ns, rgb)
        self.newest = None                      # (capture time on this computer's monotonic clock in ns, left, right)
        self.served_ns = None
        self.offset_ns = None                   # host monotonic - device capture, lowest seen
        self.rectifier = None
        self.device = ''
        self.error = None
        self.changed = threading.Condition()
        self.stopping = threading.Event()
        self.thread = None

    # -- the reader thread: decode both eyes, keep the newest pair ----------------------------------------------------
    def _run(self):
        decoders, schemas = {}, Schemas()
        try:
            while not self.stopping.is_set():
                started, first_ns = time.monotonic_ns(), None
                rows = device_rows(self.port, self.stopping) if self.port else mcap_rows(self.mcap)
                for device, topic, schema_name, schema, data in rows:
                    if self.stopping.is_set():
                        return
                    hit = TOPIC.match(topic or '')
                    if not hit or (self.root and hit['root'] != self.root):
                        continue
                    root, index, artifact = hit['root'], int(hit['index']), hit['artifact']
                    if device:
                        self.device = device
                    if artifact:
                        with self.changed:
                            self.calibrations.setdefault(root, Calibration()).add(
                                index, artifact, schemas.decode(schema_name, schema, data))
                            self.changed.notify_all()
                        continue
                    if index not in (self.left, self.right) or schema_name != 'foxglove.CompressedVideo':
                        continue
                    payload = schemas.decode(schema_name, schema, data)
                    t_ns = payload.timestamp.seconds * 1_000_000_000 + payload.timestamp.nanos
                    if self.mcap:                                       # replay in real time from when it opened
                        first_ns = t_ns if first_ns is None else first_ns
                        host_ns = started + (t_ns - first_ns)
                        wait = host_ns - time.monotonic_ns()
                        if wait > 0 and self.stopping.wait(wait / 1e9):
                            return
                    else:                                               # device clock -> ours: lowest arrival - capture
                        lag = time.monotonic_ns() - t_ns
                        if self.offset_ns is None or lag < self.offset_ns or lag - self.offset_ns > CLOCK_JUMP_NS:
                            self.offset_ns = lag                        # first frame, a faster one, or the device rebooted
                        host_ns = t_ns + self.offset_ns
                    image = decoders.setdefault((root, index), _Decoder()).decode(payload.data)
                    if image is not None:
                        self._frame(root, index, t_ns, host_ns, image)
                if not self.mcap:
                    raise ConnectionError(f'Visio device at {self.port} stopped sending')
                decoders.clear()                                        # loop the recording from its first keyframe
                with self.changed:
                    self.frames.clear()
        except Exception as exc:  # noqa: BLE001 - reported to the broker on the next read
            with self.changed:
                self.error = exc
                self.changed.notify_all()

    def _frame(self, root, index, t_ns, host_ns, image):
        with self.changed:
            self.frames[(root, index)] = (t_ns, image)
            other = self.frames.get((root, self.right if index == self.left else self.left))
            if other and abs(other[0] - t_ns) <= PAIR_TOLERANCE_NS and (self.root in (None, root)):
                left, right = (image, other[1]) if index == self.left else (other[1], image)
                self.root = root
                self.newest = (host_ns, left, right)
                self.changed.notify_all()

    # -- the camera interface Roborun calls ----------------------------------------------------------------------------
    def open(self):
        self.close()                            # a reopen (the broker retrying) starts clean, never from a past failure
        with self.changed:
            self.calibrations, self.frames, self.newest, self.served_ns = {}, {}, None, None
            self.offset_ns, self.rectifier, self.error = None, None, None
            self.root = self.root_given
        self.stopping.clear()
        self.thread = threading.Thread(target=self._run, daemon=True, name=f'visio-{self.role}')
        self.thread.start()
        deadline = time.monotonic() + OPEN_WAIT_S
        with self.changed:
            while True:
                if self.error:
                    raise self.error
                cal = self.calibrations.get(self.root) if self.root else None
                if self.newest is not None and cal and cal.has(self.left, self.right):
                    break
                left_over = deadline - time.monotonic()
                if left_over <= 0:
                    found = {r: sorted(c.intrinsics) for r, c in self.calibrations.items()}
                    raise TimeoutError(f'Visio camera: no calibrated pair {self.left},{self.right} with frames within '
                                       f'{OPEN_WAIT_S:.0f} s (calibrated cameras seen: {found or "none"})')
                self.changed.wait(left_over)
            h, w = self.newest[1].shape[:2]
        self.rectifier = Rectifier(cal, self.left, self.right, self.width, (w, h), self.height)
        self.alignment = self.rectifier.align_rows(self._fresh_pairs(ALIGN_PAIRS)) if self.align else None
        where = self.device or self.port or Path(self.mcap).name
        return dict(serial=f'visio:{where}:{self.root}', intrinsics=self.rectifier.intrinsics(), depth_scale_m=0.001)

    def _fresh_pairs(self, n, timeout_s=3.0):
        pairs, seen, deadline = [], None, time.monotonic() + timeout_s
        with self.changed:
            while len(pairs) < n and time.monotonic() < deadline and not self.error:
                if self.newest is not None and self.newest[0] != seen:
                    seen = self.newest[0]
                    pairs.append(self.newest[1:])
                self.changed.wait(0.1)
        return pairs

    def read(self):
        with self.changed:
            if self.error:
                raise self.error
            if self.newest is None or self.newest[0] == self.served_ns:
                return None
            host_ns, left, right = self.newest
            self.served_ns = host_ns
        cv2 = _cv2()
        left, right = self.rectifier.rectify(left, right)
        disparity = self.matcher(cv2.cvtColor(left, cv2.COLOR_RGB2GRAY), cv2.cvtColor(right, cv2.COLOR_RGB2GRAY),
                                 self.rectifier.max_disparity(self.min_depth_m))
        at = host_ns / 1e9
        return dict(rgb=left, depth_z16=self.rectifier.depth_mm(disparity), acquired_at_s=at, depth_acquired_at_s=at,
                    timestamp_domain='visio_device_clock')

    def close(self):
        self.stopping.set()
        if self.thread is not None:
            self.thread.join(timeout=2)
