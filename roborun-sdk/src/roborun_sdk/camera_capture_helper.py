"""RealSense capture helper: the only part of the camera stack that touches the RealSense USB devices.

Run as a stand-alone script (`python -I camera_capture_helper.py ...`): standard library only, plus pyrealsense2 inside the
per-camera workers. It imports nothing of the robot and has no motor, file or command interface. On macOS it is the one
process started through sudo, because the libusb inside pyrealsense2 must detach the system's UVC driver and macOS allows
that only with root or an Apple entitlement. On Linux the same program runs as the normal user.

serve   listens on an owner-only Unix socket in a private directory. It accepts `ping`, `shutdown` and `open <serial>` for the
        serials named on its command line, from the owning user only, and exits once idle.
worker  one process per open camera, with its own SDK context, streaming frames down the connection that asked for it. The
        camera's lifetime is that connection's: a closed connection ends the process and releases the USB device; a crash
        here costs one camera. It never resets a device: resets and retry loops next to streaming cameras have taken down
        the macOS USB stack.
list    prints the connected cameras as JSON.
"""
import argparse
import hashlib
import json
import os
import re
import select
import signal
import socket
import stat
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

MAX_WIDTH, MAX_HEIGHT = 1280, 720
USB2_BYTES_PER_S = 35_000_000      # what a USB 2 link sustains for video in practice
MAX_JSON = 1 << 20
MAX_BLOB = MAX_WIDTH * MAX_HEIGHT * 5 + 1024
FIRST_FRAME_S, STALL_S = 15., 4.
SERIAL = re.compile(r'^[A-Za-z0-9_-]{1,64}$')
UDEV_HELP = ('sudo curl -fsSL -o /etc/udev/rules.d/99-realsense-libusb.rules https://raw.githubusercontent.com/IntelRealSense/'
             'librealsense/master/config/99-realsense-libusb.rules && sudo udevadm control --reload-rules && '
             'sudo udevadm trigger')


class HelperError(Exception):
    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


def version():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()[:12]


# ---- messages: !II (json length, blob length) + json + blob
def _exact(conn, n):
    out = bytearray(n); view = memoryview(out); got = 0
    while got < n:
        k = conn.recv_into(view[got:], n - got)
        if not k:
            raise ConnectionError('capture connection ended')
        got += k
    return out


def send_msg(conn, obj, *buffers):
    head = json.dumps(obj, allow_nan=False).encode()
    views = [memoryview(b).cast('B') for b in buffers]
    conn.sendall(struct.pack('!II', len(head), sum(len(v) for v in views)) + head)
    for v in views:
        conn.sendall(v)


def recv_msg(conn):
    n, blob = struct.unpack('!II', bytes(_exact(conn, 8)))
    if n > MAX_JSON or blob > MAX_BLOB:
        raise ValueError('capture message exceeds limit')
    return json.loads(bytes(_exact(conn, n))), _exact(conn, blob)


def call(path, body, timeout=2.):
    """One request/one reply to a running helper; None when none is listening."""
    try:
        with socket.socket(socket.AF_UNIX) as conn:
            conn.settimeout(timeout); conn.connect(path); send_msg(conn, body)
            return recv_msg(conn)[0]
    except (OSError, ValueError):
        return None


# ---- choosing what to stream
def choose_modes(depth, color, usb_type='', want=None):
    """(depth, color, warning) from the advertised Z16 and RGB8 modes: 640x480 at 30 fps where the link carries it, or
    exactly the width, height and fps in `want`."""
    usable = lambda rows: [r for r in rows if r['width'] <= MAX_WIDTH and r['height'] <= MAX_HEIGHT and r['fps'] >= 6]
    pairs = [(d, c) for d in usable(depth) for c in usable(color) if d['fps'] == c['fps']]
    off = lambda r: abs(r['width'] - 640) + abs(r['height'] - 480)
    if not pairs:
        raise HelperError('mode', 'the camera advertises no matching RGB8 + Z16 modes up to 1280x720')
    want = {k: v for k, v in (want or {}).items() if v}
    if want:
        chosen = [p for p in pairs if all(p[1][k] == v for k, v in want.items())]
        if not chosen:
            have = sorted({(p[1]['width'], p[1]['height'], p[1]['fps']) for p in pairs})
            raise HelperError('mode', f'the camera has no {want} mode. It offers: ' + ', '.join(f'{w}x{h} at {f} fps' for w, h, f in have))
        same = [p for p in chosen if (p[0]['width'], p[0]['height']) == (p[1]['width'], p[1]['height'])]
        pairs = same or chosen
    if not str(usb_type).startswith('2'):
        d, c = min(pairs, key=lambda p: (abs(p[0]['fps'] - 30), off(p[0]) + off(p[1])))
        return d, c, None
    rate = lambda p: (p[0]['width'] * p[0]['height'] * 2 + p[1]['width'] * p[1]['height'] * 3) * p[0]['fps']
    fits = [p for p in pairs if rate(p) <= USB2_BYTES_PER_S
            and (p[0]['width'], p[0]['height']) == (p[1]['width'], p[1]['height'])]
    if not fits:
        raise HelperError('usb2', f'this camera is on a USB {usb_type} link, which cannot carry RGB + depth: '
                                  'connect it to a USB 3 port with a USB 3 cable')
    d, c = min(fits, key=lambda p: (abs(p[0]['fps'] - 15), off(p[0])))
    return d, c, (f"USB {usb_type} link: running at {d['width']}x{d['height']} {d['fps']} fps. "
                  'Use a USB 3 port and cable for 30 fps')


def access_error(exc):
    text = str(exc)
    low = text.lower()
    if 'busy' in low:
        return HelperError('busy', f'the camera is in use by another program: {text}')
    if not any(w in low for w in ('claim', 'access', 'permission', 'power state', 'not permitted')):
        return None
    if sys.platform == 'darwin':
        return HelperError('permission', 'macOS refused USB access to the camera, which needs administrator rights: '
                                         f'run roborun up from a terminal and accept its sudo prompt ({text})')
    return HelperError('permission', f'no USB permission for the camera ({text}). Install the RealSense udev rules, then '
                                     f'replug it: {UDEV_HELP}')


def _info(rs, device, name):
    field = getattr(rs.camera_info, name, None)
    return device.get_info(field) if field is not None and device.supports(field) else None


def _modes(rs, device):
    depth, color = [], []
    for sensor in device.query_sensors():
        for p in sensor.get_stream_profiles():
            if not p.is_video_stream_profile():
                continue
            v = p.as_video_stream_profile()
            row = dict(width=v.width(), height=v.height(), fps=v.fps())
            if p.stream_type() == rs.stream.depth and p.format() == rs.format.z16:
                depth.append(row)
            elif p.stream_type() == rs.stream.color and p.format() == rs.format.rgb8:
                color.append(row)
    return depth, color


def _stamp(frame, offset, received):
    """(acquisition time on this machine's monotonic clock, domain). Sensor time when the SDK maps it to the host clock and
    it is plausible, otherwise the arrival time, named as such."""
    domain = str(frame.get_frame_timestamp_domain()).rsplit('.', 1)[-1]
    if domain in ('global_time', 'system_time'):
        t = float(frame.get_timestamp()) / 1000 + offset
        if 0 <= received - t <= 1.:
            return t, domain
    return received, 'host_arrival'


def _color_sensor(rs, device):
    for sensor in device.query_sensors():
        for p in sensor.get_stream_profiles():
            if p.stream_type() == rs.stream.color:
                return sensor
    return None


def set_exposure(rs, sensor, us):
    """Manual exposure in microseconds. An RGB sensor counts in 100 us steps (UVC), a stereo module in microseconds."""
    name = ''
    if sensor.supports(rs.camera_info.name):
        name = str(sensor.get_info(rs.camera_info.name))
    value = float(us) / 100 if 'rgb' in name.lower() else float(us)
    span = sensor.get_option_range(rs.option.exposure)
    value = max(span.min, min(span.max, value))
    if sensor.supports(rs.option.enable_auto_exposure):
        sensor.set_option(rs.option.enable_auto_exposure, 0)
    sensor.set_option(rs.option.exposure, value)


def _exposure_requests(conn, rs, sensor):
    """The broker asks for exposures on the same connection the frames go down."""
    while select.select([conn], [], [], 0)[0]:
        body = recv_msg(conn)[0]
        if body.get('op') == 'exposure' and sensor is not None:
            try:
                set_exposure(rs, sensor, body['us'])
            except (RuntimeError, AttributeError, KeyError, TypeError, ValueError):
                pass


def stream_camera(conn, serial, rs, settings=None):
    settings = settings or {}
    context = rs.context()
    device = next((d for d in context.query_devices() if _info(rs, d, 'serial_number') == serial), None)
    if device is None:
        raise HelperError('absent', f'RealSense {serial} is not connected')
    usb_type = _info(rs, device, 'usb_type_descriptor') or ''
    d, c, warning = choose_modes(*_modes(rs, device), usb_type,
                                 {k: settings.get(k) for k in ('width', 'height', 'fps')})
    config = rs.config(); config.enable_device(serial)
    config.enable_stream(rs.stream.depth, d['width'], d['height'], rs.format.z16, d['fps'])
    config.enable_stream(rs.stream.color, c['width'], c['height'], rs.format.rgb8, c['fps'])
    pipeline = rs.pipeline(context)
    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        if str(usb_type).startswith('2'):
            raise HelperError('usb2', f'the camera would not start on its USB {usb_type} link ({exc}): '
                                      'connect it to a USB 3 port with a USB 3 cable') from exc
        raise
    try:
        started = profile.get_device()
        clock = getattr(rs.option, 'global_time_enabled', None)
        for sensor in started.query_sensors():
            if clock is not None and sensor.supports(clock):
                sensor.set_option(clock, 1.)
        scale = float(started.first_depth_sensor().get_depth_scale())
        if not 0 < scale < .1:
            raise HelperError('failed', f'invalid factory depth scale {scale}')
        k = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        send_msg(conn, dict(
            event='open', serial=serial, name=_info(rs, device, 'name'), usb_type=usb_type, warning=warning,
            firmware=_info(rs, device, 'firmware_version'), depth_scale_m=scale, profiles=dict(depth=d, color=c),
            intrinsics=dict(width=int(k.width), height=int(k.height), fx=float(k.fx), fy=float(k.fy), ppx=float(k.ppx),
                            ppy=float(k.ppy), distortion_model=str(k.model), coeffs=[float(x) for x in k.coeffs])))
        align = rs.align(rs.stream.color)
        deadline = time.monotonic() + FIRST_FRAME_S; last = None
        color_sensor = _color_sensor(rs, started)
        while True:
            _exposure_requests(conn, rs, color_sensor)
            try:
                frames = pipeline.wait_for_frames(500)
            except RuntimeError as exc:
                if time.monotonic() < deadline:
                    continue
                raise HelperError('stalled', 'the camera stopped delivering frames') from exc
            received = time.monotonic()
            aligned = align.process(frames)
            depth, color = aligned.get_depth_frame(), aligned.get_color_frame()
            if not depth or not color or color.get_frame_number() == last:
                if time.monotonic() >= deadline:
                    raise HelperError('stalled', 'the camera stopped delivering complete RGB + depth frames')
                continue
            w, h = color.get_width(), color.get_height()
            if (depth.get_width(), depth.get_height()) != (w, h) or color.get_stride_in_bytes() != w * 3 \
                    or depth.get_stride_in_bytes() != w * 2:
                raise HelperError('failed', 'the aligned depth does not match the color image layout')
            offset = time.monotonic() - time.time()
            at, domain = _stamp(color, offset, received)
            depth_at, _ = _stamp(depth, offset, received)
            last = color.get_frame_number()
            send_msg(conn, dict(event='frame', width=w, height=h, sensor_frame=int(last),
                                sensor_depth_frame=int(depth.get_frame_number()), acquired_at_s=at,
                                depth_acquired_at_s=depth_at, timestamp_domain=domain, sdk_received_monotonic_s=received),
                     color.get_data(), depth.get_data())
            deadline = time.monotonic() + STALL_S
    finally:
        threading.Timer(3., lambda: os._exit(0)).start()      # a wedged stop must not keep the USB device
        pipeline.stop()


def _load_sdk(sdk_path):
    if sdk_path and sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    try:
        import pyrealsense2
    except ImportError as exc:
        raise HelperError('sdk', f"pyrealsense2 is not available for this computer ({exc})") from exc
    return pyrealsense2


def run_worker(fd, serial, sdk_path, settings=None):
    conn = socket.socket(fileno=fd); conn.settimeout(10.)
    try:
        stream_camera(conn, serial, _load_sdk(sdk_path), settings)
    except (ConnectionError, BrokenPipeError, socket.timeout):
        pass                                                  # the broker went away
    except Exception as exc:
        error = exc if isinstance(exc, HelperError) else access_error(exc) or HelperError('failed', str(exc))
        try:
            send_msg(conn, dict(event='error', kind=error.kind, message=str(error)[:500]))
        except OSError:
            pass
    os._exit(0)


def list_cameras(sdk_path):
    rs = _load_sdk(sdk_path)
    out = []
    for d in rs.context().query_devices():
        try:
            out.append(dict(serial=_info(rs, d, 'serial_number'), name=_info(rs, d, 'name'),
                            usb_type=_info(rs, d, 'usb_type_descriptor'), firmware=_info(rs, d, 'firmware_version')))
        except RuntimeError as exc:
            out.append(dict(error=str(exc)[:200]))
    return out


# ---- the listening process
def owner_uid():
    return int(os.environ.get('SUDO_UID') or os.geteuid())


def check_private_dir(directory, owner):
    info = os.lstat(directory)
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != owner or stat.S_IMODE(info.st_mode) & 0o022:
        raise HelperError('socket', f'{directory} must be a directory owned by the user and writable by nobody else')


def peer_uid(conn):
    if sys.platform == 'darwin':                              # SOL_LOCAL, LOCAL_PEERCRED: struct xucred
        return struct.unpack_from('=II', conn.getsockopt(0, getattr(socket, 'LOCAL_PEERCRED', 1), 76))[1]
    return struct.unpack('3i', conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))[1]


def _bind(path, owner):
    check_private_dir(os.path.dirname(path), owner)
    if call(path, dict(op='ping')):
        raise HelperError('socket', 'a capture helper is already running')
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    server = socket.socket(socket.AF_UNIX)
    previous = os.umask(0o177)
    try:
        server.bind(path)
    finally:
        os.umask(previous)
    if os.geteuid() == 0 and owner != 0:
        os.chown(path, owner, -1, follow_symlinks=False)
    server.listen(8)
    return server


def _daemonize():
    if os.fork():
        print(json.dumps(dict(ready=True, version=version())), flush=True)
        os._exit(0)
    os.setsid(); signal.signal(signal.SIGHUP, signal.SIG_IGN)
    null = os.open(os.devnull, os.O_RDWR)
    for fd in (0, 1, 2):
        os.dup2(null, fd)


def _end(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(4)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(2)


def serve(path, serials, sdk_path, idle_s, daemon):
    owner = owner_uid()
    server = _bind(path, owner)
    if daemon:
        _daemonize()
    else:
        print(json.dumps(dict(ready=True, version=version())), flush=True)
    workers = {}; stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(1))
    idle_since = time.monotonic()
    while not stopping:
        for serial, p in list(workers.items()):
            if p.poll() is not None:
                del workers[serial]; idle_since = time.monotonic()
        if not workers and time.monotonic() - idle_since > idle_s:
            break
        try:
            if not select.select([server], [], [], .5)[0]:
                continue
            conn, _ = server.accept()
        except InterruptedError:
            continue
        with conn:
            try:
                conn.settimeout(2.)
                if peer_uid(conn) not in (owner, 0):
                    continue
                body = recv_msg(conn)[0]
                op = body.get('op') if isinstance(body, dict) else None
                idle_since = time.monotonic()
                if op == 'ping':
                    send_msg(conn, dict(ok=True, version=version(), serials=serials, pid=os.getpid(), root=os.geteuid() == 0))
                elif op == 'shutdown':
                    send_msg(conn, dict(ok=True)); stopping.append(1)
                elif op == 'open' and body.get('serial') in serials:
                    serial = body['serial']
                    if serial in workers:                     # the previous reader is gone: release the device first
                        _end(workers.pop(serial))
                    fd = conn.fileno()
                    argv = [sys.executable, '-I', os.path.abspath(__file__), '--worker', '--fd', str(fd), '--serial', serial,
                            '--settings', json.dumps({k: int(v) for k, v in (body.get('settings') or {}).items()
                                                      if k in ('width', 'height', 'fps') and v})]
                    if sdk_path:
                        argv += ['--sdk-path', sdk_path]
                    workers[serial] = subprocess.Popen(argv, pass_fds=[fd], stdin=subprocess.DEVNULL,
                                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                else:
                    send_msg(conn, dict(event='error', kind='refused', message='not a camera of this robot'))
            except (OSError, ValueError, AttributeError):
                pass
    for p in workers.values():
        _end(p)
    server.close()
    try:
        os.unlink(path)
    except OSError:
        pass


def main(argv=None):
    ap = argparse.ArgumentParser(description='RealSense capture helper of roborun cameras')
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--serve', action='store_true'); mode.add_argument('--worker', action='store_true')
    mode.add_argument('--list', action='store_true')
    ap.add_argument('--socket'); ap.add_argument('--serial', action='append', default=[]); ap.add_argument('--fd', type=int)
    ap.add_argument('--sdk-path'); ap.add_argument('--idle-exit', type=float, default=600.)   # roborun up may first spend minutes updating the runtime
    ap.add_argument('--daemon', action='store_true'); ap.add_argument('--settings', default='{}')
    a = ap.parse_args(argv)
    if any(not SERIAL.match(s) for s in a.serial):
        raise SystemExit('invalid camera serial')
    if a.worker:
        signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
        return run_worker(a.fd, a.serial[0], a.sdk_path, json.loads(a.settings))
    if a.list:
        try:
            print(json.dumps(dict(cameras=list_cameras(a.sdk_path), root=os.geteuid() == 0)))
        except Exception as exc:
            print(json.dumps(dict(error=str(exc)[:300])))
        return 0
    try:
        serve(a.socket, a.serial, a.sdk_path, a.idle_exit, a.daemon)
    except (HelperError, OSError) as exc:
        print(json.dumps(dict(ready=False, error=str(exc)[:300])), flush=True)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
