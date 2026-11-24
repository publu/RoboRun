"""`roborun cameras`: the robot's real cameras, served to the runtime over the camera broker protocol.

`roborun up` runs `roborun cameras --hardware FILE --socket PATH`. Each camera is captured continuously by its own thread
and only the newest frame is kept. A camera that fails is reported in `status` and on its frame requests while the others
keep streaming, and it comes back by itself. Frames are served sharp: face blur happens where a frame leaves this computer
(camera_privacy.py).

Sources (`roborun hw detect` prints the connected ones):
  realsense:SERIAL  RGB + depth aligned to RGB, factory intrinsics. Captured by camera_capture_helper.py, one process per
                    camera; on macOS that helper is the only thing that runs as root (one sudo prompt at `roborun up`).
  usb:ID            a plain USB camera through OpenCV: RGB only. Linux: a /dev/v4l/by-id name. macOS: the AVFoundation
                    unique ID.
  rtsp://... http://... https://... file:///...   any address OpenCV can open; stream:ADDRESS for any other form.
                    RGB only. Passwords in an address are never logged or put in frames.
  my_cam.py:MyCamera   your own class, also package.module:MyCamera (see CustomSource).
  roborun_sdk.visio_camera:VisioCamera   a Visio stereo device (USB serial or an MCAP recording): rectified RGB + depth
                    from its own calibration. pip install 'roborun-sdk[visio]'; see visio_camera.py.

A camera in robot.yaml is a source string, or:
  top: {source: SOURCE, width: 640, height: 480, fps: 30, rotate: 90,
        exposure: {managed: true, target_median: 110, limit_us: 20000, seed_us: 8000, smooth_idle: 0.15}}
Cameras are named top, left and right, or any name of your own (lower-case letters, digits and _) with an optional
card: {alias: top|left|right, label, mount: world|left.tool|right.tool, pose, depth, rectified, sees, device} saying where it
is and which task camera it answers as. `roborun cameras describe` prints every camera's card (camera_card.py).

A camera without depth is declared, not disguised. The wire format requires a depth array, so it carries zeros (the
RealSense value for "no measurement"), together with depth_available=False and depth_scale_m=None. A number there would make
the zeros read as real distances of 0 m. With None every metric use fails at once. Its intrinsics are None for the same reason: an uncalibrated camera has no focal length to offer.

Linux setup for RealSense (once):
  sudo curl -fsSL -o /etc/udev/rules.d/99-realsense-libusb.rules \\
    https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger     (then replug the cameras)
"""
import importlib
import importlib.util
import inspect
import io
import json
import os
import plistlib
import re
import signal
import socket
import socketserver
import struct
import subprocess
import sys
import threading
import time
import urllib.parse
import uuid
from pathlib import Path

from . import camera_capture_helper as helper
from . import net
from . import camera_privacy as privacy
from ._optional import lazy
from .camera_exposure import ExposureController

np = lazy('numpy', 'cameras', globals())   # imported when a frame is first handled: the module itself loads without numpy

ROLES = ('top', 'left', 'right')
STREAM_SCHEMES = ('rtsp', 'rtsps', 'rtmp', 'http', 'https', 'udp', 'tcp', 'file')
CUSTOM_SOURCE = re.compile(r'(.+\.py|[A-Za-z_][\w.]*):[A-Za-z_]\w*')
CAMERA_KEYS = {'source', 'width', 'height', 'fps', 'rotate', 'exposure', 'card'}    # card: camera_card.py, never a class's
EXPOSURE_KEYS = {'managed', 'target_median', 'limit_us', 'seed_us', 'smooth_idle'}
ROTATIONS = (0, 90, 180, 270)
STREAM_OPEN_MS, STREAM_READ_MS = 10000, 5000
MAX_PACKET = 64 << 20
MAX_WIDTH, MAX_HEIGHT = helper.MAX_WIDTH, helper.MAX_HEIGHT
FRESH_S = 1.0                  # an older frame is not served as current
STALL_S = 5.0                  # no frame for this long: the camera has failed
SUDO = '/usr/bin/sudo'
INSTALL = 'it is not available for this computer (RealSense on a Mac needs Apple Silicon and macOS 15 or newer)'
SUDO_PROMPT = ('Roborun: the RealSense cameras need administrator access for USB on macOS. Only the small camera capture '
               'helper runs as root, never the robot.\nPassword for %u: ')
UDEV_RULES = '99-realsense-libusb.rules'
UDEV_HELP = helper.UDEV_HELP
MAC_CAMERA_DENIED = ('macOS has not allowed this program to use the camera. Open System Settings > Privacy & Security > '
                     'Camera, turn on the app you run roborun from (Terminal, iTerm, ...), then run roborun up again')


class CameraError(Exception):
    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


# ---- the broker's wire protocol (the runtime's camera_service client speaks the same)
def _exact(conn, n):
    out = bytearray()
    while len(out) < n:
        block = conn.recv(n - len(out))
        if not block:
            raise ConnectionError('Camera connection ended')
        out.extend(block)
    return bytes(out)


def receive(conn, limit=4096):
    size = struct.unpack('!I', _exact(conn, 4))[0]
    if size > limit:
        raise ValueError('Camera packet exceeds limit')
    return _exact(conn, size)


def send(conn, payload):
    if len(payload) > MAX_PACKET:
        raise ValueError('Camera response exceeds limit')
    conn.sendall(struct.pack('!I', len(payload)) + payload)


def encode_frame(frame):
    metadata = {k: v for k, v in frame.items() if k not in ('rgb', 'depth_z16')}
    out = io.BytesIO()
    np.savez(out, rgb=frame['rgb'], depth_z16=frame['depth_z16'],
             metadata=np.frombuffer(json.dumps(metadata, allow_nan=False).encode(), dtype=np.uint8))
    return out.getvalue()


# ---- sources
class Source:
    """What the broker needs from a camera: open() -> model, read() -> frame fields or None, close(), and the retry policy."""
    settings = {}
    exposure_control = False              # True: set_exposure_us(us) works

    def close(self):
        pass

    def snapshot(self):
        return None

    def may_retry(self, error, seen):
        return True

    def retry_hint(self, error):
        return 'retrying'


def redact(address):
    """An address with its password and query values taken out: this is the form that is logged and put in frames."""
    try:
        parts = urllib.parse.urlsplit(address)
        if not parts.scheme or not parts.netloc:
            return address
        host = parts.netloc.rpartition('@')[2]
        query = '&'.join(f"{item.partition('=')[0]}=***" for item in parts.query.split('&')) if parts.query else ''
        return urllib.parse.urlunsplit((parts.scheme, host, parts.path, query, ''))
    except ValueError:
        return 'stream'


def parse_source(text):
    text = str(text)
    kind, sep, ident = text.partition(':')
    if sep and ident and kind in ('realsense', 'usb', 'stream'):
        if kind == 'realsense' and not helper.SERIAL.match(ident):
            raise ValueError(f'camera source {text!r}: not a RealSense serial number')
        return kind, ident
    if sep and ident.startswith('//') and kind.lower() in STREAM_SCHEMES:
        return 'stream', text
    if CUSTOM_SOURCE.fullmatch(text):
        return 'custom', text
    raise ValueError(f'camera source {redact(text)!r}: use realsense:SERIAL, usb:ID, an address such as rtsp://HOST/PATH, or '
                     'your own class as my_cam.py:MyCamera (roborun hw detect shows connected cameras)')


def _number(role, key, value, low, high, whole=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or (whole and int(value) != value) or not low <= value <= high:
        kind = 'a whole number' if whole else 'a number'
        raise ValueError(f'camera {role}: {key} must be {kind} from {low} to {high}, not {value!r}')
    return int(value) if whole else float(value)


def camera_settings(role, entry):
    """(checked settings, extra keys) for one camera, written as a source string or a section."""
    entry = dict(source=entry) if isinstance(entry, str) else entry
    if not isinstance(entry, dict) or not entry.get('source'):
        raise ValueError(f'camera {role}: name its source, e.g. {role}: {{source: usb:0}}')
    out = dict(source=str(entry['source']))
    if entry.get('width') is not None or entry.get('height') is not None:
        if entry.get('width') is None or entry.get('height') is None:
            raise ValueError(f'camera {role}: set width and height together')
        out['width'] = _number(role, 'width', entry['width'], 16, MAX_WIDTH, True)
        out['height'] = _number(role, 'height', entry['height'], 16, MAX_WIDTH, True)
    if entry.get('fps') is not None:
        out['fps'] = _number(role, 'fps', entry['fps'], 1, 240, True)
    rotate = entry.get('rotate') or 0
    if rotate not in ROTATIONS:
        raise ValueError(f'camera {role}: rotate must be 0, 90, 180 or 270, not {rotate!r}')
    out['rotate'] = int(rotate)
    extra = {k: v for k, v in entry.items() if k not in CAMERA_KEYS}
    exposure = entry.get('exposure')
    if exposure is not None:
        if not isinstance(exposure, dict):
            raise ValueError(f'camera {role}: exposure must be a section with managed, target_median, limit_us, seed_us, smooth_idle')
        checked = dict(managed=exposure.get('managed', True))
        if not isinstance(checked['managed'], bool):
            raise ValueError(f"camera {role}: exposure managed must be true or false, not {checked['managed']!r}")
        for key, low, high in (('target_median', 1, 254), ('limit_us', 50, 1_000_000), ('seed_us', 50, 1_000_000), ('smooth_idle', .01, 1)):
            if exposure.get(key) is not None:
                checked[key] = _number(role, f'exposure {key}', exposure[key], low, high)
        if checked.get('seed_us', 0) > checked.get('limit_us', float('inf')):
            raise ValueError(f'camera {role}: exposure seed_us is above limit_us')
        out['exposure'] = checked
        extra.update({f'exposure.{k}': v for k, v in exposure.items() if k not in EXPOSURE_KEYS})
    return out, extra


def camera_configs(hardware):
    """{role: (kind, ident, settings, extra keys)}, checked before anything starts. The role is the camera's alias (top, left,
    right: the ids top/left/right are their own) or, for a camera of your own name with no alias, its id. settings['id'] is
    the robot.yaml id and settings['card'] its checked card (camera_card.check_config)."""
    from . import camera_card
    cameras = (hardware or {}).get('cameras') or {}
    if not isinstance(cameras, dict):
        raise ValueError('cameras must be a section: top, left, right or your own camera names')
    cards, _ = camera_card.check_config(cameras)
    out, seen = {}, set()
    for cid, entry in cameras.items():
        settings, extra = camera_settings(cid, entry)
        kind, ident = parse_source(settings['source'])
        if kind != 'custom' and (kind, ident) in seen:         # your own class may serve several cameras, one instance per role
            raise ValueError('the same camera is assigned to two roles')
        seen.add((kind, ident))
        role = cards[cid]['alias'] or cid
        if role in out:
            raise ValueError(f'camera {cid}: the name {role} is already a camera\'s name or alias')
        if cid != role or (isinstance(entry, dict) and entry.get('card') is not None):     # an old entry's settings stay as they were
            settings = dict(settings, id=cid, card=cards[cid])
        out[role] = (kind, ident, settings, extra)
    return out


def parse_cameras(hardware):
    return {role: (kind, ident) for role, (kind, ident, _, _) in camera_configs(hardware).items()}


def setting_warnings(configs):
    """Unknown keys on a built-in source are ignored, and said out loud: a custom camera class receives them instead."""
    return [f'camera {role}: ignoring unknown setting {key!r}' for role, (kind, _, _, extra) in configs.items()
            for key in extra if kind != 'custom' or '.' in key]


def face_blur_enabled(hardware):
    value = ((hardware or {}).get('privacy') or {}).get('face_blur', True)
    if not isinstance(value, bool):
        raise ValueError(f'privacy face_blur must be true or false, not {value!r}')
    return value


def rotate_image(image, rotate):
    if not rotate:
        return image
    import cv2
    return cv2.rotate(image, {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}[rotate])


def rotate_intrinsics(k, rotate):
    """The camera model of the image after a clockwise rotation, so projection stays exact."""
    if not rotate or not k:
        return k
    w, h, out = k['width'], k['height'], dict(k)
    if rotate in (90, 270):
        out.update(width=h, height=w)
    if k.get('fx') is None:
        return out
    fx, fy, cx, cy = k['fx'], k['fy'], k['ppx'], k['ppy']
    if rotate == 90:
        out.update(fx=fy, fy=fx, ppx=h - 1 - cy, ppy=cx)
    elif rotate == 180:
        out.update(ppx=w - 1 - cx, ppy=h - 1 - cy)
    else:
        out.update(fx=fy, fy=fx, ppx=cy, ppy=w - 1 - cx)
    coeffs = k.get('coeffs')
    if coeffs and len(coeffs) >= 4 and 'brown' in str(k.get('distortion_model')):
        p1, p2 = coeffs[2], coeffs[3]
        p = {90: (p2, -p1), 180: (-p1, -p2), 270: (-p2, p1)}[rotate]
        out['coeffs'] = [*coeffs[:2], *p, *coeffs[4:]]
    return out


def rotate_profiles(profiles, rotate):
    if rotate not in (90, 270) or not profiles:
        return profiles
    return {name: dict(row, width=row['height'], height=row['width']) if isinstance(row, dict) and 'width' in row else row
            for name, row in profiles.items()}


def realsense_bus(platform=None, run=subprocess.run):
    """The RealSense devices on USB as a set of (place, product id), read without opening any of them."""
    platform = platform or sys.platform
    found = set()
    if platform == 'darwin':
        try:
            data = plistlib.loads(run(['/usr/sbin/ioreg', '-a', '-p', 'IOUSB', '-l'], capture_output=True, timeout=10).stdout)
        except (OSError, ValueError, subprocess.SubprocessError, plistlib.InvalidFileException):
            return found
        def walk(node):
            if isinstance(node, list):
                for child in node:
                    walk(child)
            elif isinstance(node, dict):
                if node.get('idVendor') == 0x8086 and 0x0a00 <= int(node.get('idProduct') or 0) <= 0x0bff:
                    found.add((node.get('locationID'), node.get('idProduct')))
                walk(node.get('IORegistryEntryChildren', []))
        walk(data)
        return found
    for dev in Path('/sys/bus/usb/devices').glob('*'):
        try:
            if (dev / 'idVendor').read_text().strip() == '8086' and (dev / 'idProduct').read_text().strip().startswith(('0a', '0b')):
                found.add((dev.name, (dev / 'idProduct').read_text().strip()))
        except OSError:
            pass
    return found


def needs_admin(platform=None):
    return (platform or sys.platform) == 'darwin' and os.geteuid() != 0 and not os.environ.get('ROBORUN_CAMERAS_NO_SUDO')


def helper_socket(broker_socket):
    return str(Path(broker_socket).with_name('camera-capture.sock'))


def sdk_path():
    spec = importlib.util.find_spec('pyrealsense2')
    if spec is None or not spec.origin:
        return None
    origin = Path(spec.origin).resolve()
    return str(origin.parent.parent if origin.name.startswith('__init__') else origin.parent)


_helper_lock = threading.Lock()


def ensure_helper(path, serials, *, interactive=False, sdk=None, platform=None):
    """A capture helper for these serials is listening at `path`, started if needed. Only an interactive caller (roborun up,
    --list) may start one through sudo; the background broker reports that it is missing instead."""
    with _helper_lock:
        state = helper.call(path, dict(op='ping'))
        if state and state.get('version') == helper.version() and set(serials) <= set(state.get('serials') or ()):
            return state
        admin = needs_admin(platform)
        if admin and not interactive:
            raise CameraError('needs_admin', 'the camera capture helper is not running: run roborun up from a terminal and '
                                             'accept its sudo prompt')
        sdk = sdk or sdk_path()
        if not sdk:
            raise CameraError('sdk', f'pyrealsense2 is not installed: {INSTALL}')
        if state:
            helper.call(path, dict(op='shutdown'))
            for _ in range(50):
                if not helper.call(path, dict(op='ping')):
                    break
                time.sleep(.1)
        Path(path).parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        argv = [sys.executable, '-I', os.path.abspath(helper.__file__), '--serve', '--daemon', '--socket', path, '--sdk-path', sdk]
        for serial in serials:
            argv += ['--serial', serial]
        if admin:
            if not (sys.stdin and sys.stdin.isatty()):
                raise CameraError('needs_admin', 'the RealSense cameras need one sudo prompt on macOS: run roborun up '
                                                 'from a terminal')
            argv = [SUDO, '-p', SUDO_PROMPT, *argv]
        try:
            run = subprocess.run(argv, stdout=subprocess.PIPE, timeout=180 if admin else 20, text=True)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise CameraError('needs_admin' if admin else 'failed', f'the camera capture helper did not start: {exc}') from exc
        lines = (run.stdout or '').strip().splitlines()
        try:
            answer = json.loads(lines[-1]) if lines else {}
        except ValueError:
            answer = {}
        if run.returncode or not answer.get('ready'):
            why = answer.get('error') or ('sudo was declined' if admin else f'exit code {run.returncode}')
            raise CameraError('needs_admin' if admin else 'failed', f'the camera capture helper did not start: {why}')
        state = helper.call(path, dict(op='ping'), timeout=5.)
        if not state:
            raise CameraError('failed', 'the camera capture helper started but does not answer')
        return state


class RealSenseSource(Source):
    """One RealSense camera, read from its worker process in the capture helper."""
    open_lock = threading.Lock()              # one camera opens at a time: USB settles before the next
    settle_s = 1.

    exposure_control = True

    def __init__(self, serial, helper_path, serials, *, platform=None, bus=None, sdk=None, settings=None):
        self.settings = settings or {}
        self.serial, self.helper_path, self.serials = serial, helper_path, list(serials)
        self.platform, self.sdk = platform or sys.platform, sdk
        self.bus = bus or (lambda: realsense_bus(self.platform))
        self.conn = None

    source = property(lambda self: f'realsense:{self.serial}')

    def open(self):
        with self.open_lock:
            ensure_helper(self.helper_path, self.serials, sdk=self.sdk, platform=self.platform)
            conn = socket.socket(socket.AF_UNIX)
            try:
                conn.settimeout(helper.FIRST_FRAME_S + 10); conn.connect(self.helper_path)
                helper.send_msg(conn, dict(op='open', serial=self.serial, settings={
                    k: self.settings.get(k) for k in ('width', 'height', 'fps')}))
                head = self._reply(conn)
            except BaseException:
                conn.close(); raise
            time.sleep(self.settle_s)
        self.conn = conn
        conn.settimeout(helper.STALL_S + helper.FIRST_FRAME_S)
        return dict(serial=self.serial, name=head.get('name'), usb_type=head.get('usb_type'), firmware=head.get('firmware'),
                    intrinsics=head['intrinsics'], depth_scale_m=head['depth_scale_m'], depth_available=True,
                    depth_aligned_to='color', profiles=head['profiles'], warning=head.get('warning'))

    def _reply(self, conn):
        try:
            head, blob = helper.recv_msg(conn)
        except (OSError, ValueError) as exc:
            raise CameraError('stalled', f'the capture of RealSense {self.serial} ended ({exc})') from exc
        if head.get('event') == 'error':
            raise CameraError(head.get('kind') or 'failed', head.get('message') or 'capture failed')
        self.blob = blob
        return head

    def read(self):
        head = self._reply(self.conn)
        w, h = int(head['width']), int(head['height'])
        if len(self.blob) != w * h * 5:
            raise CameraError('failed', 'the capture helper sent a frame of the wrong size')
        rgb = np.frombuffer(self.blob, np.uint8, w * h * 3).reshape(h, w, 3)
        depth = np.frombuffer(self.blob, np.uint16, w * h, w * h * 3).reshape(h, w)
        return dict(rgb=rgb, depth_z16=depth, acquired_at_s=head['acquired_at_s'], depth_acquired_at_s=head['depth_acquired_at_s'],
                    timestamp_domain=head['timestamp_domain'], sdk_received_monotonic_s=head['sdk_received_monotonic_s'],
                    sensor_frame=head['sensor_frame'], sensor_depth_frame=head['sensor_depth_frame'])

    def set_exposure_us(self, us):
        helper.send_msg(self.conn, dict(op='exposure', us=float(us)))

    def close(self):
        if self.conn is not None:
            self.conn.close(); self.conn = None

    def snapshot(self):
        return self.bus()

    def may_retry(self, error, seen):
        """After a failure on macOS a camera is reopened only when a RealSense device has newly appeared on USB: a new SDK
        context beside streaming cameras, or a retry against a wedged one, can take the whole USB stack down."""
        if self.platform != 'darwin' or error.kind in ('needs_admin', 'sdk'):
            return True
        now = self.bus()
        appeared = now - seen
        seen.intersection_update(now)                         # unplugged: its return on the same port counts as new
        return bool(appeared)

    def retry_hint(self, error):
        if self.platform == 'darwin' and error.kind not in ('needs_admin', 'sdk'):
            return 'unplug and replug this camera to reconnect it'
        return 'retrying'


def _avfoundation():
    try:
        import AVFoundation
    except ImportError as exc:
        raise CameraError('sdk', f'the macOS camera bindings are not installed: {INSTALL}') from exc
    return AVFoundation


def mac_cameras(av=None):
    """[(OpenCV index, unique ID, name)]: OpenCV indexes video + muxed devices sorted by unique ID."""
    av = av or _avfoundation()
    devices = list(av.AVCaptureDevice.devicesWithMediaType_(av.AVMediaTypeVideo)) \
        + list(av.AVCaptureDevice.devicesWithMediaType_(av.AVMediaTypeMuxed))
    devices.sort(key=lambda d: str(d.uniqueID()))
    return [(i, str(d.uniqueID()), str(d.localizedName())) for i, d in enumerate(devices)]


def mac_camera_access(av=None, ask=False):
    """'authorized', 'denied' or 'undetermined'. ask=True (a foreground terminal) shows the macOS prompt when undecided."""
    av = av or _avfoundation()
    status = int(av.AVCaptureDevice.authorizationStatusForMediaType_(av.AVMediaTypeVideo))
    if status == 0 and ask:
        done = threading.Event(); answer = []
        av.AVCaptureDevice.requestAccessForMediaType_completionHandler_(
            av.AVMediaTypeVideo, lambda granted: (answer.append(bool(granted)), done.set()))
        done.wait(120)
        return 'authorized' if answer and answer[0] else 'denied' if answer else 'undetermined'
    return {0: 'undetermined', 3: 'authorized'}.get(status, 'denied')


def linux_device(ident):
    for path in (ident, f'/dev/v4l/by-id/{ident}', f'/dev/v4l/by-path/{ident}'):
        if path.startswith('/dev/') and os.path.exists(path):
            return path
    if ident.isdigit() and os.path.exists(f'/dev/video{ident}'):
        return f'/dev/video{ident}'
    return None


def link_speed_mbps(device):
    try:
        node = os.path.basename(os.path.realpath(device))
        path = Path(os.path.realpath(f'/sys/class/video4linux/{node}/device'))
        for parent in (path, *path.parents):
            if (parent / 'speed').exists():
                return float((parent / 'speed').read_text())
    except (OSError, ValueError):
        pass
    return None


class UsbSource(Source):
    """A plain USB camera through OpenCV (AVFoundation on macOS, V4L2 on Linux): RGB only, no calibration."""
    def __init__(self, ident, *, platform=None, opener=None, access=None, settings=None):
        self.settings = settings or {}
        self.exposure_control = (platform or sys.platform) != 'darwin'    # AVFoundation offers no exposure through OpenCV
        self.ident, self.platform, self.opener = ident, platform or sys.platform, opener or self._open
        self.access = access or mac_camera_access
        self.cap = None; self.seq = 0

    source = property(lambda self: f'usb:{self.ident}')

    def _open(self):
        if self.platform == 'darwin':
            os.environ['OPENCV_AVFOUNDATION_SKIP_AUTH'] = '1'    # it cannot ask from a thread; roborun up already asked
        try:
            import cv2
        except ImportError as exc:
            raise CameraError('sdk', f'OpenCV is not installed: {INSTALL}') from exc
        speed = None
        if self.platform == 'darwin':
            if self.access() != 'authorized':
                raise CameraError('permission', MAC_CAMERA_DENIED)
            index = int(self.ident) if self.ident.isdigit() else \
                next((i for i, uid, _ in mac_cameras() if uid == self.ident), None)
            if index is None:
                raise CameraError('absent', f'USB camera {self.ident} is not connected')
            cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
        else:
            device = linux_device(self.ident)
            if device is None:
                raise CameraError('absent', f'USB camera {self.ident} is not connected')
            if not os.access(device, os.R_OK | os.W_OK):
                raise CameraError('permission', f'no permission for {device}: add this user to the video group '
                                                '(sudo usermod -aG video $USER) and log in again')
            speed = link_speed_mbps(device)
            os.environ.setdefault('OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT', '2')    # seconds a dead camera may block a read
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            raise CameraError('failed', f'could not open USB camera {self.ident}: it may be in use by another program')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.settings.get('width', 640))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.settings.get('height', 480))
        cap.set(cv2.CAP_PROP_FPS, self.settings.get('fps', 30))
        self.speed = speed
        return cap

    speed = None

    def open(self):
        self.cap = self.opener()
        ok, image = self.cap.read()
        if not ok or image is None:
            usb2 = self.speed is not None and self.speed <= 480
            self.close()
            if usb2:
                raise CameraError('usb2', f'USB camera {self.ident} is on a {self.speed:g} Mb/s link that cannot carry this '
                                          'video mode: connect it directly to a USB 3 port, away from other cameras on the hub')
            raise CameraError('failed', f'USB camera {self.ident} opened but delivered no image')
        h, w = self._size(image)
        warnings = [] if not self.speed or self.speed > 480 else [f'{self.speed:g} Mb/s USB link']
        asked = self.settings.get('width'), self.settings.get('height')
        if asked[0] and asked != (image.shape[1], image.shape[0]):
            warnings.append(f'asked for {asked[0]}x{asked[1]}, the camera delivers {image.shape[1]}x{image.shape[0]}')
        return _rgb_only_model(self.ident, w, h, '; '.join(warnings) or None)

    @staticmethod
    def _size(image):
        h, w = image.shape[:2]; k = min(1., MAX_WIDTH / w, MAX_HEIGHT / h)
        return int(h * k) & ~1, int(w * k) & ~1

    def read(self):
        import cv2
        ok, image = self.cap.read()
        now = time.monotonic()
        if not ok or image is None:
            return None
        h, w = self._size(image)
        if image.shape[:2] != (h, w):
            image = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
        self.seq += 1
        return dict(rgb=cv2.cvtColor(image, cv2.COLOR_BGR2RGB), acquired_at_s=now, sdk_received_monotonic_s=now,
                    timestamp_domain='host_arrival', sensor_frame=self.seq)

    def set_exposure_us(self, us):
        import cv2
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)           # V4L2: manual
        if not self.cap.set(cv2.CAP_PROP_EXPOSURE, float(us) / 100):      # V4L2 counts in 100 us steps
            raise CameraError('exposure', 'this camera refused a manual exposure')

    def close(self):
        if self.cap is not None:
            self.cap.release(); self.cap = None


def _rgb_only_model(serial, w, h, warning=None):
    return dict(serial=serial, name=None, intrinsics=dict(width=w, height=h, fx=None, fy=None, ppx=None, ppy=None,
                                                          distortion_model=None, coeffs=None),
                depth_scale_m=None, depth_available=False, depth_aligned_to=None,
                profiles=dict(color=dict(width=w, height=h)), warning=warning)


class StreamSource(Source):
    """A network camera, or a video file played in a loop at its own rate, through OpenCV. RGB only, no calibration."""

    def __init__(self, address, *, opener=None, settings=None):
        self.address, self.settings = address, settings or {}
        self.opener = opener or self._open
        self.cap = None; self.seq = 0; self.pending = None; self.next_at = 0.; self.period = 0.
        scheme = urllib.parse.urlsplit(address).scheme.lower()
        self.is_file = scheme in ('', 'file') or len(scheme) == 1
        self.path = urllib.parse.unquote(urllib.parse.urlsplit(address).path) if scheme == 'file' else address

    source = property(lambda self: f'stream:{redact(self.address)}')

    def _open(self):
        try:
            import cv2
        except ImportError as exc:
            raise CameraError('sdk', f'OpenCV is not installed: {INSTALL}') from exc
        if self.is_file and not os.path.exists(self.path):
            raise CameraError('absent', f'no video file at {self.path}')
        options = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, STREAM_OPEN_MS, cv2.CAP_PROP_READ_TIMEOUT_MSEC, STREAM_READ_MS]
        cap = cv2.VideoCapture(self.path, cv2.CAP_FFMPEG, options)
        if not cap.isOpened():
            cap.release()
            raise CameraError('failed', f'could not open {redact(self.address)}: check the address, the network and the password')
        return cap

    def _resized(self, image):
        import cv2
        want = self.settings.get('width'), self.settings.get('height')
        if want[0] and (image.shape[1], image.shape[0]) != want:
            return cv2.resize(image, want, interpolation=cv2.INTER_AREA)
        h, w = UsbSource._size(image)
        return image if image.shape[:2] == (h, w) else cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)

    def open(self):
        import cv2
        self.cap = self.opener()
        ok, image = self.cap.read()
        if not ok or image is None:
            self.close()
            raise CameraError('failed', f'{redact(self.address)} opened but delivered no image')
        self.pending = self._resized(image)
        rate = self.settings.get('fps') or (self.cap.get(cv2.CAP_PROP_FPS) if self.is_file else 0) or (30 if self.is_file else 0)
        self.period = 1 / rate if rate else 0.
        h, w = self.pending.shape[:2]
        return _rgb_only_model(redact(self.address), w, h)

    def read(self):
        import cv2
        image, self.pending = self.pending, None
        if image is None:
            ok, raw = self.cap.read()
            if (not ok or raw is None) and self.is_file:      # the end of a file: round again
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, raw = self.cap.read()
            if not ok or raw is None:
                return None
            image = raw
        now = time.monotonic()
        if self.period:
            if self.is_file:
                time.sleep(max(0., self.next_at - now)); now = time.monotonic()
            elif now < self.next_at:
                return None                                   # a faster stream than asked for: this frame is dropped
            self.next_at = max(self.next_at + self.period, now)
        self.seq += 1
        return dict(rgb=cv2.cvtColor(self._resized(image), cv2.COLOR_BGR2RGB), acquired_at_s=now, sdk_received_monotonic_s=now,
                    timestamp_domain='host_arrival', sensor_frame=self.seq)

    def close(self):
        if self.cap is not None:
            self.cap.release(); self.cap = None


def load_camera_class(ref):
    where, _, name = ref.rpartition(':')
    try:
        if where.endswith('.py'):
            candidates = [Path(where).expanduser(), net.roborun_home() / where]
            path = next((c for c in candidates if c.is_file()), None)
            if path is None:
                raise ValueError(f'no camera file at {where}')
            spec = importlib.util.spec_from_file_location(f'roborun_own_camera_{path.stem}', path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
        else:
            module = importlib.import_module(where)
    except ValueError:
        raise
    except Exception as exc:
        raise ValueError(f'camera class {ref}: could not load it ({type(exc).__name__}: {exc})') from exc
    kind = getattr(module, name, None)
    if not inspect.isclass(kind):
        raise ValueError(f'camera class {ref}: {where} has no class {name}')
    if not callable(getattr(kind, 'read', None)):
        raise ValueError(f'camera class {ref}: it needs a read() method returning an RGB image')
    return kind


class CustomSource(Source):
    """Your own camera class. It is constructed with whichever of role, width, height, fps its __init__ names, plus every
    other key of the camera's section in robot.yaml.

        open()   optional. May return {serial, intrinsics, depth_scale_m, warning, rectified}; rectified: true says the
                 frames are already undistorted (a pinhole), as does intrinsics with distortion_model 'none'
        read()   an RGB uint8 image, or {rgb, depth_z16, acquired_at_s}, or None when no new frame is ready
        close()  optional
        set_exposure_us(us)  optional: enables exposure management
    """

    def __init__(self, ref, role, *, settings=None, extra=None):
        self.ref, self.settings, self.seq = ref, settings or {}, 0
        kind = load_camera_class(ref)
        known = dict(role=role, **{k: self.settings[k] for k in ('width', 'height', 'fps') if k in self.settings})
        accepted = inspect.signature(kind).parameters
        open_kw = any(p.kind == p.VAR_KEYWORD for p in accepted.values())
        kw = {k: v for k, v in known.items() if open_kw or k in accepted}
        unknown = sorted(set(extra or ()) - set(accepted)) if not open_kw else []
        if unknown:
            raise ValueError(f'camera class {ref}: its __init__ does not take {", ".join(unknown)}')
        kw.update(extra or {})
        try:
            self.camera = kind(**kw)
        except TypeError as exc:
            raise ValueError(f'camera class {ref}: {exc}') from exc
        self.exposure_control = callable(getattr(self.camera, 'set_exposure_us', None))
        self.pending = None

    source = property(lambda self: self.ref)

    def open(self):
        opener = getattr(self.camera, 'open', None)
        info = (opener() if callable(opener) else None) or {}
        deadline = time.monotonic() + helper.FIRST_FRAME_S
        item = None
        while item is None:
            item = self.read()
            if item is None:
                if time.monotonic() > deadline:
                    raise CameraError('failed', f'{self.ref} delivered no image')
                time.sleep(.01)
        self.pending = item
        h, w = item['rgb'].shape[:2]
        model = _rgb_only_model(str(info.get('serial') or self.ref), w, h, info.get('warning'))
        if info.get('intrinsics'):
            model['intrinsics'] = dict(info['intrinsics'], width=w, height=h)
        if isinstance(info.get('rectified'), bool):
            model['rectified'] = info['rectified']
        if 'depth_z16' in item:
            if not info.get('depth_scale_m'):
                raise CameraError('failed', f'{self.ref} returns depth, so open() must return depth_scale_m')
            model.update(depth_scale_m=float(info['depth_scale_m']), depth_available=True, depth_aligned_to='color')
        return model

    def read(self):
        if self.pending is not None:
            item, self.pending = self.pending, None
            return item
        got = self.camera.read()
        if got is None:
            return None
        item = dict(got) if isinstance(got, dict) else dict(rgb=got)
        now = time.monotonic()
        self.seq += 1
        item.setdefault('acquired_at_s', now)
        item.setdefault('depth_acquired_at_s', item['acquired_at_s'])
        item.setdefault('sdk_received_monotonic_s', now)
        item.setdefault('timestamp_domain', 'host_arrival')
        item.setdefault('sensor_frame', self.seq)
        return item

    def set_exposure_us(self, us):
        self.camera.set_exposure_us(float(us))

    def close(self):
        closer = getattr(self.camera, 'close', None)
        if callable(closer):
            closer()


# ---- the broker
class Broker:
    def __init__(self, sources, *, backoff=(1., 30.), stall_s=STALL_S, fresh_s=FRESH_S, busy=None):
        self.sources = dict(sources)
        self.busy = busy
        self.exposure = {}
        self.backoff, self.stall_s, self.fresh_s = backoff, stall_s, fresh_s
        self.session = uuid.uuid4().hex
        self.frames, self.models, self.errors, self.ids, self.reconnects = {}, {}, {}, {}, {}
        self.packets, self.opened_at, self.failed_once, self.rates = {}, {}, set(), {}
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.threads = []

    def start(self):
        for role in self.sources:
            t = threading.Thread(target=self._capture, args=(role,), daemon=True, name=f'camera-{role}')
            self.threads.append(t); t.start()

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        for t in self.threads:
            t.join(3)

    def _log(self, **event):
        print(json.dumps(dict(event='camera', **event)), flush=True)

    def _capture(self, role):
        source = self.sources[role]; delay = self.backoff[0]
        while not self.stop_event.is_set():
            produced = False
            try:
                self._opened(role, source.open())
                last = time.monotonic()
                while not self.stop_event.is_set():
                    item = source.read()
                    if item is None:
                        if time.monotonic() - last > self.stall_s:
                            raise CameraError('stalled', f'no frames for {self.stall_s:g} s')
                        time.sleep(.005)
                        continue
                    self._publish(role, item); produced = True; last = time.monotonic()
            except CameraError as exc:
                error = exc
            except Exception as exc:                          # one camera's surprise never reaches the others
                error = CameraError('failed', f'{type(exc).__name__}: {exc}')
            else:
                break
            finally:
                try:
                    source.close()
                except Exception:
                    pass
            seen = source.snapshot()
            hint = source.retry_hint(error)
            with self.condition:
                self.errors[role] = dict(kind=error.kind, message=f'{error} ({hint})')
                self.failed_once.add(role); self.condition.notify_all()
            self._log(role=role, state='failed', kind=error.kind, message=str(error)[:300])
            delay = self.backoff[0] if produced else min(delay * 2, self.backoff[1])
            if self.stop_event.wait(delay):
                break
            while not source.may_retry(error, seen):
                if self.stop_event.wait(1.):
                    return

    def _opened(self, role, model):
        source = self.sources[role]
        settings = source.settings or {}
        rotate = settings.get('rotate', 0)
        model = dict(model, source=source.source, reference_stream='color', rotate=rotate,
                     intrinsics=rotate_intrinsics(model['intrinsics'], rotate),
                     profiles=rotate_profiles(model.get('profiles'), rotate))
        model['warning'] = self._start_exposure(role, source, settings, model.get('warning'))
        with self.condition:
            before = self.models.get(role)
            same = before is not None and all(before.get(k) == model.get(k) for k in ('serial', 'intrinsics', 'depth_scale_m', 'profiles'))
            if before is not None and not same:               # readers tie calibration and identity to the session
                self.session = uuid.uuid4().hex
            self.models[role] = model
        self._log(role=role, state='open', source=model['source'], depth=model['depth_available'], warning=model.get('warning'))

    def _start_exposure(self, role, source, settings, warning):
        wanted = settings.get('exposure') or {}
        self.exposure.pop(role, None)
        if not wanted.get('managed'):
            return warning
        if not source.exposure_control:
            note = 'this source has no exposure control: its exposure setting is ignored'
            return f'{warning}; {note}' if warning else note
        control = ExposureController(wanted, source.set_exposure_us, self.busy)
        control.start()
        self.exposure[role] = control
        return warning

    def _publish(self, role, item):
        now = time.monotonic()
        model = self.models[role]
        rotate = model.get('rotate', 0)
        rgb = rotate_image(item['rgb'], rotate); h, w = rgb.shape[:2]
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise CameraError('failed', f'unusable image {rgb.shape}')
        if not (0 < w <= MAX_WIDTH and 0 < h <= MAX_HEIGHT):
            raise CameraError('mode', f'the image is {w}x{h} after rotation; the most the robot accepts is {MAX_WIDTH}x{MAX_HEIGHT}: '
                                      'set a smaller width and height for this camera')
        if (model['intrinsics']['width'], model['intrinsics']['height']) != (w, h):
            raise CameraError('failed', 'the image size changed while streaming')
        has_depth = bool(model['depth_available'])
        depth = rotate_image(item['depth_z16'], rotate) if has_depth else np.zeros((h, w), np.uint16)
        control = self.exposure.get(role)
        if control is not None:
            control.update(rgb)
        with self.condition:
            fid = self.ids[role] = self.ids.get(role, 0) + 1  # never restarts, so after_id survives a reconnect
            at = min(float(item['acquired_at_s']), now)
            frame = dict(rgb=rgb, depth_z16=depth, frame_id=fid, depth_frame_id=fid if has_depth else None,
                         acquired_at_s=at, depth_acquired_at_s=min(float(item['depth_acquired_at_s']), now) if has_depth else None,
                         sdk_received_monotonic_s=item.get('sdk_received_monotonic_s', now), received_monotonic_s=now,
                         received_at=time.time(), camera_session_id=self.session, role=role, source=model['source'],
                         camera_id=(self.sources[role].settings or {}).get('id', role), rotate=rotate,
                         serial=model['serial'], intrinsics=model['intrinsics'], profiles=model['profiles'],
                         depth_available=has_depth, depth_scale_m=model['depth_scale_m'],
                         depth_aligned_to=model['depth_aligned_to'], reference_stream='color',
                         timestamp_domain=item['timestamp_domain'], acquisition_timestamp_verified=True,
                         sensor_frame=item.get('sensor_frame'), sensor_depth_frame=item.get('sensor_depth_frame'),
                         exposure_us=round(control.us) if control is not None else None,
                         exposure_held=control.held if control is not None else None)
            previous = self.frames.get(role)
            if previous is not None and now > previous['received_monotonic_s']:
                rate = 1 / (now - previous['received_monotonic_s'])
                self.rates[role] = .9 * self.rates.get(role, rate) + .1 * rate
            self.frames[role] = frame; self.packets.pop(role, None)
            recovered = self.errors.pop(role, None) is not None
            if recovered:
                self.reconnects[role] = self.reconnects.get(role, 0) + 1
            self.condition.notify_all()
        if recovered:
            self._log(role=role, state='streaming', reconnects=self.reconnects[role])

    def status(self):
        with self.condition:
            now = time.monotonic()
            ages = {r: now - f['acquired_at_s'] for r, f in self.frames.items()}
            errors = {r: e['message'] for r, e in self.errors.items()}
            for role, age in ages.items():
                if role not in errors and age > self.stall_s:
                    errors[role] = f'no new frame for {age:.0f} s'
            cameras = {}
            live = 0
            for role, source in self.sources.items():
                model = self.models.get(role) or {}
                fresh = role not in errors and ages.get(role, 1e9) <= self.fresh_s
                live += fresh
                cameras[role] = dict(state='streaming' if fresh else 'failed' if role in errors else 'connecting',
                                     source=source.source, depth=model.get('depth_available'), error=errors.get(role),
                                     error_kind=(self.errors.get(role) or {}).get('kind'), warning=model.get('warning'),
                                     fps=round(self.rates[role], 1) if role in self.rates else None,
                                     reconnects=self.reconnects.get(role, 0),
                                     exposure=self.exposure[role].status() if role in self.exposure else None)
            total = len(self.sources)
            state = 'streaming' if live == total else 'degraded' if live and errors else 'failed' if errors else 'warming'
            # the task cameras (top, left, right) are what the runtime iterates; a camera of your own name with no alias is
            # served under its name for viewing and recording, listed in extra_roles only
            result = dict(state=state, camera_session_id=self.session, motor_access=False, calibrated=False,
                          models={r: self.models[r] for r in self.models}, frame_age_s=ages, errors=errors, cameras=cameras,
                          available_roles=[r for r in self.sources if r in ROLES],
                          unavailable_roles=[r for r in ROLES if r not in self.sources],
                          complete_observation=all(r in self.sources for r in ROLES),
                          roles_without_depth=[r for r, m in self.models.items() if r in ROLES and not m['depth_available']],
                          extra_roles=[r for r in self.sources if r not in ROLES])
        return result

    def frame(self, role, after):
        if not isinstance(role, str) or (role not in ROLES and role not in self.sources) or \
                (after is not None and (isinstance(after, bool) or not isinstance(after, int) or after < 0)):
            raise ValueError('Invalid frame selector')
        if role not in self.sources:
            raise ValueError(f'No fresh acquired frame: no {role} camera is configured on this robot')
        with self.condition:
            ready = lambda: role in self.errors or (role in self.frames and (after is None or self.frames[role]['frame_id'] > after))
            self.condition.wait_for(lambda: ready() or self.stop_event.is_set(), timeout=.3)
            if role in self.errors:
                raise ValueError(f"No fresh acquired frame: the {role} camera failed: {self.errors[role]['message']}")
            frame = self.frames.get(role)
            if frame is None or not 0 <= time.monotonic() - frame['acquired_at_s'] <= self.fresh_s:
                raise ValueError('No fresh acquired frame')
            if after is not None and frame['frame_id'] <= after:
                raise ValueError('No new frame')
            cached = self.packets.get(role)
            if cached and cached[0] == frame['frame_id']:
                return cached[1]
        packet = encode_frame(frame)
        with self.condition:
            if self.frames.get(role) is frame:
                self.packets[role] = (frame['frame_id'], packet)
        return packet

    def dispatch(self, body):
        if not isinstance(body, dict):
            raise ValueError('Invalid camera request')
        op = body.get('operation')
        if op in ('status', 'stop', 'restart'):               # the broker's lifetime belongs to its service, not to clients
            return json.dumps(self.status(), allow_nan=False).encode()
        if op != 'frame':
            raise ValueError(f'Unknown camera operation {op!r}')
        return self.frame(body.get('role'), body.get('after_id'))


class _Handler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(2)
        try:
            payload = self.server.broker.dispatch(json.loads(receive(self.request)))
        except Exception as exc:
            payload = json.dumps(dict(error=str(exc))).encode()
        try:
            send(self.request, payload)
        except OSError:
            pass


class _Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    request_queue_size = 32


def listen(broker, path):
    path = Path(path)
    if len(os.fsencode(path)) > 100:
        raise SystemExit(f'camera socket path too long for a Unix socket: {path}')
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if path.exists():
        with socket.socket(socket.AF_UNIX) as probe:
            probe.settimeout(1)
            if probe.connect_ex(str(path)) == 0:
                raise SystemExit(f'another camera service is already running on {path}')
        path.unlink()
    previous = os.umask(0o177)
    try:
        server = _Server(str(path), _Handler)
    finally:
        os.umask(previous)
    server.broker = broker
    threading.Thread(target=server.serve_forever, kwargs=dict(poll_interval=.1), daemon=True, name='camera-socket').start()
    return server


def build_sources(hardware, broker_socket, platform=None):
    configs = camera_configs(hardware)
    serials = [ident for kind, ident, _, _ in configs.values() if kind == 'realsense']
    path = helper_socket(broker_socket)
    out = {}
    for role, (kind, ident, settings, extra) in configs.items():
        if kind == 'realsense':
            out[role] = RealSenseSource(ident, path, serials, platform=platform, settings=settings)
        elif kind == 'usb':
            out[role] = UsbSource(ident, platform=platform, settings=settings)
        elif kind == 'stream':
            out[role] = StreamSource(ident, settings=settings)
        else:
            out[role] = CustomSource(ident, role, settings=settings, extra={k: v for k, v in extra.items() if '.' not in k})
    return out


class TaskWatcher:
    """Is a task running on this robot? Asked of the local backend once a second; exposure holds still while it is."""

    def __init__(self, check, every_s=1.):
        self.check, self.every_s, self.value = check, every_s, False
        self.stop = threading.Event()
        threading.Thread(target=self._run, daemon=True, name='camera-task-watch').start()

    def _run(self):
        while not self.stop.wait(self.every_s):
            self.value = bool(self.check())

    def __call__(self):
        return self.value


# ---- roborun up: everything that needs the operator's terminal happens here, before the background services start
NO_DEPTH = 'Tasks need a depth camera (e.g. RealSense). Your webcam will show and record.'   # the runtime's own words (OAK is not read yet)
SETUP_SAYS_NO_DEPTH = False        # the guided setup says NO_DEPTH once, at its end


def webcams_only(sources):
    """True when there are cameras and every one is a plain webcam or stream (RGB only, known for sure). A RealSense gives
    depth; your own class (my_cam.py:MyCamera) may too, so it is never told it has none."""
    sources = [s.get('source') if isinstance(s, dict) else s for s in sources]
    try:
        return bool(sources) and all(parse_source(s)[0] in ('usb', 'stream') for s in sources)
    except ValueError:
        return False


def prepare_up(hardware, broker_socket, *, platform=None, say=print):
    platform = platform or sys.platform
    try:
        configs = camera_configs(hardware)
        blur = face_blur_enabled(hardware)
    except ValueError as exc:
        raise SystemExit(str(exc))
    if not configs:
        return
    parsed = {role: row[:2] for role, row in configs.items()}
    for warning in setting_warnings(configs):
        say(f'cameras: {warning}')
    if blur:
        try:
            privacy.FaceBlur()
        except privacy.BlurUnavailable as exc:
            say(f'cameras: face blur cannot run ({exc}), so no camera image can leave this computer and tasks will stop before '
                'moving. The cameras and Stop still work. Reinstall with curl -fsSL https://roborun.app/install.sh | sh')
    else:
        say('cameras: FACE BLUR IS OFF (privacy: face_blur: false in robot.yaml): camera frames go to the motion service '
            'unblurred, and history images stay on this computer')
    kinds = {kind for kind, _ in parsed.values()}
    missing = []
    if 'realsense' in kinds and not sdk_path():
        missing.append('pyrealsense2')
    if kinds & {'usb', 'stream'} and importlib.util.find_spec('cv2') is None:
        missing.append('OpenCV')
    if 'usb' in kinds and platform == 'darwin' and importlib.util.find_spec('AVFoundation') is None:
        missing.append('the macOS camera bindings')
    if missing:
        raise SystemExit(f"cameras need {', '.join(missing)}: {INSTALL}"
                         + ('  (RealSense wheels for macOS are built for Apple Silicon on macOS 15 or newer)' if platform == 'darwin' else ''))
    if parsed and kinds <= {'usb', 'stream'} and not SETUP_SAYS_NO_DEPTH:
        say(NO_DEPTH)
    if platform == 'linux' and 'realsense' in kinds \
            and not any((Path(d) / UDEV_RULES).exists() for d in ('/etc/udev/rules.d', '/lib/udev/rules.d', '/usr/lib/udev/rules.d')):
        say(f'cameras: the RealSense udev rules are not installed, so USB access will fail. Run: {UDEV_HELP}')
    if platform != 'darwin':
        return
    if 'usb' in kinds:
        try:
            if mac_camera_access(ask=sys.stdin.isatty()) != 'authorized':
                say('cameras: ' + MAC_CAMERA_DENIED)
        except CameraError as exc:
            say(f'cameras: {exc}')
    serials = [ident for kind, ident in parsed.values() if kind == 'realsense']
    if serials and needs_admin(platform):
        try:
            state = ensure_helper(helper_socket(broker_socket), serials, interactive=True, platform=platform)
            say(f"cameras: capture helper running{' as root' if state.get('root') else ''}, for RealSense {', '.join(serials)}")
        except CameraError as exc:
            say(f'cameras: {exc}. The arms will start; these cameras stay offline until this is fixed.')


def _prepare_from_args(a):
    from .arms.hardware import config_from_args
    from .robot import CONFIG, camera_socket
    stored = json.loads(CONFIG.read_text()).get('hardware') if CONFIG.exists() else None
    hardware = config_from_args(a, stored)
    if hardware and not (getattr(a, 'sim', None) and not a.camera):
        prepare_up(hardware, camera_socket())


def _protect_config():
    """A camera address can hold a password: the files `roborun up` stores it in are readable by this user only."""
    from .robot import CONFIG, HOME
    for path in (Path(CONFIG), Path(HOME) / 'robot' / 'hardware.json'):
        try:
            path.chmod(0o600)
        except OSError:
            pass


# ---- roborun cameras
def list_realsense(broker_socket, platform=None):
    """([(source, description)], [notes]). Serials come from the running service when there is one: opening another SDK
    context beside streaming cameras is what must not happen on macOS."""
    platform = platform or sys.platform
    try:
        with socket.socket(socket.AF_UNIX) as conn:
            conn.settimeout(2); conn.connect(broker_socket); send(conn, json.dumps(dict(operation='status')).encode())
            status = json.loads(receive(conn, MAX_PACKET))
        rows = [(c['source'], f"in use as {role}: {c['state']}") for role, c in status.get('cameras', {}).items()
                if c['source'].startswith('realsense:')]
        return rows, ['the camera service is running: only the cameras it serves are listed']
    except (OSError, ValueError, KeyError, struct.error):
        pass
    sdk = sdk_path()
    on_bus = realsense_bus(platform)
    if not sdk:
        return [], [f'{len(on_bus)} RealSense device(s) on USB, but pyrealsense2 is not installed: {INSTALL}'] if on_bus else []
    argv = [sys.executable, '-I', os.path.abspath(helper.__file__), '--list', '--sdk-path', sdk]
    def ask(cmd):
        try:
            out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=60, text=True).stdout
            return json.loads(out.strip().splitlines()[-1])
        except (OSError, ValueError, IndexError, subprocess.SubprocessError):
            return dict(error='the RealSense SDK did not answer')
    answer = ask(argv)
    cams = [c for c in answer.get('cameras', []) if c.get('serial')]
    notes = []
    if on_bus and not cams and needs_admin(platform):
        if sys.stdin.isatty():
            answer = ask([SUDO, '-p', SUDO_PROMPT, *argv])
            cams = [c for c in answer.get('cameras', []) if c.get('serial')]
        else:
            notes.append(f'{len(on_bus)} RealSense device(s) on USB need administrator access to read: run this in a terminal')
    elif on_bus and not cams and platform == 'linux':
        notes.append(f'{len(on_bus)} RealSense device(s) on USB but none readable: install the udev rules: {UDEV_HELP}')
    if answer.get('error'):
        notes.append(answer['error'])
    return [(f"realsense:{c['serial']}", f"{c.get('name')}, USB {c.get('usb_type')}, RGB + depth") for c in cams], notes


def list_usb(platform=None):
    platform = platform or sys.platform
    if platform == 'darwin':
        try:
            return [(f'usb:{uid}', f'{name}, RGB only') for _, uid, name in mac_cameras() if 'realsense' not in name.lower()], []
        except CameraError as exc:
            return [], [str(exc)]
    rows, named = [], set()
    for folder in ('/dev/v4l/by-id', '/dev/v4l/by-path'):
        for link in sorted(Path(folder).glob('*-video-index0')) if Path(folder).is_dir() else []:
            node = os.path.basename(os.path.realpath(link))
            if node in named:
                continue
            named.add(node)
            try:
                name = Path(f'/sys/class/video4linux/{node}/name').read_text().strip()
            except OSError:
                name = node
            if 'realsense' not in (name + link.name).lower():
                rows.append((f'usb:{link.name}', f'{name}, RGB only'))
    return rows, []


def cmd_list(a):
    from .robot import camera_socket
    realsense, notes = list_realsense(a.socket or camera_socket())
    usb, more = list_usb()
    rows = realsense + usb
    for source, text in rows:
        print(f'{source}\t{text}')
    if not rows:
        print('no cameras found')
    for note in notes + more:
        print(f'note: {note}')
    return 0


def cmd_check_roles(a):
    from .camera_roles import watch
    from .robot import camera_socket
    print(f'Watching for {a.seconds:g} s. Move one arm at a time (by hand or in a task); nothing here moves the robot.', flush=True)
    try:
        answer = watch(a.socket or camera_socket(), a.seconds)
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        raise SystemExit(f'could not read the cameras and arms: {exc}. Next: start the robot with roborun up, with both wrist '
                         'cameras and both arms configured')
    print(f"{answer['verdict']}: {answer['reason']}")
    return 0 if answer['verdict'] == 'ok' else 1


def _socket_path():
    """Where the camera service listens, worked out without creating anything (robot.camera_socket makes its folder)."""
    if os.environ.get('MANIFEST_CAMERA_SOCKET'):
        return os.environ['MANIFEST_CAMERA_SOCKET']
    from .robot import socket_path
    return socket_path('camera.sock')


def broker_status(path, timeout=2.):
    """The camera service's status (its read-only `status` operation), or None when nothing answers."""
    try:
        with socket.socket(socket.AF_UNIX) as conn:
            conn.settimeout(timeout); conn.connect(path); send(conn, json.dumps(dict(operation='status')).encode())
            answer = json.loads(receive(conn, MAX_PACKET))
        return answer if isinstance(answer, dict) and not answer.get('error') else None
    except (OSError, ValueError, struct.error):
        return None


def _design_mount():
    """The YAM's design wrist mount from the installed robot runtime, or None (no runtime here, or it cannot say)."""
    try:
        from .robot import CONFIG
        root = Path(json.loads(Path(CONFIG).read_text()).get('manifest') or '') if Path(CONFIG).exists() else None
        if not root or not (root / 'kin.py').exists():
            return None
        for p in (str(root), str(root / 'workspaces' / 'yam' / 'policies')):
            if p not in sys.path:
                sys.path.append(p)
        import video_calibration
        return [[float(v) for v in row] for row in video_calibration.mount()]
    except Exception:
        return None


def cmd_describe(a):
    """roborun cameras describe: every camera's card (the SDK's layer), as text or --json. Reads robot.yaml, the profile and
    the camera service's status; opens no device, starts nothing, writes nothing."""
    import yaml
    from . import camera_card
    from .arms import settings
    path = Path(a.config).expanduser() if a.config else settings.path()
    try:
        data = (yaml.safe_load(path.read_text()) if path.exists() else None) or {}
    except yaml.YAMLError as exc:
        raise SystemExit(f'{path} is not valid: {exc}')
    if not isinstance(data, dict):
        raise SystemExit(f'{path}: expected settings like "cameras:" at the top level')
    cams = data.get('cameras') or {}
    arms = data.get('arms') if isinstance(data.get('arms'), dict) else {}
    if arms.get('right') and not arms.get('left'):            # a single right arm runs in the left slot, its cameras with it
        cams = settings.one_arm_on_the_left(dict(right=arms['right'], cameras=cams)).get('cameras') or {}
    body = data.get('body') if isinstance(data.get('body'), dict) else {}
    profile = a.profile or data.get('profile') or body.get('profile')
    status = broker_status(a.socket or _socket_path())
    try:
        doc, warnings = camera_card.describe(cams, profile=profile, status=status, cad=_design_mount())
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    if a.json:
        print(json.dumps(doc, indent=1))
        for w in warnings:
            print(f'warning: {w}', file=sys.stderr)
    else:
        print(camera_card.describe_text(doc, warnings))
    return 0


def cmd_cameras(a):
    if getattr(a, 'action', None) == 'describe':
        return cmd_describe(a)                                # read-only: installs nothing, opens nothing
    from .arms.packages import ensure
    ensure('cameras', 'cameras')
    if a.list:
        return cmd_list(a)
    if a.check_roles:
        return cmd_check_roles(a)
    if not (a.hardware and a.socket):
        raise SystemExit('roborun cameras: use describe, --list, --check-roles, or --hardware FILE --socket PATH')
    hardware = json.loads(Path(a.hardware).read_text())
    try:
        sources = build_sources(hardware, a.socket)
        face_blur_enabled(hardware)
        for warning in setting_warnings(camera_configs(hardware)):
            print(json.dumps(dict(event='camera_warning', message=warning)), flush=True)
    except ValueError as exc:
        raise SystemExit(str(exc))
    if not sources:
        raise SystemExit(f'no cameras in {a.hardware}')
    import cv2
    cv2.setNumThreads(1)                  # one core per camera at most: the decoders never crowd the robot
    busy = None
    if any((s.settings.get('exposure') or {}).get('managed') for s in sources.values()):
        from .robot import CONFIG, _busy
        backend = (json.loads(CONFIG.read_text()) if CONFIG.exists() else {}).get('backend') or 'http://127.0.0.1:8120'
        busy = TaskWatcher(lambda: _busy(backend))
    broker = Broker(sources, busy=busy)
    server = listen(broker, a.socket)
    broker.start()
    done = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: done.set())
    print(json.dumps(dict(event='cameras', socket=a.socket, roles={r: s.source for r, s in sources.items()})), flush=True)
    done.wait()
    server.shutdown(); server.server_close(); broker.close()
    try:
        os.unlink(a.socket)
    except OSError:
        pass
    return 0


def add_parser(sub):
    p = sub.add_parser('cameras', help='serve this robot\'s real cameras to Roborun; --list shows the connected ones',
                       description=__doc__, formatter_class=__import__('argparse').RawDescriptionHelpFormatter)
    p.add_argument('action', nargs='?', choices=('describe',),
                   help='describe: print every camera\'s card (where it is, what it delivers); nothing is opened or changed')
    p.add_argument('--json', action='store_true', help='with describe: the cards as JSON (roborun.cameras/1)')
    p.add_argument('--config', help='with describe: this robot.yaml instead of the one in ~/.config/roborun')
    p.add_argument('--profile', help='with describe: this profile instead of robot.yaml\'s')
    p.add_argument('--list', action='store_true', help='print connected cameras and the SOURCE to use for each')
    p.add_argument('--check-roles', action='store_true', help='check, from arm motion, that the wrist cameras are on the right arms')
    p.add_argument('--seconds', type=float, default=30., help='how long --check-roles watches')
    p.add_argument('--hardware', help='the hardware file written by roborun up'); p.add_argument('--socket', help='the broker socket')
    p.set_defaults(func=cmd_cameras)
    up = sub.choices.get('up')
    inner = up.get_default('func') if up else None
    if inner is not None:                 # the sudo and camera-permission prompts need `roborun up`'s own terminal
        def cmd_up_with_cameras(a):
            from .arms.packages import prepare_for_up
            prepare_for_up(a)                 # in the foreground: nothing is left to install in a background service
            _prepare_from_args(a)
            try:
                return inner(a)
            finally:
                _protect_config()
        up.set_defaults(func=cmd_up_with_cameras)
