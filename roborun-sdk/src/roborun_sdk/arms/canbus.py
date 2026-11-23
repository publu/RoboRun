"""CAN buses by address, the same on Linux and macOS.

    can0                  a Linux socketcan interface; brought up at 1 Mbit/s when it is down, or the exact command is named
    slcan://SERIAL        a CANable-style USB serial adapter, by its USB serial number
    slcan://port/1-2.3    the same, by the USB port it is plugged into: for adapters with no serial or a shared one
    slcan:///dev/ttyACM0  the same, by its device path
    gs_usb://SERIAL       a candleLight-style USB adapter, by its USB serial number (libusb, no driver to install)
    gs_usb://port/1-2.3   the same, by USB port
    virtual://NAME        python-can's in-process bus (tests)

`roborun hw detect` prints the address of everything connected. A bus has one Roborun owner at a time: opening one that
another process holds fails at once and names that process.

USB IDs come from public sources: the gs_usb table of the Linux kernel (drivers/net/can/usb/gs_usb.c), the candleLight
firmware, and the CANable firmwares (normaldotcom/canable-fw, normaldotcom/canable2-fw).
"""
import collections
import errno
import getpass
import hashlib
import json
import logging
import os
import re
import shlex
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

BITRATE = 1_000_000
FD_DATA_BITRATE = 5_000_000              # CAN FD data phase, as OpenArm's `-fd` setup runs it
SLCAN_IDS = {(0xad50, 0x60c4): 'CANable (slcan firmware)', (0x16d0, 0x117e): 'CANable 2 (slcan firmware)'}
GS_USB_IDS = {(0x1d50, 0x606f): 'candleLight', (0x1209, 0x2323): 'candleLight', (0x1cd2, 0x606f): 'CANext FD',
              (0x16d0, 0x10b8): 'CANdebugger FD', (0x16d0, 0x0f30): 'SAINT3', (0x1209, 0xca01): 'CANnectivity'}
ST_SERIAL_ID = (0x0483, 0x5740)                           # ST's stock USB serial ID: an adapter only if it says it is CAN
ST_DFU_ID = (0x0483, 0xdf11)                              # any STM32 adapter with its boot jumper set
ARPHRD_CAN = 280
UDEV_FILE = '/etc/udev/rules.d/70-roborun-can.rules'
SYS_NET, PROC = Path('/sys/class/net'), Path('/proc')     # module level so tests can point them at a folder

log = logging.getLogger(__name__)


class AdapterError(RuntimeError):
    """A connection failure in the user's terms: which adapter, what is wrong, what to do next."""

    def __init__(self, address, cause, fix):
        super().__init__(f'{address}: {cause}. Next: {fix}')
        self.address, self.cause, self.fix = address, cause, fix


def udev_rules():
    return [f'SUBSYSTEM=="usb", ATTR{{idVendor}}=="{vid:04x}", ATTR{{idProduct}}=="{pid:04x}", MODE="0660", GROUP="dialout"'
            for vid, pid in sorted(GS_USB_IDS)]


def in_dialout():
    try:
        import grp                      # POSIX only: importing this module works on Windows too
        return grp.getgrnam('dialout').gr_gid in os.getgroups() or getpass.getuser() in grp.getgrnam('dialout').gr_mem
    except (ImportError, KeyError, OSError):
        return False


def usb_permission_fix():
    """The commands that give this user the USB CAN adapters on Linux."""
    lines = ' '.join(shlex.quote(rule) for rule in udev_rules())
    steps = [f"printf '%s\\n' {lines} | sudo tee {UDEV_FILE} >/dev/null", 'sudo udevadm control --reload-rules',
             'sudo udevadm trigger']
    if not in_dialout():
        steps.append('sudo usermod -aG dialout "$USER"')
    return ' && '.join(steps) + ', then replug the adapter' + ('' if in_dialout() else ' and log in again')


def serial_permission_fix(device):
    if sys.platform == 'darwin':
        return f'check that no other program has {device} open, then replug the adapter'
    return f'run: sudo usermod -aG dialout "$USER", then log in again (or for now: sudo chmod a+rw {device})'


# -- USB discovery --------------------------------------------------------------------------------------------------
def _usb_backend():
    """libusb from the libusb-package wheel, so nothing has to be installed system-wide."""
    import libusb_package
    return libusb_package.get_libusb1_backend()


def _usb_all():
    import usb.core
    return list(usb.core.find(find_all=True, backend=_usb_backend()) or ())


def _usb_devices():
    return [d for d in _usb_all() if (d.idVendor, d.idProduct) in GS_USB_IDS]


def _usb_port(dev):
    """'1-2.3': the bus and port chain, which is also the device's name in Linux sysfs."""
    bus, ports = getattr(dev, 'bus', None), getattr(dev, 'port_numbers', None)
    return f"{bus}-{'.'.join(str(p) for p in ports)}" if bus is not None and ports else None


def _usb_text(dev, field):
    """(value, problem): problem is 'permission' when the device cannot be opened to read its strings."""
    import usb.core
    try:
        return getattr(dev, field), None
    except ValueError as exc:                             # pyusb: "The device has no langid (permission issue, ...)"
        return None, 'permission' if 'permission' in str(exc).lower() or 'langid' in str(exc).lower() else None
    except usb.core.USBError as exc:
        return None, 'permission' if exc.errno in (errno.EACCES, errno.EPERM) else None


def _serial_location(port):
    return (getattr(port, 'location', None) or '').split(':')[0] or None


def _is_slcan(port):
    ids = (port.vid, port.pid)
    return ids in SLCAN_IDS or (ids == ST_SERIAL_ID and re.search(r'can(able)?\b', str(port.product or ''), re.I) is not None)


def _serial_adapters():
    from serial.tools import list_ports
    rows = []
    for port in list_ports.comports():
        if _is_slcan(port):
            device = port.device.replace('/dev/tty.', '/dev/cu.') if sys.platform == 'darwin' else port.device
            rows.append(dict(kind='slcan', serial=port.serial_number or None, usb_port=_serial_location(port), port=device,
                             product=port.product or SLCAN_IDS.get((port.vid, port.pid)), problem=None))
    return rows


def _gs_usb_adapters():
    rows = []
    for dev in _usb_devices():
        serial, problem = _usb_text(dev, 'serial_number')
        product, other = _usb_text(dev, 'product')
        rows.append(dict(kind='gs_usb', serial=serial or None, usb_port=_usb_port(dev), dev=dev, problem=problem or other,
                         product=product or GS_USB_IDS.get((dev.idVendor, dev.idProduct))))
    return rows


def _with_addresses(rows):
    """Each adapter's address: its serial when that is unique, otherwise where it is plugged in."""
    count = collections.Counter((r['kind'], r['serial']) for r in rows if r['serial'])
    for row in rows:
        kind = row['kind']
        if row['serial'] and count[kind, row['serial']] == 1:
            row['address'] = f"{kind}://{row['serial']}"
        elif row['usb_port']:
            row['address'] = f"{kind}://port/{row['usb_port']}"
            if row['serial']:
                row['note'] = f"shares serial {row['serial']} with another adapter: this address follows the USB port"
            elif not row['problem']:
                row['note'] = 'has no serial number: this address follows the USB port, so keep it plugged in there'
        elif row.get('port'):
            row['address'] = f"{kind}://{row['port']}"
        else:
            row['address'] = None
        if row['problem'] == 'permission':
            row['note'] = 'no permission to open it: ' + usb_permission_fix()
    return rows


def _scan():
    rows, errors = [], []
    for find in (_serial_adapters, _gs_usb_adapters):
        try:
            rows += find()
        except Exception as exc:                          # no libusb, or no USB access at all: the other kind still lists
            errors.append(f'{type(exc).__name__}: {exc}')
    return _with_addresses(rows), errors


def list_adapters():
    """Every connected USB CAN adapter: kind, address, serial, usb_port, product, and problem/note when there is one."""
    rows = _scan()[0]
    return sorted(({k: v for k, v in r.items() if k != 'dev'} for r in rows), key=lambda r: r['address'] or '')


def dfu_adapters():
    """How many STM32 devices sit in firmware-update mode: an adapter there is not a CAN adapter until it restarts."""
    try:
        return sum((d.idVendor, d.idProduct) == ST_DFU_ID for d in _usb_all())
    except Exception:
        return 0


def _known(kind):
    rows = [r for r in _scan()[0] if r['kind'] == kind and r['address']]
    return ', '.join(r['address'] for r in rows) or 'none'


def _find(address):
    """The connected adapter `address` names."""
    kind, _, target = address.partition('://')
    rows, errors = _scan()
    same = [r for r in rows if r['kind'] == kind]
    if target.startswith('port/'):
        matches = [r for r in same if r['usb_port'] == target[5:]]
    else:
        matches = [r for r in same if r['serial'] == target]
    if len(matches) == 1:
        if matches[0]['problem'] == 'permission':
            raise AdapterError(address, 'this computer sees the adapter but this user may not open it', usb_permission_fix())
        return matches[0]
    if len(matches) > 1:
        ports = ', '.join(r['address'] for r in matches)
        raise AdapterError(address, f'{len(matches)} connected adapters share this serial number',
                           f'address one by its USB port: {ports}')
    if any(r['problem'] == 'permission' for r in same):
        raise AdapterError(address, 'a CAN adapter is connected, but this user may not open it to read its serial number',
                           usb_permission_fix())
    if errors and not same:
        raise AdapterError(address, f'USB could not be read ({errors[-1]})', 'check the install with: roborun hw detect')
    if dfu_adapters():
        raise AdapterError(address, 'this adapter is not connected, and an STM32 device is in firmware-update (DFU) mode',
                           'remove the boot jumper or leave DFU mode (dfu-util -a 0 -s :leave), then run: roborun hw detect')
    raise AdapterError(address, f'this adapter is not connected (connected {kind} adapters: {_known(kind)})',
                       'check the USB cable, or run `roborun hw detect` for the address to use')


def _device_row(address, device):
    if not os.path.exists(device):
        raise AdapterError(address, f'{device} does not exist', 'check the USB cable, or run `roborun hw detect`')
    return dict(kind='slcan', serial=None, usb_port=None, port=device, product=None, problem=None)


# -- one owner per bus ----------------------------------------------------------------------------------------------
def _holder_text(path):
    try:
        pid = int(path.read_text().split()[0])
        name = (PROC / str(pid) / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
        return f'process {pid}' + (f' ({name[:80]})' if name else '')
    except (OSError, ValueError, IndexError):
        return 'another process'


def _claim(address, key=None):
    """Exclusive ownership of a bus across processes; released when the returned file closes."""
    import fcntl
    digest = hashlib.sha256((key or address).encode()).hexdigest()[:16]
    path = Path(tempfile.gettempdir()) / f'roborun-can-{digest}.lock'
    lock = path.open('a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        lock.close()
        raise AdapterError(address, f'it is in use by {_holder_text(path)}',
                           'stop that program (`roborun down` if it is this robot), then try again') from None
    lock.seek(0); lock.truncate(); lock.write(str(os.getpid())); lock.flush()
    return lock


# -- socketcan links ------------------------------------------------------------------------------------------------
def _parse_link(link):
    data = link.get('linkinfo') or {}
    can = data.get('info_data') or {}
    state = 'up' if 'UP' in (link.get('flags') or ()) else 'down'
    if state == 'up' and str(can.get('state', '')).upper() in ('BUS-OFF', 'STOPPED'):
        state = str(can['state']).lower()
    mode = can.get('ctrlmode') or ()                     # a list of flags, or {flag: ...} in older iproute2
    return dict(state=state, kind=data.get('info_kind'), bitrate=(can.get('bittiming') or {}).get('bitrate'), fd='FD' in mode)


def _sysfs_link(name):
    """The same facts from sysfs, for a computer without `ip`. None when there is no such link."""
    try:
        if int((SYS_NET / name / 'type').read_text()) != ARPHRD_CAN:
            return None
        up = int((SYS_NET / name / 'flags').read_text(), 16) & 1
    except (OSError, ValueError):
        return None
    return dict(state='up' if up else 'down', kind=None, bitrate=None, fd=None)


def _link_info(name, run=subprocess.run):
    """{state: up|down|bus-off|stopped, kind, bitrate}, or None when this computer has no such CAN link."""
    try:
        done = run(['ip', '-details', '-json', 'link', 'show', name], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return _sysfs_link(name)
    if done.returncode:
        return None
    try:
        link = json.loads(done.stdout)[0]
    except (ValueError, IndexError):
        return _sysfs_link(name)
    return _parse_link(link) if link.get('link_type', 'can') == 'can' else None


def _link_names(run=subprocess.run):
    try:
        done = run(['ip', '-json', 'link', 'show'], capture_output=True, text=True, timeout=5)
        return [link['ifname'] for link in json.loads(done.stdout) if link.get('link_type') == 'can']
    except (OSError, ValueError, KeyError, subprocess.SubprocessError):
        try:
            return sorted(p.name for p in SYS_NET.iterdir() if _sysfs_link(p.name))
        except OSError:
            return []


def can_links(run=subprocess.run):
    """[{name, state, kind, bitrate, usb_port, driver}] for this computer's CAN network links (Linux)."""
    out = []
    for name in _link_names(run):
        info = _link_info(name, run)
        if info is None:
            continue
        row = dict(name=name, usb_port=None, driver=None, **info)
        device = SYS_NET / name / 'device'
        if device.exists():
            match = re.search(r'/(\d+-[\d.]+):\d+\.\d+$', os.path.realpath(device))
            row['usb_port'] = match[1] if match else None
            driver = device / 'driver'
            row['driver'] = os.path.basename(os.path.realpath(driver)) if driver.exists() else None
        out.append(row)
    return out


def _link_commands(name, kind, bitrate, running):
    """What makes this link usable, as argument lists without sudo."""
    if kind == 'vcan':
        return [['ip', 'link', 'set', name, 'up']]
    steps = [['ip', 'link', 'set', name, 'down']] if running else []
    return steps + [['ip', 'link', 'set', name, 'up', 'type', 'can', 'bitrate', str(bitrate)]]


def fd_commands(name, bitrate=BITRATE, data_bitrate=FD_DATA_BITRATE):
    """What brings a link up in CAN FD mode (OpenArm's `-fd` setup: 1 Mbit/s arbitration, 5 Mbit/s data)."""
    return [['ip', 'link', 'set', name, 'down'],
            ['ip', 'link', 'set', name, 'up', 'type', 'can', 'bitrate', str(bitrate), 'dbitrate', str(data_bitrate), 'fd', 'on']]


def _run_all(commands, run, prefix=()):
    for command in commands:
        try:
            if run([*prefix, *command], capture_output=True, text=True, timeout=5).returncode:
                return False
        except (OSError, subprocess.SubprocessError):
            return False
    return True


def ensure_link(name, bitrate=BITRATE, run=subprocess.run, platform=None):
    """The socketcan link is up, at `bitrate` and not bus-off: fixed here when this user may, else the exact command."""
    if (platform or sys.platform) != 'linux':
        raise AdapterError(name, 'CAN network links exist only on Linux',
                           'use the adapter directly: `roborun hw detect` prints its slcan:// or gs_usb:// address')
    info = _link_info(name, run)
    if info is None:
        others = ', '.join(_link_names(run)) or 'none'
        raise AdapterError(name, f'there is no CAN link with this name (CAN links here: {others})',
                           '`roborun hw detect` lists every adapter and link with the address to use')
    wrong_rate = info['kind'] != 'vcan' and info['bitrate'] not in (None, bitrate)
    if info['state'] == 'up' and not wrong_rate:
        return info
    why = (f"the link runs at {info['bitrate']} bit/s, not {bitrate}" if info['state'] == 'up'
           else f"the link is {info['state'] or 'not up'}")
    commands = _link_commands(name, info['kind'], bitrate, info['state'] != 'down')
    if _run_all(commands, run) or (shutil.which('sudo') and _run_all(commands, run, ('sudo', '-n'))):
        info = _link_info(name, run) or {}
        if info.get('state') == 'up':
            log.info('%s: brought up at %s bit/s', name, bitrate)
            return info
    raise AdapterError(name, why, 'run: ' + ' && '.join('sudo ' + shlex.join(c) for c in commands))


# -- who has a serial device open (Linux) ----------------------------------------------------------------------------
def _tty_holders(device):
    """[(pid, name, argv)] of the other processes with `device` open, as far as this user can see."""
    target, out = os.path.realpath(device), []
    try:
        pids = [p for p in PROC.iterdir() if p.name.isdigit() and int(p.name) != os.getpid()]
    except OSError:
        return out
    for proc in pids:
        try:
            argv = (proc / 'cmdline').read_bytes().decode(errors='replace').split('\0')
            uses = any(os.path.realpath(fd) == target for fd in (proc / 'fd').iterdir())
        except OSError:
            continue
        if uses:
            out.append((int(proc.name), os.path.basename(argv[0]), [a for a in argv if a]))
    return out


def _check_tty_free(address, device):
    for pid, name, argv in _tty_holders(device):
        if name == 'slcand':
            link = next((a for a in reversed(argv[1:]) if not a.startswith('-') and not a.startswith('/')), None)
            raise AdapterError(address, f'slcand (process {pid}) already holds this adapter'
                               + (f' and serves it as CAN link {link}' if link else ''),
                               f'use the address {link} instead, or stop it first: sudo kill {pid}' if link
                               else f'stop it first: sudo kill {pid}')
        raise AdapterError(address, f'{device} is open in {name} (process {pid})', 'close that program, then try again')


# -- candleLight / gs_usb, spoken directly over USB ------------------------------------------------------------------
_REQ_OUT, _REQ_IN = 0x41, 0xC1                            # vendor request, to the interface
_BREQ_HOST_FORMAT, _BREQ_BITTIMING, _BREQ_MODE, _BREQ_BT_CONST = 0, 1, 2, 4
_MODE_RESET, _MODE_START = 0, 1
_EP_IN, _EP_OUT = 0x81, 0x02
_FRAME = struct.Struct('<IIBBBB8s')                       # echo_id, can_id, dlc, channel, flags, reserved, data
_RX_ECHO = 0xFFFFFFFF                                     # echo_id of a received frame; anything else confirms a send
_FLAG_OVERFLOW = 0x01
_CAN_EFF, _CAN_RTR, _CAN_ERR = 0x80000000, 0x40000000, 0x20000000
_TX_SLOTS = 8


def gs_usb_timing(bt_const, bitrate=BITRATE, sample_point=0.875):
    """(prop_seg, phase_seg1, phase_seg2, sjw, brp) for an adapter's BT_CONST values, closest to the sample point."""
    _feature, fclk, t1_min, t1_max, t2_min, t2_max, sjw_max, brp_min, brp_max, brp_inc = bt_const
    best = None
    for brp in range(brp_min, brp_max + 1, max(1, brp_inc)):
        quanta = fclk / (brp * bitrate)
        if abs(quanta - round(quanta)) > 1e-9:
            continue
        quanta = round(quanta)
        seg2 = max(t2_min, min(t2_max, round(quanta * (1 - sample_point))))
        seg1 = quanta - 1 - seg2
        if not t1_min <= seg1 <= t1_max:
            continue
        error = abs((1 + seg1) / quanta - sample_point)
        if best is None or error < best[0] - 1e-9:
            best = (error, brp, seg1, seg2)
    if best is None:
        raise RuntimeError(f'the adapter cannot run CAN at {bitrate} bit/s from a {fclk} Hz clock')
    _, brp, seg1, seg2 = best
    return 1, seg1 - 1, seg2, max(1, min(sjw_max, seg2 // 2)), brp


def _open_gs_usb(address, device, bitrate):
    import can
    import usb.core
    import usb.util

    class GsUsbBus(can.BusABC):
        """One channel of a gs_usb adapter. A reader thread keeps a transfer pending, so no frame waits on a poll.
        The device is never USB-reset: that re-enumerates it, which macOS handles badly."""

        def __init__(self, dev):
            self._dev, self._detached, self._claimed, self._started = dev, False, False, False
            self._cond = threading.Condition()
            self._rx = collections.deque()
            self._pending, self._next_echo = {}, 0
            self._fault = None
            self._closing = threading.Event()
            self._write_lock = threading.Lock()
            self._can_protocol = can.CanProtocol.CAN_20
            self.channel_info = address
            try:
                self._configure(bitrate)
            except BaseException:
                self._release()
                raise
            self._reader = threading.Thread(target=self._read_loop, daemon=True, name=f'gs-usb-{address}')
            self._reader.start()
            super().__init__(channel=address)

        def _configure(self, rate):
            dev = self._dev
            try:
                if dev.is_kernel_driver_active(0):                # Linux: the gs_usb module has it
                    dev.detach_kernel_driver(0)
                    self._detached = True
            except (NotImplementedError, usb.core.USBError):      # macOS has no kernel driver to detach
                pass
            try:
                dev.get_active_configuration()
            except usb.core.USBError:
                dev.set_configuration()
            usb.util.claim_interface(dev, 0)
            self._claimed = True
            dev.ctrl_transfer(_REQ_OUT, _BREQ_HOST_FORMAT, 1, 0, struct.pack('<I', 0x0000BEEF), 1000)
            dev.ctrl_transfer(_REQ_OUT, _BREQ_MODE, 0, 0, struct.pack('<II', _MODE_RESET, 0), 1000)
            raw = bytes(dev.ctrl_transfer(_REQ_IN, _BREQ_BT_CONST, 0, 0, 40, 1000))
            prop, seg1, seg2, sjw, brp = gs_usb_timing(struct.unpack('<10I', raw), rate)
            dev.ctrl_transfer(_REQ_OUT, _BREQ_BITTIMING, 0, 0, struct.pack('<5I', prop, seg1, seg2, sjw, brp), 1000)
            dev.ctrl_transfer(_REQ_OUT, _BREQ_MODE, 0, 0, struct.pack('<II', _MODE_START, 0), 1000)
            self._started = True

        def _read_loop(self):
            while not self._closing.is_set():
                try:
                    raw = bytes(self._dev.read(_EP_IN, 512, 100))
                except usb.core.USBTimeoutError:
                    continue
                except usb.core.USBError as exc:
                    if not self._closing.is_set():
                        with self._cond:
                            self._fault = f'{address}: USB failure: {exc}'
                            self._cond.notify_all()
                    return
                for at in range(0, len(raw) - _FRAME.size + 1, _FRAME.size):
                    self._on_frame(*_FRAME.unpack_from(raw, at))

        def _on_frame(self, echo_id, can_id, dlc, _channel, flags, _reserved, data):
            with self._cond:
                if echo_id != _RX_ECHO:                           # the adapter finished sending one of ours
                    self._pending.pop(echo_id, None)
                    self._cond.notify_all()
                    return
                if flags & _FLAG_OVERFLOW:
                    log.warning('%s: the adapter dropped received frames', address)
                if can_id & _CAN_ERR:
                    return
                self._rx.append(can.Message(
                    timestamp=time.time(), arbitration_id=can_id & 0x1FFFFFFF, is_extended_id=bool(can_id & _CAN_EFF),
                    is_remote_frame=bool(can_id & _CAN_RTR), dlc=min(dlc, 8), data=data[:min(dlc, 8)],
                    is_rx=True, channel=address))
                self._cond.notify_all()

        def _recv_internal(self, timeout):
            deadline = None if timeout is None else time.monotonic() + timeout
            with self._cond:
                while True:
                    if self._fault:
                        raise can.CanOperationError(self._fault)
                    if self._rx:
                        return self._rx.popleft(), False
                    left = None if deadline is None else deadline - time.monotonic()
                    if left is not None and left <= 0:
                        return None, False
                    self._cond.wait(left)

        def send(self, msg, timeout=None):
            budget = 0.05 if timeout is None else timeout
            deadline = time.monotonic() + budget
            with self._cond:
                while True:
                    if self._fault:
                        raise can.CanOperationError(self._fault)
                    now = time.monotonic()
                    self._pending = {k: t for k, t in self._pending.items() if now - t < 1.0}
                    if len(self._pending) < _TX_SLOTS:
                        break
                    if now >= deadline:
                        raise can.CanOperationError(f'{address}: frames are not leaving the adapter (bus off or no motor acknowledging?)')
                    self._cond.wait(deadline - now)
                echo = self._next_echo
                self._next_echo = (echo + 1) % 256
                self._pending[echo] = now
            can_id = msg.arbitration_id | (_CAN_EFF if msg.is_extended_id else 0) | (_CAN_RTR if msg.is_remote_frame else 0)
            frame = _FRAME.pack(echo, can_id, msg.dlc, 0, 0, 0, bytes(msg.data).ljust(8, b'\0'))
            try:
                with self._write_lock:
                    self._dev.write(_EP_OUT, frame, max(1, int(budget * 1000)))
            except usb.core.USBError as exc:
                with self._cond:
                    self._pending.pop(echo, None)
                raise can.CanOperationError(f'{address}: send failed: {exc}') from exc

        def _release(self):
            dev = self._dev
            steps = []
            if self._started:
                steps.append(lambda: dev.ctrl_transfer(_REQ_OUT, _BREQ_MODE, 0, 0, struct.pack('<II', _MODE_RESET, 0), 200))
            if self._claimed:
                steps.append(lambda: usb.util.release_interface(dev, 0))
            if self._detached:
                steps.append(lambda: dev.attach_kernel_driver(0))
            steps.append(lambda: usb.util.dispose_resources(dev))
            for step in steps:
                try:
                    step()
                except (usb.core.USBError, NotImplementedError, ValueError):
                    pass
            self._started = self._claimed = self._detached = False

        def shutdown(self):
            if self._is_shutdown:
                return
            super().shutdown()
            self._closing.set()
            self._reader.join(timeout=1)
            self._release()

    try:
        return GsUsbBus(device)
    except usb.core.NoBackendError as exc:
        raise AdapterError(address, 'libusb is missing', 'reinstall Roborun: curl -fsSL https://roborun.app/install.sh | sh') from exc
    except (usb.core.USBError, ValueError) as exc:
        code = getattr(exc, 'errno', None)
        if code in (errno.EACCES, errno.EPERM) or 'permission' in str(exc).lower():
            raise AdapterError(address, 'this user may not open the adapter', usb_permission_fix()) from exc
        if code == errno.EBUSY:
            raise AdapterError(address, 'another program or the kernel driver is using the adapter',
                               'stop the program using it; on Linux also try: sudo ip link set can0 down') from exc
        if code in (errno.ENODEV, errno.ENOENT, errno.EIO, errno.EPIPE):
            raise AdapterError(address, f'the adapter stopped answering over USB ({exc})', 'replug it and try again') from exc
        raise


def _owned(bus, lock):
    """`bus`, with the adapter's ownership released when it shuts down."""
    close = bus.shutdown

    def shutdown():
        try:
            close()
        finally:
            lock.close()
    bus.shutdown = shutdown
    return bus


def _open_slcan(address, device, bitrate):
    import serial
    from can.interfaces.slcan import slcanBus

    class Adapter(slcanBus):
        def open(self):                                   # these adapters can keep a nonstandard mode across power cycles
            for command in ('C', 'M0', 'A1', 'O'):
                self._write(command)
                time.sleep(.02)
    _check_tty_free(address, device)
    try:
        return Adapter(channel=device, bitrate=bitrate, sleep_after_open=.2, timeout=.001)
    except (serial.SerialException, OSError) as exc:
        code = getattr(exc, 'errno', None)
        if code in (errno.EACCES, errno.EPERM) or 'permission' in str(exc).lower():
            raise AdapterError(address, f'this user may not open {device}', serial_permission_fix(device)) from exc
        if code == errno.EBUSY or 'busy' in str(exc).lower():
            raise AdapterError(address, f'{device} is in use by another program', 'close it, then try again') from exc
        if code == errno.ENOENT:
            raise AdapterError(address, f'{device} is gone', 'check the USB cable, or run `roborun hw detect`') from exc
        raise


def _open_socketcan(address, bitrate):
    """The bus speaks CAN FD when the link was brought up with `fd on` (OpenArm's `-fd` setup): motors set up for FD
    answer only FD frames, so the link's own mode decides, and `bus.roborun_fd` tells the arm which frames to send."""
    import can
    lock = _claim(address, f'socketcan:{address}')
    try:
        fd = bool(ensure_link(address, bitrate).get('fd'))
        bus = _owned(can.Bus(interface='socketcan', channel=address, bitrate=bitrate, fd=fd), lock)
        bus.roborun_fd = fd
        return bus
    except OSError as exc:
        lock.close()
        raise AdapterError(address, f'the CAN link could not be opened ({exc})', '`roborun hw detect` shows its state') from exc
    except BaseException:
        lock.close()
        raise


ADDRESS = re.compile(r'(slcan|gs_usb)://(.+)')


def resolve(address):
    """The adapter row for a USB or device address, without opening it. Raises AdapterError when it cannot be used."""
    match = ADDRESS.fullmatch(address)
    if match[2].startswith('/'):
        if match[1] != 'slcan':
            raise AdapterError(address, 'gs_usb adapters have no device path',
                               'use gs_usb://SERIAL or gs_usb://port/PATH: `roborun hw detect` prints it')
        return _device_row(address, match[2])
    return _find(address)


def check(address, bitrate=BITRATE):
    """What `roborun up` asks before it starts anything in the background: is this bus there and usable?"""
    if address.startswith('virtual://'):
        return
    if ADDRESS.fullmatch(address):
        row = resolve(address)
        if row['kind'] == 'slcan':
            _check_tty_free(address, row['port'])
        return
    ensure_link(address, bitrate)


def open_bus(address, bitrate=BITRATE):
    """A python-can bus for ."""
    import can
    if address.startswith('virtual://'):
        return can.Bus(interface='virtual', channel=address[len('virtual://'):], receive_own_messages=False)
    match = ADDRESS.fullmatch(address)
    if match is None:
        return _open_socketcan(address, bitrate)
    row = resolve(address)
    key = f"slcan:{os.path.realpath(row['port'])}" if row['kind'] == 'slcan' else f"gs_usb:{row['usb_port'] or row['serial']}"
    lock = _claim(address, key)                           # both spellings of one adapter contend for the same lock
    try:
        if row['kind'] == 'gs_usb':
            return _owned(_open_gs_usb(address, row['dev'], bitrate), lock)
        return _owned(_open_slcan(address, row['port'], bitrate), lock)
    except BaseException:
        lock.close()
        raise


# -- bus load ----------------------------------------------------------------------------------------------------------
# A classic CAN frame with an 11-bit id and 8 data bytes is 111 bits before bit stuffing and up to about 135 with it;
# 130 is the figure i2rt's own bandwidth check uses. A bus is treated as saturated past 1/1.1 of its bitrate, as there.
FRAME_BITS = 130
LOAD_LIMIT = 1 / 1.1


def bus_load(frames_per_s, bitrate=BITRATE):
    """The share of the bus `frames_per_s` classic frames take."""
    return float(frames_per_s) * FRAME_BITS / float(bitrate)


def load_warning(address, frames_per_s, bitrate=BITRATE, who='the arm'):
    """None, or one line saying the stream would saturate the bus, with what to change."""
    load = bus_load(frames_per_s, bitrate)
    if load <= LOAD_LIMIT:
        return None
    return (f'{address}: {who} would put about {frames_per_s:.0f} frames/s on a {bitrate // 1000} kbit/s CAN bus, '
            f'{load:.0%} of what it carries: replies will be late or lost and the arm will stop as silent. Next: lower '
            f'rate_hz, or put the arms on separate adapters')
