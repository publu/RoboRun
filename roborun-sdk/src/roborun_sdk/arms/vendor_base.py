"""What every driver built on a maker's SDK shares.

The maker's SDK is imported only when an arm is opened, so Roborun installs without it. The joint count comes from the
catalog (`limits`): a 7-joint arm stays a 7-joint arm, and a command of any other length is refused rather than cut down
or padded. Motors are engaged by the first stream write, which by construction carries the measured pose.
"""
import importlib
import math
import threading
from urllib.parse import urlsplit

from .base import ArmHardwareError, StreamedArm


def load_sdk(name: str, label: str, extra: str = None, hint: str = None):
    try:
        return importlib.import_module(name)
    except ImportError as exc:
        how = hint or 'it is not available for this computer'
        raise SystemExit(f'{label} needs {name}, which could not be imported ({exc}): {how}')


def host_port(bus, default_port=None, default_host=None):
    """'scheme://host:port', 'host:port' or 'host' -> (host, port)."""
    text = str(bus or '')
    if not text:
        if default_host is None:
            raise ValueError('this arm needs an address: ARM@HOST')
        return default_host, default_port
    part = urlsplit(text if '://' in text else '//' + text)
    return part.hostname or default_host, part.port or default_port


def check_code(code, what):
    if code not in (0, None):
        raise ArmHardwareError(f'{what} returned {code}')


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class ContinuousJoints:
    """Joints that turn without a stop: reported near zero and without a jump, commanded back in the arm's own turns."""

    def __init__(self, indices):
        self.indices = tuple(indices)
        self.last, self.offset = {}, {}

    def measured(self, raw):
        out = list(raw)
        for i in self.indices:
            if i not in self.last:
                self.last[i] = wrap_pi(raw[i])
                self.offset[i] = raw[i] - self.last[i]
            else:
                self.last[i] += wrap_pi(raw[i] - self.offset[i] - self.last[i])
            out[i] = self.last[i]
        return out

    def command(self, q):
        out = list(q)
        for i in self.indices:
            out[i] = q[i] + self.offset.get(i, 0.0)
        return out


class VendorArm(StreamedArm):
    """A StreamedArm whose hardware is a vendor SDK. Subclasses implement the sdk_* methods; all run under one lock."""
    label = 'arm'
    has_gripper = True
    jaw_every = 1                       # write the jaw every N stream cycles (some grippers are slow links)
    jaw_deadband = 0.0
    jaw_guard = False                   # the maker's gripper controller limits its own force, and may be slow

    def __init__(self, bus=None, side='left', home=None, limits=None, side_limits=None, side_home=None, **options):
        if side_limits and side in side_limits: limits = side_limits[side]
        if side_home and side in side_home: home = side_home[side]
        if limits is None:
            raise ValueError(f'{self.label}: its maker publishes no joint limits; pass limits=[[lo, hi], ...] for your arm')
        if home is None:
            raise ValueError(f'{self.label}: pass home=[...]')
        super().__init__(bus=bus, side=side, home=home, limits=limits, **options)
        self.dof = len(self.limits)
        if len(self.home) != self.dof:
            raise ValueError(f'{self.label}: home has {len(self.home)} joints, limits have {self.dof}')
        self._io = threading.RLock()
        self._engaged = False
        self._jaw_tick = 0
        self._jaw_sent = None

    # -- a driver implements these ------------------------------------------------------------------------------
    def sdk_open(self):
        """Connect. No enabling, no mode change that can move the arm."""
        raise NotImplementedError

    def sdk_read(self) -> dict:
        """{positions_rad, [gripper], [velocities_rad_s], ...}; raises ArmHardwareError when the arm reports a fault."""
        raise NotImplementedError

    def sdk_engage(self, q, jaw):
        """Enter the maker's streaming mode, holding q (the measured pose)."""
        raise NotImplementedError

    def sdk_write(self, q):
        raise NotImplementedError

    def sdk_jaw(self, jaw):
        """Send the jaw target (0 closed .. 1 open); return False if it could not be taken yet."""

    def sdk_limp(self):
        raise NotImplementedError

    def sdk_close(self):
        pass

    def sdk_survey(self) -> dict:
        raise NotImplementedError

    # -- StreamedArm ----------------------------------------------------------------------------------------------
    def open_hardware(self):
        with self._io:
            try:
                self.sdk_open()
            except BaseException:
                self._close_quietly()
                raise

    def open_dry(self):
        self.open_hardware()                                  # sdk_open connects only: engaging happens at the first write

    def read_hardware(self):
        with self._io:
            sample = dict(self.sdk_read())
        q = [float(v) for v in sample['positions_rad']]
        if len(q) != self.dof or not all(math.isfinite(v) for v in q):
            raise ArmHardwareError(f'{self.label}: expected {self.dof} finite joint angles, got {q}')
        sample['positions_rad'] = q
        if not self.has_gripper or sample.get('gripper') is None:
            sample['gripper'], sample['gripper_available'] = 1.0, False
        return sample

    def write_hardware(self, q, jaw):
        if len(q) != self.dof:
            raise ArmHardwareError(f'{self.label}: refusing a {len(q)}-joint target, this arm has {self.dof}')
        with self._io:
            if self._limp:                                    # a release landed while this cycle was on its way here
                return
            if not self._engaged:
                # an arm found slightly outside a limit may hold there; it is never driven further out
                self._bounds = [(min(lo, v), max(hi, v)) for v, (lo, hi) in zip(q, self.limits)]
                self.sdk_engage(list(q), jaw)
                self._engaged = True
                self._jaw_sent = jaw
            # the shaper can pass its target by about a milliradian as it settles: the limit still holds on the wire
            self.sdk_write([max(lo, min(hi, v)) for v, (lo, hi) in zip(q, self._bounds)])
            if self.has_gripper:
                self._jaw_tick += 1
                moved = self._jaw_sent is None or abs(jaw - self._jaw_sent) > self.jaw_deadband
                if self._jaw_tick >= self.jaw_every and moved:
                    self._jaw_tick = 0
                    if self.sdk_jaw(jaw) is not False:           # False: the gripper is busy, try again next time
                        self._jaw_sent = jaw

    def limp_hardware(self):
        with self._io:
            was, self._engaged = self._engaged, False
            if was:
                self.sdk_limp()

    def close_hardware(self):
        with self._io:
            self._close_quietly()

    def _close_quietly(self):
        self._engaged = False
        try:
            self.sdk_close()
        except Exception:
            pass

    def command(self, positions_rad, gripper):
        values = [float(v) for v in positions_rad]
        if len(values) != self.dof or not all(math.isfinite(v) for v in values):
            raise ValueError(f'{self.label}: a command must be {self.dof} finite joint angles, got {len(values)}')
        super().command(values, gripper)

    def survey(self) -> dict:
        """Read-only: its own connection, no enable, no mode change, no motion command."""
        out = dict(self.sdk_survey())
        out.update(arm=self.label, joints=self.dof, read_only=True)
        return out
