"""Unitree R1-A7 arms through unitree_sdk2py (DDS, `ADDRESS` is the network interface, e.g. eth0).

Stream: rt/lowcmd with position targets and the gains from Unitree's r1_A7_low_level_example. One message carries all
16 upper-body motors, so both arms and the head share one link: an arm that is not driven holds where it was first seen.
Unitree's motion service must already be released (Unitree's app or MotionSwitcherClient): releasing it takes balance
control away, so Roborun refuses to start while it runs instead of releasing it for you. No gripper API is public.
"""
import threading
import time

from .base import ArmHardwareError
from .vendor_base import VendorArm, load_sdk

HINT = 'install unitree_sdk2_python from https://github.com/unitreerobotics/unitree_sdk2_python (needs cyclonedds 0.10.x)'
MOTORS = list(range(15, 31))             # left arm 15-21, right arm 22-28, head 29-30, in Unitree's order
ARM_MOTORS = dict(left=MOTORS[0:7], right=MOTORS[7:14])
KP = [100, 100, 100, 100, 50, 50, 50] * 2 + [50, 10]
KD = [2.0] * 14 + [2.0, 0.1]
LINKS, LINKS_LOCK = {}, threading.Lock()


class _Link:
    def __init__(self, interface):
        self.interface = interface
        self.channel = load_sdk('unitree_sdk2py.core.channel', 'Unitree R1', hint=HINT)
        self.lock = threading.RLock()
        self.state, self.state_at, self.users, self.publisher = None, 0.0, 0, None
        self.targets, self.limp = {}, set()
        self.channel.ChannelFactoryInitialize(0, interface)
        idl = load_sdk('unitree_sdk2py.idl.unitree_hg.msg.dds_', 'Unitree R1', hint=HINT)
        self.LowCmd = idl.LowCmd_
        self.crc = load_sdk('unitree_sdk2py.utils.crc', 'Unitree R1', hint=HINT).CRC()
        self.default = load_sdk('unitree_sdk2py.idl.default', 'Unitree R1', hint=HINT)
        self.switcher = load_sdk('unitree_sdk2py.comm.motion_switcher.motion_switcher_client', 'Unitree R1', hint=HINT)
        self.subscriber = self.channel.ChannelSubscriber('rt/lowstate', idl.LowState_)
        self.subscriber.Init(self._on_state, 10)

    def _on_state(self, msg):
        if msg.crc == self.crc.Crc(msg):
            with self.lock:
                self.state, self.state_at = msg, time.monotonic()

    def wait(self, seconds=3.0):
        end = time.monotonic() + seconds
        while self.state is None:
            if time.monotonic() > end:
                raise ArmHardwareError('no rt/lowstate on this network interface')
            time.sleep(.01)

    def read(self, motors):
        with self.lock:
            if self.state is None or time.monotonic() - self.state_at > 0.1:
                raise ArmHardwareError('rt/lowstate stopped')
            rows = [self.state.motor_state[m] for m in motors]
            return [float(r.q) for r in rows], [float(r.dq) for r in rows]

    def check_released(self):
        client = self.switcher.MotionSwitcherClient()
        client.SetTimeout(5.0)
        client.Init()
        status, result = client.CheckMode()
        if status != 0:
            raise ArmHardwareError(f'motion service query failed: {status}')
        if result.get('name'):
            raise ArmHardwareError(f'Unitree motion service "{result["name"]}" is active: release it first')

    def claim(self):
        with self.lock:
            if self.publisher is None:
                self.check_released()
                self.cmd = self.default.unitree_hg_msg_dds__LowCmd_()
                self.publisher = self.channel.ChannelPublisher('rt/lowcmd', self.LowCmd)
                self.publisher.Init()
                self.targets = dict(zip(MOTORS, self.read(MOTORS)[0]))   # everything holds where it is

    def publish(self, motors, q):
        with self.lock:
            self.targets.update(zip(motors, q))
            self._send()

    def engage(self, motors, q):
        with self.lock:
            self.claim()
            self.targets.update(zip(motors, q))                  # stiffness returns at the measured pose
            self.limp.difference_update(motors)
            self._send()

    def go_limp(self, motors):
        with self.lock:
            self.limp.update(motors)
            if self.publisher is not None:
                self._send()

    def _send(self):
        cmd = self.cmd
        cmd.mode_pr, cmd.mode_machine = 0, self.state.mode_machine
        for i, m in enumerate(MOTORS):
            row = cmd.motor_cmd[m]
            row.mode, row.q, row.dq, row.tau, row.kd = 1, self.targets[m], 0.0, 0.0, KD[i]
            row.kp = 0.0 if m in self.limp else KP[i]
        cmd.crc = self.crc.Crc(cmd)
        self.publisher.Write(cmd)


def _link(interface):
    with LINKS_LOCK:
        link = LINKS.get(interface) or LINKS.setdefault(interface, _Link(interface))
        link.users += 1
        return link


def _unlink(link):
    with LINKS_LOCK:
        link.users -= 1
        if link.users <= 0:
            LINKS.pop(link.interface, None)
            for end in (link.subscriber, link.publisher):
                if end is not None:
                    end.Close()


class UnitreeR1A7(VendorArm):
    label = 'Unitree R1-A7 arm'
    rate_hz = 250.0
    has_gripper = False

    def _interface(self):
        if not self.bus:
            raise ValueError('name the robot\'s network interface: unitree_r1_a7@eth0')
        return str(self.bus)

    def sdk_open(self):
        self.motors = ARM_MOTORS[self.side]
        self.link = _link(self._interface())
        self.link.wait()

    def sdk_read(self):
        q, dq = self.link.read(self.motors)
        return dict(positions_rad=q, velocities_rad_s=dq)

    def sdk_engage(self, q, jaw):
        self.link.engage(self.motors, q)

    def sdk_write(self, q):
        self.link.publish(self.motors, q)

    def sdk_limp(self):
        self.link.go_limp(self.motors)                               # zero stiffness, damping stays

    def sdk_close(self):
        link = getattr(self, 'link', None)
        if link is not None:
            self.link = None
            link.go_limp(self.motors)
            _unlink(link)

    def sdk_survey(self):
        link = _link(self._interface())
        try:
            link.wait()
            return dict(positions_rad=link.read(ARM_MOTORS[self.side])[0], mode_machine=link.state.mode_machine)
        finally:
            _unlink(link)
