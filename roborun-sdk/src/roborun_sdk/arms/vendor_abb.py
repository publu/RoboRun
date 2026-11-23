"""ABB YuMi (IRB 14000, IRC5) through abb_robot_client's EGM client (UDP).

The controller side is yours to provide: RobotWare 6 with the Externally Guided Motion option, and a RAPID program in
EGMRunJoint for this arm sending to this computer. ADDRESS is the local UDP port (default 6511 left, 6512 right).

Joint order here is the arm's kinematic order: axes 1, 2, 7, 3, 4, 5, 6. RAPID and EGM keep axis 7 apart, as the first
external axis. That mapping is from ABB's joint target convention and has not been checked on a real YuMi. The grippers
are not on EGM and are not driven.
"""
import math
import time

from .base import ArmHardwareError
from .vendor_base import VendorArm, load_sdk

DEFAULT_PORT = dict(left=6511, right=6512)


class YuMiArm(VendorArm):
    label = 'ABB YuMi arm'
    has_gripper = False

    def _egm(self):
        sdk = load_sdk('abb_robot_client.egm', self.label, extra='abb')
        port = int(str(self.bus).rsplit(':', 1)[-1].strip('/')) if self.bus else DEFAULT_PORT[self.side]
        return sdk.EGM(port=port)

    def _drain(self, egm, wait):
        newest = None
        ok, state = egm.receive_from_robot(timeout=wait)
        while ok:
            newest = state
            ok, state = egm.receive_from_robot(timeout=0)
        return newest

    def _kinematic(self, state):
        axes = [float(v) for v in state.joint_angles]
        extra = [] if state.external_axes is None else [float(v) for v in state.external_axes]
        self.axis7_external = len(axes) == 6
        if len(axes) == 7:
            axis7 = axes.pop()
        elif len(axes) == 6 and extra:
            axis7 = extra[0]
        else:
            raise ArmHardwareError(f'EGM reported {len(axes)} joints and {len(extra)} external axes, expected 7 in all')
        return [math.radians(v) for v in axes[:2] + [axis7] + axes[2:]]

    def sdk_open(self):
        self.egm = self._egm()
        self.state, self.state_at = None, 0.0
        end = time.monotonic() + 5.0
        while self.state is None:
            self.state = self._drain(self.egm, 0.05)
            if time.monotonic() > end:
                raise ArmHardwareError('no EGM data: is the RAPID program running with this computer as its partner?')
        self.state_at = time.monotonic()

    def sdk_read(self):
        state = self._drain(self.egm, 0.005)
        now = time.monotonic()
        if state is not None:
            self.state, self.state_at = state, now
        elif now - self.state_at > 0.2:
            raise ArmHardwareError('EGM data stopped')
        if not (self.state.rapid_running and self.state.motors_on):
            raise ArmHardwareError('motors are off or the RAPID program stopped')
        return dict(positions_rad=self._kinematic(self.state))

    def sdk_engage(self, q, jaw):
        pass                                                         # EGMRunJoint is already waiting for corrections

    def sdk_write(self, q):
        deg = [math.degrees(v) for v in q]
        axes, axis7 = deg[:2] + deg[3:], deg[2]
        if self.axis7_external:
            sent = self.egm.send_to_robot(axes, external_joints=[axis7])
        else:
            sent = self.egm.send_to_robot(axes + [axis7])
        if not sent:
            raise ArmHardwareError('EGM send failed')

    def sdk_limp(self):
        pass                                                         # no more corrections: EGM ends on its own timeout

    def sdk_close(self):
        egm = getattr(self, 'egm', None)
        if egm is not None:
            self.egm = None
            egm.close()

    def sdk_survey(self):
        self.sdk_open()                                              # listening only: nothing is sent
        try:
            return dict(positions_rad=self._kinematic(self.state), rapid_running=bool(self.state.rapid_running),
                        motors_on=bool(self.state.motors_on))
        finally:
            self.sdk_close()
