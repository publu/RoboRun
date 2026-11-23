"""AgileX Piper through piper_sdk (CAN). Stream: CAN command mode, MOVE J, JointCtrl at the stream rate."""
import math
import time

from .base import ArmHardwareError
from .vendor_base import VendorArm, load_sdk

MILLIDEG = 180000.0 / math.pi


class PiperArm(VendorArm):
    label = 'AgileX Piper'
    rate_hz = 200.0
    gripper_travel_m = 0.07
    gripper_effort = 1000                # 0.001 N*m, the value in the maker's joint control example

    def _interface(self):
        sdk = load_sdk('piper_sdk', self.label, extra='piper')
        arm = sdk.C_PiperInterface_V2(str(self.bus or 'can0'))
        arm.ConnectPort()                # opens the bus and sends the SDK's parameter queries; nothing moves
        return arm

    @staticmethod
    def _joints(arm):
        j = arm.GetArmJointMsgs().joint_state
        return [getattr(j, f'joint_{i}') / MILLIDEG for i in range(1, 7)]

    def _wait_for_feedback(self, arm, seconds=2.0):
        end = time.monotonic() + seconds
        while arm.GetArmJointMsgs().time_stamp == 0:
            if time.monotonic() > end:
                raise ArmHardwareError('no joint feedback on the CAN bus')
            time.sleep(.01)

    def sdk_open(self):
        self.arm = self._interface()
        self._wait_for_feedback(self.arm)

    def sdk_read(self):
        status = self.arm.GetArmStatus().arm_status
        if int(status.arm_status) != 0:
            raise ArmHardwareError(f'arm status {status.arm_status}')
        if not self.arm.isOk():
            raise ArmHardwareError('CAN reader stopped')
        grip = self.arm.GetArmGripperMsgs().gripper_state
        return dict(positions_rad=self._joints(self.arm),
                    gripper=max(0.0, min(1.0, grip.grippers_angle / 1e6 / self.gripper_travel_m)))

    def sdk_engage(self, q, jaw):
        end = time.monotonic() + 3.0
        while not self.arm.EnablePiper():
            if time.monotonic() > end:
                raise ArmHardwareError('the arm did not enable')
            time.sleep(.01)
        self.sdk_write(q)
        self.sdk_jaw(jaw, code=0x01)

    def sdk_write(self, q):
        self.arm.MotionCtrl_2(0x01, 0x01, 100, 0x00)          # CAN control, joint move, position-speed mode (not MIT)
        self.arm.JointCtrl(*[round(v * MILLIDEG) for v in q])

    def sdk_jaw(self, jaw, code=0x01):
        self.arm.GripperCtrl(round(jaw * self.gripper_travel_m * 1e6), self.gripper_effort, code, 0)

    def sdk_limp(self):
        self.arm.DisableArm(7)           # torque off: this arm has no gravity-compensated mode in the SDK, it will drop

    def sdk_close(self):
        arm = getattr(self, 'arm', None)
        if arm is not None:
            self.arm = None
            arm.DisconnectPort()

    def sdk_survey(self):
        arm = self._interface()
        try:
            self._wait_for_feedback(arm)
            status = arm.GetArmStatus().arm_status
            return dict(positions_rad=self._joints(arm), arm_status=str(status.arm_status), ctrl_mode=str(status.ctrl_mode),
                        firmware=arm.GetPiperFirmwareVersion(), enabled=arm.GetArmEnableStatus())
        finally:
            arm.DisconnectPort()
