"""ARX X5 through arx-x5-sdk, a packaging of ARX's C++ interface (CAN; Linux x86-64, CPython 3.10 only).

Stream: status 5 (joint position control) with set_joint_positions each cycle, the sequence ARX's single-arm script uses.
ARX publishes no joint limits (its URDFs carry +-10 rad placeholders), so `limits` must be given for your arm.
Building the SDK's interface object starts ARX's own control thread, and what that thread does to the motors is not
documented, so there is no read-only check for this arm.
"""
from .base import ArmHardwareError
from .vendor_base import VendorArm, load_sdk

JOINT_POSITION, GRAVITY_COMPENSATION = 5, 3


class ArxX5Arm(VendorArm):
    label = 'ARX X5'
    rate_hz = 100.0

    def sdk_open(self):
        sdk = load_sdk('arx_x5_sdk', self.label, extra='arx')
        kind = getattr(sdk.ArmType, self.options.get('arm_type', 'X5_2025'))
        self.gripper_open = float(self.options.get('gripper_open', -3.4))     # ARX's ROS 2 controller scale for X5 2025
        self.arm = sdk.ArxInterface(str(self.bus or 'can0'), kind)

    def sdk_read(self):
        q = list(self.arm.get_joint_positions())
        if len(q) < 6:
            raise ArmHardwareError(f'{len(q)} joint positions from the SDK')
        out = dict(positions_rad=q[:6], velocities_rad_s=list(self.arm.get_joint_velocities())[:6])
        if len(q) > 6:
            out['gripper'] = max(0.0, min(1.0, q[6] / self.gripper_open))
        return out

    def sdk_engage(self, q, jaw):
        self.sdk_write(q)

    def sdk_write(self, q):
        if self.arm.set_joint_positions(q) is False or self.arm.set_arm_status(JOINT_POSITION) is False:
            raise ArmHardwareError('the SDK refused the joint command')

    def sdk_jaw(self, jaw):
        self.arm.set_catch(jaw * self.gripper_open)

    def sdk_limp(self):
        self.arm.set_arm_status(GRAVITY_COMPENSATION)

    def sdk_close(self):
        arm = getattr(self, 'arm', None)
        if arm is not None:
            self.arm = None
            arm.set_arm_status(GRAVITY_COMPENSATION)

    def sdk_survey(self):
        raise NotImplementedError('ARX X5 has no read-only check: opening the SDK starts its control thread')
