"""UFACTORY xArm 7 through xarm-python-sdk (Ethernet). Stream: servo mode 1, set_servo_angle_j, last command wins."""
from .base import ArmHardwareError
from .vendor_base import VendorArm, check_code, load_sdk

SERVO_MODE, TEACH_MODE = 1, 2
GRIPPER_OPEN_PULSES = 850.0


class XArm7(VendorArm):
    label = 'UFACTORY xArm 7'
    jaw_every = 10                       # the gripper sits on the tool RS-485 link
    jaw_deadband = 0.01

    def _api(self):
        sdk = load_sdk('xarm.wrapper', self.label, extra='xarm')
        return sdk.XArmAPI(str(self.bus or '192.168.1.185'), is_radian=True)

    def _angles(self, api):
        code, q = api.get_servo_angle(is_radian=True)
        check_code(code, 'get_servo_angle')
        return [float(v) for v in q[:7]]

    def sdk_open(self):
        self.api = self._api()
        if not self.api.connected:
            raise ArmHardwareError('no connection to the controller')
        if self.api.axis != 7:
            raise ArmHardwareError(f'this controller reports {self.api.axis} axes, not 7')
        self.has_gripper = bool(self.options.get('gripper', True))
        self._jaw_seen, self._jaw_read = 1.0, 0

    def sdk_read(self):
        if self.api.error_code:
            raise ArmHardwareError(f'controller error {self.api.error_code}')
        q = self._angles(self.api)
        out = dict(positions_rad=q)
        if self.has_gripper:
            self._jaw_read += 1
            if self._jaw_read >= self.jaw_every:
                self._jaw_read = 0
                code, pulses = self.api.get_gripper_position()
                if code == 0 and pulses is not None:
                    self._jaw_seen = max(0.0, min(1.0, float(pulses) / GRIPPER_OPEN_PULSES))
            out['gripper'] = self._jaw_seen
        return out

    def _servo_mode(self):
        check_code(self.api.motion_enable(True), 'motion_enable')
        check_code(self.api.set_mode(SERVO_MODE), 'set_mode')
        check_code(self.api.set_state(0), 'set_state')

    def sdk_engage(self, q, jaw):
        self._servo_mode()
        if self.has_gripper:
            check_code(self.api.set_gripper_mode(0), 'set_gripper_mode')
            check_code(self.api.set_gripper_enable(True), 'set_gripper_enable')

    def sdk_write(self, q):
        check_code(self.api.set_servo_angle_j(q, is_radian=True), 'set_servo_angle_j')

    def sdk_jaw(self, jaw):
        check_code(self.api.set_gripper_position(jaw * GRIPPER_OPEN_PULSES, wait=False), 'set_gripper_position')

    def sdk_limp(self):
        check_code(self.api.set_mode(TEACH_MODE), 'set_mode')        # joint teach: gravity compensated, moves by hand
        check_code(self.api.set_state(0), 'set_state')

    def sdk_close(self):
        api = getattr(self, 'api', None)
        if api is not None:
            self.api = None
            api.set_state(4)                                         # stop state: holds, ignores further motion
            api.disconnect()

    def sdk_survey(self):
        api = self._api()
        try:
            return dict(positions_rad=self._angles(api), axis=api.axis, mode=api.mode, state=api.state,
                        error_code=api.error_code, warn_code=api.warn_code, version=api.version_number)
        finally:
            api.disconnect()
