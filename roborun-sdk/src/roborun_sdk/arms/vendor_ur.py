"""Universal Robots e-Series through ur_rtde (RTDE over Ethernet). Stream: servoJ.

The robot must already be powered on with brakes released and, on e-Series, in Remote Control: Roborun does not do that
for you. The arm has no built-in gripper, so none is reported.
"""
from .base import ArmHardwareError
from .vendor_base import VendorArm, load_sdk

ROBOT_RUNNING = 7
SAFETY_OK = (1, 2)                       # normal, reduced


class URArm(VendorArm):
    label = 'Universal Robots UR5e'
    rate_hz = 125.0
    has_gripper = False
    lookahead_s = 0.1
    gain = 300

    def _receive(self):
        return load_sdk('rtde_receive', self.label, extra='ur').RTDEReceiveInterface(str(self.bus))

    def sdk_open(self):
        if not self.bus:
            raise ValueError('UR arms need an address: ur5e@ROBOT_IP')
        self.control_sdk = load_sdk('rtde_control', self.label, extra='ur')
        self.rx = self._receive()
        mode = self.rx.getRobotMode()
        if mode != ROBOT_RUNNING:
            raise ArmHardwareError(f'robot mode {mode}: power it on and release the brakes on the teach pendant first')
        self.ctl = None

    def sdk_read(self):
        if not self.rx.isConnected():
            raise ArmHardwareError('RTDE connection lost')
        if self.rx.isProtectiveStopped() or self.rx.isEmergencyStopped():
            raise ArmHardwareError('protective or emergency stop')
        safety = self.rx.getSafetyMode()
        if safety not in SAFETY_OK:
            raise ArmHardwareError(f'safety mode {safety}')
        if self.ctl is not None and not self.ctl.isProgramRunning():
            raise ArmHardwareError('the RTDE control script stopped')
        return dict(positions_rad=list(self.rx.getActualQ()), velocities_rad_s=list(self.rx.getActualQd()),
                    temperatures_c=list(self.rx.getJointTemperatures()))

    def sdk_engage(self, q, jaw):
        if self.ctl is None:
            self.ctl = self.control_sdk.RTDEControlInterface(str(self.bus))     # uploads the control script; no motion

    def sdk_write(self, q):
        # speed and acceleration are not used by servoJ; the time, lookahead and gain shape the motion
        if not self.ctl.servoJ(q, 0.5, 0.5, 1.0 / self.rate_hz, self.lookahead_s, self.gain):
            raise ArmHardwareError('servoJ was refused')

    def sdk_limp(self):
        self.ctl.servoStop()                                         # decelerates and holds; UR arms do not go limp here

    def sdk_close(self):
        ctl, rx = getattr(self, 'ctl', None), getattr(self, 'rx', None)
        self.ctl = self.rx = None
        if ctl is not None:
            ctl.servoStop()
            ctl.stopScript()
            ctl.disconnect()
        if rx is not None:
            rx.disconnect()

    def sdk_survey(self):
        if not self.bus:
            raise ValueError('UR arms need an address: ur5e@ROBOT_IP')
        rx = self._receive()                                         # the receive interface cannot command anything
        try:
            return dict(positions_rad=list(rx.getActualQ()), robot_mode=rx.getRobotMode(), safety_mode=rx.getSafetyMode(),
                        protective_stop=rx.isProtectiveStopped(), emergency_stop=rx.isEmergencyStopped())
        finally:
            rx.disconnect()
