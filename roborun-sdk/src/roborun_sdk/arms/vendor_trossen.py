"""Trossen WidowX AI through trossen_arm (Ethernet, TCP/UDP to the arm's controller).

Stream: position mode with short non-blocking goals; the driver interpolates linearly over each goal_time.
"""
import math

from .base import ArmHardwareError
from .vendor_base import VendorArm, load_sdk


class TrossenArm(VendorArm):
    label = 'Trossen WidowX AI'
    gripper_travel_m = 0.04

    def _sdk(self):
        return load_sdk('trossen_arm', self.label, extra='trossen')

    def _configure(self, sdk):
        driver = sdk.TrossenArmDriver()
        effector = getattr(sdk.StandardEndEffector, self.options.get('end_effector', 'wxai_v0_base'))
        driver.configure(sdk.Model.wxai_v0, effector, str(self.bus or '192.168.1.2'), False)     # never clears a fault
        return driver

    @staticmethod
    def _fault(driver):
        text = str(driver.get_error_information() or '').strip()
        return None if not text or 'no error' in text.lower() else text

    def sdk_open(self):
        self.sdk = self._sdk()
        self.driver = self._configure(self.sdk)

    def sdk_read(self):
        fault = self._fault(self.driver)
        if fault:
            raise ArmHardwareError(fault)
        q, v = list(self.driver.get_all_positions()), list(self.driver.get_all_velocities())
        return dict(positions_rad=q[:6], velocities_rad_s=v[:6], gripper=max(0.0, min(1.0, q[6] / self.gripper_travel_m)))

    def sdk_engage(self, q, jaw):
        self.driver.set_all_modes(self.sdk.Mode.position)

    def sdk_write(self, q):
        self.driver.set_arm_positions(q, goal_time=1.0 / self.rate_hz, blocking=False)

    def sdk_jaw(self, jaw):
        self.driver.set_gripper_position(jaw * self.gripper_travel_m, goal_time=1.0 / self.rate_hz, blocking=False)

    def sdk_limp(self):
        self.driver.set_all_modes(self.sdk.Mode.idle)                # the maker's idle: joints braked

    def sdk_close(self):
        driver = getattr(self, 'driver', None)
        if driver is not None:
            self.driver = None
            driver.set_all_modes(self.sdk.Mode.idle)
            driver.cleanup()

    def sdk_survey(self):
        sdk = self._sdk()
        driver = self._configure(sdk)
        try:
            q = list(driver.get_all_positions())
            return dict(positions_rad=q[:6], gripper_m=q[6], fault=self._fault(driver),
                        controller=driver.get_controller_version(), joints_reported=len(q) - 1,
                        degrees=[round(math.degrees(v), 2) for v in q[:6]])
        finally:
            driver.cleanup()
