"""Franka FR3 / Panda through franky (libfranka). Linux with a real-time kernel, wired to the robot's control box.

franky runs the 1 kHz FCI loop in C++ with an online trajectory generator. Each stream cycle replaces its target with a
non-finishing JointMotion, so Python never has to meet the 1 kHz deadline. The robot's FCI must be activated in Desk and
its joints unlocked first.
"""
from .base import ArmHardwareError
from .vendor_base import VendorArm, load_sdk


class FrankaArm(VendorArm):
    label = 'Franka'
    rate_hz = 50.0
    jaw_every = 10
    jaw_deadband = 0.02
    gripper_speed = 0.05                 # m/s

    def _sdk(self):
        return load_sdk('franky', self.label, extra='franka')

    def _host(self):
        if not self.bus:
            raise ValueError('Franka arms need an address: franka_fr3@ROBOT_IP')
        return str(self.bus)

    def sdk_open(self):
        self.sdk = self._sdk()
        self.robot = self.sdk.Robot(self._host(), relative_dynamics_factor=float(self.options.get('dynamics', 0.2)))
        self.gripper, self.pending = None, None
        self.jaw_seen, self.jaw_read = None, 0
        if self.options.get('gripper', True):
            try:
                self.gripper = self.sdk.Gripper(self._host())
                self.max_width = float(self.gripper.max_width) or 0.08
            except Exception:
                self.gripper = None
        self.has_gripper = self.gripper is not None

    def sdk_read(self):
        if self.robot.has_errors:
            raise ArmHardwareError(f'robot errors: {self.robot.state.current_errors}')
        self.robot.poll_motion()                                     # raises what the control loop caught
        state = self.robot.state
        out = dict(positions_rad=[float(v) for v in state.q], velocities_rad_s=[float(v) for v in state.dq])
        if self.gripper is not None:
            self.jaw_read -= 1
            if self.jaw_read <= 0 or self.jaw_seen is None:
                self.jaw_read = 25
                self.jaw_seen = max(0.0, min(1.0, float(self.gripper.width) / self.max_width))
            out['gripper'] = self.jaw_seen
        return out

    def sdk_engage(self, q, jaw):
        pass                                                         # the first JointMotion takes control

    def sdk_write(self, q):
        self.robot.move(self.sdk.JointMotion(q, return_when_finished=False), asynchronous=True)

    def sdk_jaw(self, jaw):
        if self.pending is not None and not self.pending.wait(0):
            return False                                             # one gripper move at a time
        self.pending = self.gripper.move_async(jaw * self.max_width, self.gripper_speed)

    def sdk_limp(self):
        self.robot.stop()                                            # ends the motion; the arm holds under its own control

    def sdk_close(self):
        robot = getattr(self, 'robot', None)
        if robot is not None:
            self.robot = None
            robot.stop()

    def sdk_survey(self):
        sdk = self._sdk()
        robot = sdk.Robot(self._host(), realtime_config=sdk.RealtimeConfig.Ignore)      # reading needs no real-time host
        state = robot.state
        return dict(positions_rad=[float(v) for v in state.q], robot_mode=str(state.robot_mode),
                    errors=str(state.current_errors), has_errors=bool(robot.has_errors))
