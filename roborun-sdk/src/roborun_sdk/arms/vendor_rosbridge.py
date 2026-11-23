"""Arms whose public interface is ROS topics, reached through rosbridge with roslibpy.

ROS 1 clients need the robot's Python and message packages; rosbridge needs neither on this computer, works from any
Python and OS, and carries the same topics. Run rosbridge_server on the robot's ROS network and give ADDRESS as
host[:port] (default port 9090). This is not a real-time link: each arm's own controller does the servo loop, and its
command timeout or short trajectory ends motion when the stream stops.
"""
import threading
import time

from .base import ArmHardwareError
from .vendor_base import ContinuousJoints, VendorArm, host_port, load_sdk

JOINT_STATE_ROS1, JOINT_STATE_ROS2 = 'sensor_msgs/JointState', 'sensor_msgs/msg/JointState'


class RosbridgeArm(VendorArm):
    rate_hz = 50.0
    state_topic = '/joint_states'
    state_type = JOINT_STATE_ROS1
    stale_s = 0.5
    continuous = ()

    # -- each arm defines these ------------------------------------------------------------------------------------
    def joint_names(self):
        raise NotImplementedError

    def extra_subscriptions(self):
        """{topic: (type, handler)} read while connected."""
        return {}

    def advertise(self):
        """Create the command topics; only called when motors are engaged."""
        raise NotImplementedError

    def fault(self):
        return None

    # -- shared -------------------------------------------------------------------------------------------------------
    def _connect(self):
        self.rl = load_sdk('roslibpy', self.label, extra='rosbridge')
        host, port = host_port(self.bus, 9090)
        self.ros = self.rl.Ros(host=host, port=int(port))
        self.ros.run(timeout=5)
        if not self.ros.is_connected:
            raise ArmHardwareError(f'no rosbridge at {host}:{port}')
        self.lock = threading.Lock()
        self.names = self.joint_names()
        self.joints, self.joints_at, self.subs = {}, 0.0, []
        self._subscribe(self.state_topic, self.state_type, self._on_joints)
        for topic, (kind, handler) in self.extra_subscriptions().items():
            self._subscribe(topic, kind, handler)

    def _subscribe(self, topic, kind, handler):
        sub = self.rl.Topic(self.ros, topic, kind)
        sub.subscribe(handler)
        self.subs.append(sub)

    def _topic(self, topic, kind):
        out = self.rl.Topic(self.ros, topic, kind)
        out.advertise()
        self.pubs.append(out)
        return out

    def _send(self, topic, body):
        topic.publish(self.rl.Message(body))

    def _on_joints(self, msg):
        with self.lock:
            self.joints.update(zip(msg.get('name') or [], msg.get('position') or []))
            self.joints_at = time.monotonic()

    def _positions(self):
        with self.lock:
            if time.monotonic() - self.joints_at > self.stale_s:
                raise ArmHardwareError(f'{self.state_topic} stopped')
            missing = [n for n in self.names if n not in self.joints]
            if missing:
                raise ArmHardwareError(f'{self.state_topic} lacks {missing}')
            return [float(self.joints[n]) for n in self.names]

    def _wait(self, seconds=3.0):
        end = time.monotonic() + seconds
        while True:
            try:
                return self._positions()
            except ArmHardwareError:
                if time.monotonic() > end:
                    raise
                time.sleep(.01)

    def gripper_now(self):
        return None

    def sdk_open(self):
        self.pubs = []
        self.turns = ContinuousJoints(self.continuous)
        self._connect()
        self._wait()

    def sdk_read(self):
        if not self.ros.is_connected:
            raise ArmHardwareError('rosbridge connection lost')
        fault = self.fault()
        if fault:
            raise ArmHardwareError(fault)
        return dict(positions_rad=self.turns.measured(self._positions()), gripper=self.gripper_now())

    def sdk_engage(self, q, jaw):
        if not self.pubs:
            self.advertise()

    def sdk_limp(self):
        pass                                                         # stop streaming: the arm's controller takes over

    def sdk_close(self):
        ros = getattr(self, 'ros', None)
        if ros is not None:
            self.ros = None
            for end in self.pubs:
                end.unadvertise()
            for end in self.subs:
                end.unsubscribe()
            ros.close()

    def sdk_survey(self):
        self.pubs = []
        self.turns = ContinuousJoints(self.continuous)
        self._connect()                                              # subscriptions only: nothing is advertised
        try:
            return dict(positions_rad=self.turns.measured(self._wait()), fault=self.fault(), joint_names=self.names)
        finally:
            self.sdk_close()


class _RethinkArm(RosbridgeArm):
    """Sawyer and Baxter: POSITION_MODE joint commands, which keep the robot's collision avoidance on."""
    state_topic = '/robot/joint_states'
    has_gripper = False                  # the end-effector protocols differ per gripper and are not driven here
    assembly_type = command_type = None
    speed_ratio = 0.5

    def limb(self):
        return self.side

    def extra_subscriptions(self):
        self.assembly, self.assembly_at = None, 0.0
        return {'/robot/state': (self.assembly_type, self._on_assembly)}

    def _on_assembly(self, msg):
        self.assembly, self.assembly_at = msg, time.monotonic()

    def fault(self):
        a = self.assembly
        if a is None or time.monotonic() - self.assembly_at > 2.0:
            return '/robot/state is not arriving'
        if a.get('stopped') or a.get('error') or a.get('estop_button') == 1:
            return f'robot state: stopped={a.get("stopped")} error={a.get("error")} estop={a.get("estop_button")}'
        return None

    def _wait(self, seconds=3.0):
        out = super()._wait(seconds)
        end = time.monotonic() + seconds
        while self.assembly is None and time.monotonic() < end:
            time.sleep(.01)
        return out

    def advertise(self):
        ns = f'/robot/limb/{self.limb()}'
        self.enable = self._topic('/robot/set_super_enable', 'std_msgs/Bool')
        self.ratio = self._topic(f'{ns}/set_speed_ratio', 'std_msgs/Float64')
        self.cmd = self._topic(f'{ns}/joint_command', self.command_type)

    def sdk_engage(self, q, jaw):
        super().sdk_engage(q, jaw)
        self._send(self.enable, dict(data=True))
        self._send(self.ratio, dict(data=self.speed_ratio))


class SawyerArm(_RethinkArm):
    label = 'Rethink Sawyer'
    assembly_type = 'intera_core_msgs/RobotAssemblyState'
    command_type = 'intera_core_msgs/JointCommand'

    def limb(self):
        return 'right'                                               # Sawyer's one arm, whichever slot it fills

    def joint_names(self):
        return [f'right_j{i}' for i in range(7)]

    def sdk_write(self, q):
        self._send(self.cmd, dict(mode=1, names=self.names, position=q, velocity=[], acceleration=[], effort=[]))


class BaxterArm(_RethinkArm):
    label = 'Rethink Baxter arm'
    assembly_type = 'baxter_core_msgs/AssemblyState'
    command_type = 'baxter_core_msgs/JointCommand'

    def joint_names(self):
        return [f'{self.side}_{j}' for j in ('s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2')]

    def sdk_write(self, q):
        self._send(self.cmd, dict(mode=1, names=self.names, command=q))


class PR2Arm(RosbridgeArm):
    label = 'Willow Garage PR2 arm'
    rate_hz = 25.0
    continuous = (4, 6)                  # forearm roll and wrist roll turn without a stop
    gripper_open_m = 0.086
    horizon_s = 0.12

    @property
    def s(self):
        return self.side[0]

    def joint_names(self):
        return [f'{self.s}_{j}_joint' for j in ('shoulder_pan', 'shoulder_lift', 'upper_arm_roll', 'elbow_flex',
                                                'forearm_roll', 'wrist_flex', 'wrist_roll')]

    def extra_subscriptions(self):
        self.width, self.halted = None, False
        return {f'/{self.s}_gripper_controller/state': ('pr2_controllers_msgs/JointControllerState', self._on_gripper),
                '/pr2_ethercat/motors_halted': ('std_msgs/Bool', self._on_halted)}

    def _on_gripper(self, msg):
        self.width = float(msg['process_value'])

    def _on_halted(self, msg):
        self.halted = bool(msg['data'])

    def fault(self):
        return 'PR2 motors are halted' if self.halted else None

    def gripper_now(self):
        return None if self.width is None else max(0.0, min(1.0, self.width / self.gripper_open_m))

    def advertise(self):
        self.cmd = self._topic(f'/{self.s}_arm_controller/command', 'trajectory_msgs/JointTrajectory')
        self.grip = self._topic(f'/{self.s}_gripper_controller/command', 'pr2_controllers_msgs/Pr2GripperCommand')

    def sdk_write(self, q):
        point = dict(positions=self.turns.command(q), velocities=[], accelerations=[],
                     time_from_start=dict(secs=0, nsecs=int(self.horizon_s * 1e9)))
        self._send(self.cmd, dict(joint_names=self.names, points=[point]))

    def sdk_jaw(self, jaw):
        self._send(self.grip, dict(position=jaw * self.gripper_open_m, max_effort=float(self.options.get('grip_effort', 30.0))))


class GalaxeaR1LiteArm(RosbridgeArm):
    """Needs Galaxea's HDAS driver and mobiman joint tracker running (r1_lite_jointTrackerdemo)."""
    label = 'Galaxea R1 Lite arm'
    stroke_mm = 100.0

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        if self.options.get('ros2'):
            self.state_type = JOINT_STATE_ROS2
        self.state_topic = f'/hdas/feedback_arm_{self.side}'

    def joint_names(self):
        return [f'{self.side}_arm_joint{i}' for i in range(1, 7)]

    def extra_subscriptions(self):
        self.stroke = None
        return {f'/hdas/feedback_gripper_{self.side}': (self.state_type, self._on_gripper)}

    def _on_gripper(self, msg):
        if msg.get('position'):
            self.stroke = float(msg['position'][0])

    def _on_joints(self, msg):                                       # this topic's order is documented, names are not
        position = msg.get('position') or []
        with self.lock:
            self.joints.update(zip(self.names, position[:len(self.names)]))
            self.joints_at = time.monotonic()

    def gripper_now(self):
        return None if self.stroke is None else max(0.0, min(1.0, self.stroke / self.stroke_mm))

    def advertise(self):
        float_type = 'std_msgs/msg/Float32' if self.options.get('ros2') else 'std_msgs/Float32'
        self.cmd = self._topic(f'/motion_target/target_joint_state_arm_{self.side}', self.state_type)
        self.grip = self._topic(f'/motion_control/position_control_gripper_{self.side}', float_type)

    def sdk_write(self, q):
        self._send(self.cmd, dict(position=q, velocity=[self.joint_speed_rad_s * 2] * self.dof))

    def sdk_jaw(self, jaw):
        self._send(self.grip, dict(data=jaw * self.stroke_mm))


class RobotisAIWorkerArm(RosbridgeArm):
    """ROBOTIS AI Worker (FFW follower, ROS 2): the arm and its gripper joint share one trajectory controller."""
    label = 'ROBOTIS AI Worker arm'
    state_type = JOINT_STATE_ROS2
    gripper_closed = 1.1                 # rad of gripper_*_joint1: 0 is open
    horizon_s = 0.06

    @property
    def s(self):
        return self.side[0]

    def joint_names(self):
        return [f'arm_{self.s}_joint{i}' for i in range(1, 8)]

    def gripper_now(self):
        with self.lock:
            value = self.joints.get(f'gripper_{self.s}_joint1')
        return None if value is None else max(0.0, min(1.0, 1.0 - float(value) / self.gripper_closed))

    def advertise(self):
        # the bringup launch remaps each arm controller's joint_trajectory to this topic
        topic = self.options.get('command_topic', '/leader/joint_trajectory_command_broadcaster_{side}/joint_trajectory')
        self.cmd = self._topic(topic.format(side=self.side), 'trajectory_msgs/msg/JointTrajectory')
        self.jaw_now = None

    def sdk_engage(self, q, jaw):
        super().sdk_engage(q, jaw)
        self.jaw_now = jaw

    def sdk_jaw(self, jaw):
        self.jaw_now = jaw

    def sdk_write(self, q):
        names = self.names + [f'gripper_{self.s}_joint1']
        point = dict(positions=list(q) + [(1.0 - self.jaw_now) * self.gripper_closed], velocities=[], accelerations=[],
                     time_from_start=dict(sec=0, nanosec=int(self.horizon_s * 1e9)))
        self._send(self.cmd, dict(joint_names=names, points=[point]))
