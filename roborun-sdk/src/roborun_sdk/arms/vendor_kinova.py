"""Kinova Gen3 (7 joints) through kortex_api. Stream: low-level servoing, BaseCyclic.Refresh over UDP port 10001.

Kinova ships kortex_api as a wheel from its own artifactory, not PyPI. Joints 1, 3, 5, 7 turn without a stop; the arm
reports every joint in 0..360 degrees. Leaving low-level servoing returns the arm to its own holding controller.
"""
import math

from .base import ArmHardwareError
from .vendor_base import ContinuousJoints, VendorArm, host_port, load_sdk

HINT = ('install the kortex_api wheel from Kinova\'s artifactory: '
        'https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl')
TCP_PORT, UDP_PORT = 10000, 10001


class _Link:
    def __init__(self, label, host, port, user, password):
        transport = load_sdk('kortex_api.TCPTransport' if port == TCP_PORT else 'kortex_api.UDPTransport', label, hint=HINT)
        router = load_sdk('kortex_api.RouterClient', label, hint=HINT)
        session = load_sdk('kortex_api.SessionManager', label, hint=HINT)
        messages = load_sdk('kortex_api.autogen.messages.Session_pb2', label, hint=HINT)
        self.transport = transport.TCPTransport() if port == TCP_PORT else transport.UDPTransport()
        self.router = router.RouterClient(self.transport, lambda exc: None)
        self.transport.connect(host, port)
        info = messages.CreateSessionInfo()
        info.username, info.password = user, password
        info.session_inactivity_timeout, info.connection_inactivity_timeout = 10000, 2000
        self.session = session.SessionManager(self.router)
        self.session.CreateSession(info)

    def close(self):
        try:
            self.session.CloseSession()
        finally:
            self.transport.disconnect()


class KinovaGen3(VendorArm):
    label = 'Kinova Gen3'
    rate_hz = 200.0
    continuous = (0, 2, 4, 6)

    def _links(self, udp):
        host, _ = host_port(self.bus, default_host='192.168.1.10')
        user, password = self.options.get('username', 'admin'), self.options.get('password', 'admin')
        return _Link(self.label, host, TCP_PORT, user, password), _Link(self.label, host, UDP_PORT, user, password) if udp else None

    def _clients(self, tcp, udp):
        base = load_sdk('kortex_api.autogen.client_stubs.BaseClientRpc', self.label, hint=HINT).BaseClient(tcp.router)
        cyclic = load_sdk('kortex_api.autogen.client_stubs.BaseCyclicClientRpc', self.label, hint=HINT)
        return base, cyclic.BaseCyclicClient((udp or tcp).router)

    def sdk_open(self):
        self.tcp, self.udp = self._links(True)
        self.base, self.cyclic = self._clients(self.tcp, self.udp)
        self.pb_base = load_sdk('kortex_api.autogen.messages.Base_pb2', self.label, hint=HINT)
        self.pb_cyclic = load_sdk('kortex_api.autogen.messages.BaseCyclic_pb2', self.label, hint=HINT)
        router = load_sdk('kortex_api.RouterClient', self.label, hint=HINT)
        self.send_options = router.RouterClientSendOptions()
        self.send_options.andForget, self.send_options.delay_ms, self.send_options.timeout_ms = False, 0, 10
        self.turns = ContinuousJoints(self.continuous)
        self.feedback = self.cyclic.RefreshFeedback()
        if len(self.feedback.actuators) != self.dof:
            raise ArmHardwareError(f'{len(self.feedback.actuators)} actuators on this arm, expected {self.dof}')
        self.has_gripper = len(self.feedback.interconnect.gripper_feedback.motor) > 0
        self.cmd = None

    @staticmethod
    def _faults(feedback):
        banks = [feedback.base.fault_bank_a, feedback.base.fault_bank_b]
        for a in feedback.actuators:
            banks += [a.fault_bank_a, a.fault_bank_b]
        return [b for b in banks if b]

    def _positions(self, feedback):
        raw = [math.radians(a.position) for a in feedback.actuators]
        for i, v in enumerate(raw):                                  # 0..360 degrees -> the catalog's signed range
            if i not in self.continuous and v > math.pi:
                raw[i] = v - 2 * math.pi
        return self.turns.measured(raw)

    def sdk_read(self):
        feedback = self.feedback if self._engaged else self.cyclic.RefreshFeedback()
        self.feedback = feedback
        faults = self._faults(feedback)
        if faults:
            raise ArmHardwareError(f'actuator or base fault banks {faults}')
        out = dict(positions_rad=self._positions(feedback), velocities_rad_s=[math.radians(a.velocity) for a in feedback.actuators],
                   temperatures_c=[a.temperature_motor for a in feedback.actuators])
        if self.has_gripper:
            out['gripper'] = max(0.0, min(1.0, 1.0 - feedback.interconnect.gripper_feedback.motor[0].position / 100.0))
        return out

    def sdk_engage(self, q, jaw):
        feedback = self.cyclic.RefreshFeedback()
        cmd = self.pb_cyclic.Command()
        for a in feedback.actuators:
            row = cmd.actuators.add()
            row.flags, row.position, row.velocity, row.command_id = 1, a.position, 0.0, 0
        if self.has_gripper:
            self.grip = cmd.interconnect.gripper_command.motor_cmd.add()
            self.grip.position = feedback.interconnect.gripper_feedback.motor[0].position
            self.grip.velocity, self.grip.force = 0.0, 100.0
        self.cmd = cmd
        mode = self.pb_base.ServoingModeInformation()
        mode.servoing_mode = self.pb_base.LOW_LEVEL_SERVOING
        self.base.SetServoingMode(mode)

    def sdk_write(self, q):
        cmd = self.cmd
        cmd.frame_id = (cmd.frame_id + 1) & 0xffff
        for row, value in zip(cmd.actuators, self.turns.command(q)):
            row.position = math.degrees(value) % 360.0
            row.command_id = cmd.frame_id
        self.feedback = self.cyclic.Refresh(cmd, 0, self.send_options)

    def sdk_jaw(self, jaw):
        self.grip.position = (1.0 - jaw) * 100.0
        self.grip.velocity = 50.0

    def _single_level(self):
        mode = self.pb_base.ServoingModeInformation()
        mode.servoing_mode = self.pb_base.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(mode)

    def sdk_limp(self):
        self._single_level()

    def sdk_close(self):
        tcp, udp = getattr(self, 'tcp', None), getattr(self, 'udp', None)
        self.tcp = self.udp = None
        try:
            if tcp is not None and self.cmd is not None:
                self._single_level()
        finally:
            for link in (udp, tcp):
                if link is not None:
                    link.close()

    def sdk_survey(self):
        tcp, _ = self._links(False)
        try:
            _, cyclic = self._clients(tcp, None)
            feedback = cyclic.RefreshFeedback()
            self.turns = ContinuousJoints(self.continuous)
            return dict(positions_rad=self._positions(feedback), actuators=len(feedback.actuators),
                        faults=self._faults(feedback), active_state=feedback.base.active_state)
        finally:
            tcp.close()
