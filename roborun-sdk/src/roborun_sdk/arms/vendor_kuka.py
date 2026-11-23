"""KUKA LBR iiwa 14 through pyFRI (Fast Robot Interface over UDP).

The robot side is yours to provide: the Sunrise FRI option and a running joint-position overlay application with this
computer as the FRI partner. KUKA drives the exchange: every cycle (use a 5 ms send period) the client must answer at
once, so a dedicated thread steps FRI and interpolates between stream targets. pyFRI is built from source.
"""
import threading
import time

from .base import ArmHardwareError
from .vendor_base import VendorArm, host_port, load_sdk

HINT = 'build pyFRI from https://github.com/lbr-stack/pyFRI (needs FRI_CLIENT_VERSION matching your Sunrise FRI)'
DEFAULT_PORT = 30200


class KukaIiwa(VendorArm):
    label = 'KUKA LBR iiwa 14'
    rate_hz = 200.0
    has_gripper = False

    def _start(self):
        fri = load_sdk('pyfri', self.label, hint=HINT)
        np = load_sdk('numpy', self.label)
        arm = self

        class Client(fri.LBRClient):
            def monitor(self):
                arm._seen(self.robotState())

            def onStateChange(self, old, new):
                arm.session = new

            def waitForCommand(self):
                state = self.robotState()
                arm._seen(state)
                self.robotCommand().setJointPosition(state.getIpoJointPosition().astype(np.float32))

            def command(self):
                state = self.robotState()
                arm._seen(state)
                target = arm._blend()
                if target is None:
                    target = state.getIpoJointPosition()
                self.robotCommand().setJointPosition(np.asarray(target, dtype=np.float32))

        self.fri, self.session = fri, None
        self.latest, self.latest_at, self.step_error = None, 0.0, None
        self.client = Client()
        self.app = fri.ClientApplication(self.client)
        host, port = host_port(self.bus, DEFAULT_PORT, default_host='')
        if not self.app.connect(int(port or DEFAULT_PORT), host or None):
            raise ArmHardwareError('FRI connection failed')
        self.stepping = True
        self.stepper = threading.Thread(target=self._step_loop, daemon=True, name=f'fri-{self.side}')
        self.stepper.start()

    def _seen(self, state):
        self.latest, self.latest_at = [float(v) for v in state.getMeasuredJointPosition()], time.monotonic()

    def _blend(self):
        with self.target_lock:
            if self.target is None:
                return None
            start, goal, at = self.previous, self.target, self.target_at
        share = min(1.0, (time.monotonic() - at) * self.rate_hz)
        return [a + (b - a) * share for a, b in zip(start, goal)]

    def _step_loop(self):
        try:
            while self.stepping:
                if not self.app.step():
                    self.step_error = 'FRI connection closed'
                    return
        except Exception as exc:
            self.step_error = f'{type(exc).__name__}: {exc}'

    def _begin(self):
        self.target_lock = threading.Lock()
        self.previous, self.target, self.target_at = None, None, 0.0
        self._start()

    def _wait_for_sample(self, seconds=5.0):
        end = time.monotonic() + seconds
        while self.latest is None:
            if self.step_error or time.monotonic() > end:
                raise ArmHardwareError(self.step_error or 'no FRI data: is the robot application running?')
            time.sleep(.005)

    def sdk_open(self):
        self._begin()
        self._wait_for_sample()

    def sdk_read(self):
        if self.step_error:
            raise ArmHardwareError(self.step_error)
        if time.monotonic() - self.latest_at > 0.1:
            raise ArmHardwareError('FRI data stopped')
        if self.target is not None and not self._commanding():
            raise ArmHardwareError(f'FRI session left commanding: {self.session}')
        return dict(positions_rad=list(self.latest))

    def _commanding(self):
        states = self.fri.ESessionState
        return self.session in (states.COMMANDING_WAIT, states.COMMANDING_ACTIVE)

    def sdk_engage(self, q, jaw):
        if not self._commanding():
            raise ArmHardwareError(f'FRI session is {self.session}: start the robot application\'s joint-position overlay')
        with self.target_lock:
            self.previous, self.target, self.target_at = list(q), list(q), time.monotonic()

    def sdk_write(self, q):
        with self.target_lock:
            now = time.monotonic()
            share = min(1.0, (now - self.target_at) * self.rate_hz)
            self.previous = [a + (b - a) * share for a, b in zip(self.previous, self.target)]
            self.target, self.target_at = list(q), now

    def sdk_limp(self):
        with self.target_lock:                                       # back to following the robot's own interpolator
            self.previous, self.target = None, None

    def sdk_close(self):
        app = getattr(self, 'app', None)
        if app is not None:
            self.app, self.stepping = None, False
            self.stepper.join(timeout=1)
            app.disconnect()

    def sdk_survey(self):
        self._begin()                                                # monitoring only: no target is ever set
        try:
            self._wait_for_sample()
            state = self.client.robotState()
            return dict(positions_rad=list(self.latest), session=str(self.session), sample_time=state.getSampleTime())
        finally:
            self.sdk_close()
