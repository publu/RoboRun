"""Our use of each maker's SDK, against small stand-ins written from that SDK's documented API.

These tests prove that we call the documented API in a safe order: nothing moves at connect, the maker's streaming mode is
used, limits and joint counts hold, faults latch, and the read-only check stays read-only. They do NOT prove that any arm
moves correctly: no real arm or vendor simulator is involved. test_real_sdk_has_what_we_call compares our calls with the
real packages when those are installed.
"""
import importlib
import inspect
import math
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS

import pytest

from pathlib import Path

STATION_SRC = Path(__file__).resolve().parents[2] / "roborun-station/src"     # the station tests here, when this file runs alone
if STATION_SRC.is_dir() and str(STATION_SRC) not in sys.path:
    sys.path.insert(0, str(STATION_SRC))

from roborun_sdk.arms import catalog, open_arm
from roborun_sdk.arms.base import ArmHardwareError
from roborun_sdk.arms import vendor_unitree


def wrap(a): return math.atan2(math.sin(a), math.cos(a))


def wait(check, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if check(): return True
        time.sleep(.005)
    return bool(check())


class Rec:
    """What the stand-in SDK saw."""
    def __init__(self, dof):
        self.q, self.dof, self.moves, self.changes, self.bad = [0.0] * dof, dof, [], [], []
        self.streaming = self.fault = self.surveying = self.limped = False
        self.jaw = []
        self.lock = threading.Lock()

    def change(self, what):              # anything that is more than reading
        self.changes.append(what)

    def move(self, q):
        q = [float(v) for v in q]
        if not self.streaming: self.bad.append('motion outside the streaming mode')
        if len(q) != self.dof: self.bad.append(f'{len(q)} joints')
        with self.lock: self.moves.append(q)

    @property
    def last(self):
        with self.lock: return self.moves[-1]


@pytest.fixture(autouse=True)
def _vendor_sdks_are_the_fakes(monkeypatch):
    """The makers' SDKs here are fake modules: nothing to install (open_arm would otherwise pip-install the real one, and an
    offline run fails on the download, not on the driver under test)."""
    monkeypatch.setattr('roborun_sdk.arms.packages.ensure', lambda extra, label=None: None)


def module(monkeypatch, name, **attrs):
    mod = ModuleType(name)
    mod.__dict__.update(attrs)
    monkeypatch.setitem(sys.modules, name, mod)
    if '.' in name:
        parent, _, leaf = name.rpartition('.')
        if parent not in sys.modules: module(monkeypatch, parent)
        setattr(sys.modules[parent], leaf, mod)
    return mod


# ---- stand-ins, one per SDK --------------------------------------------------------------------------------------------
def trossen(mp, rec):
    Mode = NS(idle='idle', position='position')

    class TrossenArmDriver:
        def configure(self, model, effector, ip, clear_error, timeout=20.0):
            if clear_error: rec.bad.append('cleared a fault')
        def get_error_information(self): return 'joint 2 overheated' if rec.fault else 'No error'
        def get_all_positions(self): return rec.q + [.02]
        def get_all_velocities(self): return [0.0] * 7
        def get_controller_version(self): return '1.9'
        def set_all_modes(self, mode):
            rec.change(mode); rec.streaming = mode == Mode.position; rec.limped = mode == Mode.idle
        def set_arm_positions(self, q, goal_time=2.0, blocking=True):
            if blocking or not 0.001 < goal_time <= .05: rec.bad.append('not a streaming goal')
            rec.move(q)
        def set_gripper_position(self, m, goal_time=2.0, blocking=True): rec.jaw.append(m)
        def cleanup(self, reboot_controller=False): pass
    module(mp, 'trossen_arm', TrossenArmDriver=TrossenArmDriver, Mode=Mode, Model=NS(wxai_v0=0),
           StandardEndEffector=NS(wxai_v0_base=1))


def piper(mp, rec):
    k = 180000 / math.pi

    class C_PiperInterface_V2:
        def __init__(self, can): pass
        def ConnectPort(self): pass
        def DisconnectPort(self): pass
        def isOk(self): return True
        def GetArmJointMsgs(self):
            return NS(time_stamp=1.0, joint_state=NS(**{f'joint_{i + 1}': round(v * k) for i, v in enumerate(rec.q)}))
        def GetArmStatus(self): return NS(arm_status=NS(arm_status=7 if rec.fault else 0, ctrl_mode=1))
        def GetArmGripperMsgs(self): return NS(gripper_state=NS(grippers_angle=35000))
        def GetPiperFirmwareVersion(self): return 'S-V1.6'
        def GetArmEnableStatus(self): return [False] * 6
        def EnablePiper(self): rec.change('enable'); return True
        def DisableArm(self, n): rec.change('disable'); rec.streaming = False; rec.limped = True
        def MotionCtrl_2(self, ctrl, move, speed, mit=0):
            rec.change('mode'); rec.streaming = (ctrl, move, mit) == (1, 1, 0)
        def JointCtrl(self, *j):
            if not all(isinstance(v, int) for v in j): rec.bad.append('joint targets must be integers')
            rec.move([v / k for v in j])
        def GripperCtrl(self, angle, effort, code, zero): rec.jaw.append(angle)
    module(mp, 'piper_sdk', C_PiperInterface_V2=C_PiperInterface_V2)


def arx(mp, rec):
    class ArxInterface:
        def __init__(self, can, kind): self.want = None
        def get_joint_positions(self):
            if rec.fault: raise OSError('CAN down')
            return rec.q + [-1.7]
        def get_joint_velocities(self): return [0.0] * 7
        def set_joint_positions(self, q): self.want = list(q); return True
        def set_catch(self, v): rec.jaw.append(v); return True
        def set_arm_status(self, s):
            rec.change(s); rec.streaming = s == 5; rec.limped = s == 3
            if s == 5 and self.want is not None: rec.move(self.want)
            return True
    module(mp, 'arx_x5_sdk', ArxInterface=ArxInterface, ArmType=NS(X5_2025=2))


def xarm(mp, rec):
    class XArmAPI:
        connected, axis, warn_code, version_number = True, 7, 0, '2.5'
        def __init__(self, ip, is_radian=False):
            self.mode, self.state = 0, 3
            if not is_radian: rec.bad.append('degrees')
        error_code = property(lambda self: 31 if rec.fault else 0)
        def get_servo_angle(self, is_radian=None): return 0, rec.q + [0.0]
        def get_gripper_position(self): return 0, 425.0
        def motion_enable(self, enable=True): rec.change('enable'); return 0
        def set_mode(self, mode):
            rec.change(f'mode{mode}'); self.mode = mode; rec.limped = mode == 2; return 0
        def set_state(self, state):
            rec.change(f'state{state}'); self.state = state; rec.streaming = self.mode == 1 and state == 0; return 0
        def set_gripper_mode(self, m): rec.change('gmode'); return 0
        def set_gripper_enable(self, on): rec.change('genable'); return 0
        def set_gripper_position(self, pos, wait=False):
            if wait: rec.bad.append('blocking gripper call')
            rec.jaw.append(pos); return 0
        def set_servo_angle_j(self, q, is_radian=None): rec.move(q); return 0
        def disconnect(self): pass
    module(mp, 'xarm.wrapper', XArmAPI=XArmAPI)


def ur(mp, rec):
    class RTDEReceiveInterface:
        def __init__(self, host): pass
        def getRobotMode(self): return rec.__dict__.get('robot_mode', 7)
        def getSafetyMode(self): return 1
        def isConnected(self): return True
        def isProtectiveStopped(self): return rec.fault
        def isEmergencyStopped(self): return False
        def getActualQ(self): return list(rec.q)
        def getActualQd(self): return [0.0] * 6
        def getJointTemperatures(self): return [30.0] * 6
        def disconnect(self): pass

    class RTDEControlInterface:
        def __init__(self, host): rec.change('control script'); rec.streaming = True
        def isProgramRunning(self): return True
        def servoJ(self, q, v, a, dt, lookahead, gain):
            if not (.002 <= dt <= .02 and .03 <= lookahead <= .2 and 100 <= gain <= 2000): rec.bad.append('servoJ range')
            rec.move(q); return True
        def servoStop(self, a=10.0): rec.limped = True; return True
        def stopScript(self): pass
        def disconnect(self): pass
    module(mp, 'rtde_receive', RTDEReceiveInterface=RTDEReceiveInterface)
    module(mp, 'rtde_control', RTDEControlInterface=RTDEControlInterface)


def franky(mp, rec):
    class JointMotion:
        def __init__(self, target, return_when_finished=True):
            if return_when_finished: rec.bad.append('a finishing motion would stop between targets')
            self.target = target

    class Robot:
        def __init__(self, host, relative_dynamics_factor=1.0, realtime_config=None): pass
        has_errors = property(lambda self: rec.fault)
        state = property(lambda self: NS(q=list(rec.q), dq=[0.0] * 7, robot_mode='Idle', current_errors='none'))
        def poll_motion(self): return True
        def move(self, motion, asynchronous=False):
            if not asynchronous: rec.bad.append('blocking move')
            rec.streaming = True; rec.move(motion.target)
        def stop(self): rec.limped = True; rec.streaming = False

    class Gripper:
        max_width, width = .08, .04
        def __init__(self, host): pass
        def move_async(self, width, speed):
            rec.jaw.append(width); return NS(wait=lambda timeout=None: not rec.__dict__.get('gripper_busy'))
    module(mp, 'franky', Robot=Robot, Gripper=Gripper, JointMotion=JointMotion, RealtimeConfig=NS(Ignore='ignore'))


class Repeated(list):
    def __init__(self, make): super().__init__(); self.make = make
    def add(self): self.append(self.make()); return self[-1]


def kinova(mp, rec):
    state = NS(mode=None)
    def feedback():
        return NS(base=NS(fault_bank_a=int(rec.fault), fault_bank_b=0, active_state=1),
                  actuators=[NS(position=math.degrees(v) % 360, velocity=0.0, temperature_motor=30.0, fault_bank_a=0, fault_bank_b=0)
                             for v in rec.q], interconnect=NS(gripper_feedback=NS(motor=[NS(position=50.0)])))

    class Command:
        def __init__(self):
            self.frame_id = 0
            self.actuators = Repeated(lambda: NS(flags=0, position=0.0, velocity=0.0, command_id=0))
            self.interconnect = NS(gripper_command=NS(motor_cmd=Repeated(lambda: NS(position=0.0, velocity=0.0, force=0.0))))

    class BaseCyclicClient:
        def __init__(self, router): pass
        def RefreshFeedback(self): return feedback()
        def Refresh(self, cmd, device, options):
            if any(a.flags != 1 or a.command_id != cmd.frame_id for a in cmd.actuators): rec.bad.append('stale command ids')
            rec.move([wrap(math.radians(a.position)) for a in cmd.actuators])
            return feedback()

    class BaseClient:
        def __init__(self, router): pass
        def SetServoingMode(self, info):
            rec.change(info.servoing_mode); rec.streaming = info.servoing_mode == 'low'; rec.limped = info.servoing_mode == 'single'
    class Transport:
        def connect(self, host, port): pass
        def disconnect(self): pass
    module(mp, 'kortex_api')
    module(mp, 'kortex_api.TCPTransport', TCPTransport=Transport)
    module(mp, 'kortex_api.UDPTransport', UDPTransport=Transport)
    module(mp, 'kortex_api.RouterClient', RouterClient=lambda t, cb: NS(),
           RouterClientSendOptions=lambda: NS(andForget=True, delay_ms=0, timeout_ms=0))
    module(mp, 'kortex_api.SessionManager', SessionManager=lambda r: NS(CreateSession=lambda i: None, CloseSession=lambda: None))
    module(mp, 'kortex_api.autogen.messages.Session_pb2', CreateSessionInfo=lambda: NS())
    module(mp, 'kortex_api.autogen.messages.Base_pb2', ServoingModeInformation=lambda: NS(servoing_mode=None),
           LOW_LEVEL_SERVOING='low', SINGLE_LEVEL_SERVOING='single')
    module(mp, 'kortex_api.autogen.messages.BaseCyclic_pb2', Command=Command)
    module(mp, 'kortex_api.autogen.client_stubs.BaseClientRpc', BaseClient=BaseClient)
    module(mp, 'kortex_api.autogen.client_stubs.BaseCyclicClientRpc', BaseCyclicClient=BaseCyclicClient)


def kuka(mp, rec):
    np = importlib.import_module('numpy')
    States = NS(IDLE='idle', MONITORING_READY='ready', COMMANDING_WAIT='wait', COMMANDING_ACTIVE='active')

    class LBRClient:
        def __init__(self):
            self._cmd = NS(setJointPosition=self._set)
        def _set(self, q):
            if q.dtype != np.float32: rec.bad.append('FRI wants float32')
            if rec.surveying: rec.bad.append('survey sent a command')
            rec.move(q.tolist())
        def robotCommand(self): return self._cmd
        def robotState(self):
            q = np.array(rec.q)
            return NS(getMeasuredJointPosition=lambda: q, getIpoJointPosition=lambda: q, getSampleTime=lambda: .005)

    class ClientApplication:
        def __init__(self, client): self.client, self.was = client, None
        def connect(self, port, host): return True
        def disconnect(self): pass
        def step(self):
            time.sleep(.001)
            if rec.fault: return False
            state = States.MONITORING_READY if rec.surveying else States.COMMANDING_ACTIVE
            if state != self.was:
                self.client.onStateChange(self.was, state); self.was = state
            rec.streaming = state == States.COMMANDING_ACTIVE
            (self.client.monitor if rec.surveying else self.client.command)()
            return True
    module(mp, 'pyfri', LBRClient=LBRClient, ClientApplication=ClientApplication, ESessionState=States)


def unitree(mp, rec):
    vendor_unitree.LINKS.clear()
    side_of = {}
    def low_state():
        motors = [NS(q=0.0, dq=0.0) for _ in range(35)]
        for side, ids in vendor_unitree.ARM_MOTORS.items():
            if side == rec.__dict__.get('side', 'left'):
                for m, v in zip(ids, rec.q): motors[m].q = v
        return NS(motor_state=motors, mode_machine=5, crc=0)

    class ChannelSubscriber:
        def __init__(self, topic, kind): self.run = True
        def Init(self, handler, depth):
            def loop():
                while self.run:
                    if not rec.fault: handler(low_state())
                    time.sleep(.003)
            threading.Thread(target=loop, daemon=True).start()
        def Close(self): self.run = False

    class ChannelPublisher:
        def __init__(self, topic, kind):
            if topic != 'rt/lowcmd': rec.bad.append(topic)
        def Init(self): rec.change('publisher'); rec.streaming = True
        def Write(self, cmd):
            if cmd.mode_machine != 5: rec.bad.append('mode_machine not echoed')
            ids = vendor_unitree.ARM_MOTORS[rec.__dict__.get('side', 'left')]
            rec.limped = cmd.motor_cmd[ids[0]].kp == 0
            if not rec.limped: rec.move([cmd.motor_cmd[m].q for m in ids])
            if cmd.motor_cmd[30].kp == 0: rec.bad.append('the head was let go')
        def Close(self): pass
    module(mp, 'unitree_sdk2py')
    module(mp, 'unitree_sdk2py.core.channel', ChannelFactoryInitialize=lambda d, nic: side_of.setdefault('nic', nic),
           ChannelSubscriber=ChannelSubscriber, ChannelPublisher=ChannelPublisher)
    module(mp, 'unitree_sdk2py.idl.unitree_hg.msg.dds_', LowCmd_=object, LowState_=object)
    module(mp, 'unitree_sdk2py.idl.default', unitree_hg_msg_dds__LowCmd_=lambda: NS(
        mode_pr=None, mode_machine=None, crc=None, motor_cmd=[NS(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0) for _ in range(35)]))
    module(mp, 'unitree_sdk2py.utils.crc', CRC=lambda: NS(Crc=lambda msg: 0))
    active = lambda: rec.__dict__.get('service', '')
    module(mp, 'unitree_sdk2py.comm.motion_switcher.motion_switcher_client', MotionSwitcherClient=lambda: NS(
        SetTimeout=lambda s: None, Init=lambda: None, CheckMode=lambda: (0, dict(name=active())),
        ReleaseMode=lambda: rec.bad.append('released the motion service')))


def abb(mp, rec):
    np = importlib.import_module('numpy')

    class EGM:
        def __init__(self, port=6510): self.at = 0.0
        def receive_from_robot(self, timeout=0):
            if time.monotonic() - self.at < .004:
                time.sleep(timeout); return False, None
            self.at = time.monotonic()
            d = [math.degrees(v) for v in rec.q]              # kinematic 1,2,7,3,4,5,6 -> EGM axes 1-6 + external 7
            return True, NS(joint_angles=np.array(d[:2] + d[3:]), external_axes=np.array([d[2]]),
                            rapid_running=True, motors_on=not rec.fault)
        def send_to_robot(self, joints, external_joints=None):
            rec.streaming = True
            if external_joints is None or len(joints) != 6: rec.bad.append('axis 7 must go out as the external axis')
            rec.move([math.radians(v) for v in list(joints[:2]) + list(external_joints) + list(joints[2:])]); return True
        def close(self): pass
    module(mp, 'abb_robot_client.egm', EGM=EGM)


ROS = dict(
    rethink_sawyer=dict(names=[f'right_j{i}' for i in range(7)], state='/robot/joint_states', cmd='/robot/limb/right/joint_command', key='position'),
    rethink_baxter=dict(names=[f'left_{j}' for j in ('s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2')], state='/robot/joint_states',
                        cmd='/robot/limb/left/joint_command', key='command'),
    pr2=dict(names=[f'l_{j}_joint' for j in ('shoulder_pan', 'shoulder_lift', 'upper_arm_roll', 'elbow_flex', 'forearm_roll', 'wrist_flex', 'wrist_roll')],
             state='/joint_states', cmd='/l_arm_controller/command', key='points'),
    galaxea_r1_lite=dict(names=None, state='/hdas/feedback_arm_left', cmd='/motion_target/target_joint_state_arm_left', key='position'),
    robotis_ai_worker=dict(names=[f'arm_l_joint{i}' for i in range(1, 8)], state='/joint_states',
                           cmd='/leader/joint_trajectory_command_broadcaster_left/joint_trajectory', key='points'))


def rosbridge(arm_id):
    spec = ROS[arm_id]

    def install(mp, rec):
        class Ros:
            def __init__(self, host, port=None): self.is_connected, self.subs, self.alive = False, {}, True
            def run(self, timeout=10):
                self.is_connected = True
                threading.Thread(target=self.feed, daemon=True).start()
            def close(self, timeout=10): self.alive = False
            def feed(self):
                while self.alive:
                    if not rec.fault:
                        names = spec['names']
                        msg = dict(position=list(rec.q) + [.3], name=(names + ['gripper_l_joint1']) if names else [])
                        for topic, handlers in list(self.subs.items()):
                            for h in handlers:
                                if topic == spec['state']: h(msg)
                                elif topic == '/robot/state': h(dict(stopped=False, error=False, estop_button=0))
                                elif topic.endswith('gripper_controller/state'): h(dict(process_value=.04))
                                elif 'feedback_gripper' in topic: h(dict(position=[50.0]))
                    time.sleep(.004)

        class Topic:
            def __init__(self, ros, name, kind): self.ros, self.name, self.kind = ros, name, kind
            def subscribe(self, cb): self.ros.subs.setdefault(self.name, []).append(cb)
            def unsubscribe(self): self.ros.subs.pop(self.name, None)
            def advertise(self): rec.change(f'advertise {self.name}'); rec.streaming = True
            def unadvertise(self): pass
            def publish(self, msg):
                if self.name != spec['cmd']:
                    rec.jaw.append(msg); return
                q = msg['points'][0]['positions'] if spec['key'] == 'points' else msg[spec['key']]
                if spec['key'] == 'points' and not msg['joint_names'][:len(rec.q)] == spec['names']: rec.bad.append('joint names')
                if msg.get('mode', 1) != 1: rec.bad.append('not POSITION_MODE')
                rec.move(q[:rec.dof])
        module(mp, 'roslibpy', Ros=Ros, Topic=Topic, Message=dict)
    return install


CASES = dict(
    trossen_wxai=(trossen, '192.168.1.2'), piper=(piper, 'can0'), arx_x5=(arx, 'can0'), xarm7=(xarm, '192.168.1.185'),
    ur5e=(ur, '192.168.1.5'), franka_fr3=(franky, '172.16.0.2'), franka_panda=(franky, '172.16.0.2'), kinova_gen3=(kinova, '192.168.1.10'),
    kuka_iiwa14=(kuka, '172.24.1.1:30200'), unitree_r1_a7=(unitree, 'eth0'), abb_yumi=(abb, '6511'),
    **{k: (rosbridge(k), 'robot:9090') for k in ROS})
ARX_LIMITS = [[-3.0, 3.0]] * 6


@pytest.fixture(params=sorted(CASES))
def rig(request, monkeypatch):
    arm_id = request.param
    row = catalog()[arm_id]
    rec = Rec(row['dof'])
    install, bus = CASES[arm_id]
    install(monkeypatch, rec)
    limits = row['limits'] or ARX_LIMITS
    rec.q = [0.3 if hi - lo >= 12 else (lo + hi) / 2 for lo, hi in limits]
    made = []

    def make(connect=True, **kw):
        if arm_id == 'arx_x5': kw.setdefault('limits', ARX_LIMITS)
        arm = open_arm(arm_id, 'left', bus, connect=connect, **kw)
        made.append(arm)
        return arm
    yield NS(id=arm_id, rec=rec, make=make, limits=limits)
    for arm in made:
        arm.close()


def near(a, b, tol=2e-3):
    return len(a) == len(b) and all(abs(wrap(x - y)) < tol for x, y in zip(a, b))


def test_connect_moves_nothing_and_uses_the_streaming_mode(rig):
    rig.make()
    assert wait(lambda: len(rig.rec.moves) >= 3)
    time.sleep(.05)
    assert rig.rec.streaming, 'the maker\'s streaming mode was not entered'
    assert all(near(q, rig.rec.q) for q in list(rig.rec.moves)), 'a stream target differed from the measured pose'
    assert not rig.rec.bad


def test_targets_stop_at_the_joint_limit(rig):
    j = next(i for i, (lo, hi) in enumerate(rig.limits) if hi - lo < 12)
    hi = rig.limits[j][1]
    rig.rec.q[j] = hi - .02
    arm = rig.make()
    assert wait(lambda: rig.rec.moves)
    target = list(rig.rec.q); target[j] = hi + 1
    arm.command(target, 1.0)
    assert wait(lambda: abs(rig.rec.last[j] - hi) < 1e-3)
    time.sleep(.05)
    assert max(q[j] for q in list(rig.rec.moves)) <= hi + 1e-3 and not rig.rec.bad


def test_a_command_with_the_wrong_joint_count_is_refused(rig):
    arm = rig.make()
    assert wait(lambda: rig.rec.moves)
    for bad in (rig.rec.q[:-1], rig.rec.q + [0.0]):
        with pytest.raises(ValueError): arm.command(bad, 1.0)
    with pytest.raises(ValueError): arm.command([math.nan] * rig.rec.dof, 1.0)
    time.sleep(.03)
    assert all(len(q) == rig.rec.dof for q in list(rig.rec.moves))


def test_a_fault_latches(rig):
    arm = rig.make()
    assert wait(lambda: rig.rec.moves)
    rig.rec.fault = True
    assert wait(lambda: arm._error is not None), 'the fault was not noticed'
    rig.rec.fault = False                                    # clearing on the arm does not clear the latch
    time.sleep(.03)
    with pytest.raises(ArmHardwareError): arm.read()
    with pytest.raises(ArmHardwareError): arm.command(rig.rec.q, 1.0)


def test_release_stops_the_stream_and_a_command_resumes_it(rig):
    arm = rig.make()
    assert wait(lambda: rig.rec.moves)
    arm.release()
    count = len(rig.rec.moves)
    time.sleep(.06)
    if rig.id == 'kuka_iiwa14':                              # FRI must be answered every cycle: it gets the robot's own pose
        assert all(near(q, rig.rec.q) for q in list(rig.rec.moves)[count:])
        count = len(rig.rec.moves)
    else:
        assert len(rig.rec.moves) == count
    arm.command(rig.rec.q, 1.0)
    assert wait(lambda: len(rig.rec.moves) > count + 2)
    assert near(rig.rec.last, rig.rec.q)


def test_the_check_is_read_only(rig):
    arm = rig.make(connect=False)
    rig.rec.surveying = True
    if rig.id == 'arx_x5':
        with pytest.raises(NotImplementedError): arm.survey()
        return
    out = arm.survey()
    assert out['read_only'] and near(out['positions_rad'], rig.rec.q)
    assert not rig.rec.moves and not rig.rec.changes and not rig.rec.bad


def test_catalog_rows_are_complete():
    rows = catalog()
    for arm_id in CASES:
        row = rows[arm_id]
        assert row['status'] == 'beta' and row['driver'] and row['interface'] and row['sources']
        module_name, _, cls = row['driver'].partition(':')
        assert hasattr(importlib.import_module(module_name), cls)        # no vendor SDK is needed to import a driver
        options = row.get('options') or {}
        tables = [row['limits']] + list((options.get('side_limits') or {}).values())
        for limits in filter(None, tables):
            assert len(limits) == row['dof'] and all(lo < hi for lo, hi in limits)
            assert all(lo - 1e-9 <= h <= hi + 1e-9 for h, (lo, hi) in zip(row['home'], limits)), arm_id


@pytest.mark.parametrize('arm_id', ['blueworm_mantis', 'bracketbot', 'robo_t1'])
def test_arms_without_a_usable_interface_claim_no_driver(arm_id):
    row = catalog()[arm_id]
    assert row['driver'] is None and row['status'] == 'bring-your-own' and row['note']
    with pytest.raises(SystemExit): open_arm(arm_id, 'left')


def test_a_missing_sdk_names_what_to_install(monkeypatch):
    monkeypatch.setitem(sys.modules, 'xarm.wrapper', None)
    monkeypatch.setitem(sys.modules, 'xarm', None)
    with pytest.raises(SystemExit, match='not available for this computer'):
        open_arm('xarm7', 'left', '10.0.0.1')


def test_arx_will_not_open_without_limits(monkeypatch):
    arx(monkeypatch, Rec(6))
    with pytest.raises(ValueError, match='no joint limits'):
        open_arm('arx_x5', 'left', 'can0', connect=False)


def test_ur_needs_a_running_robot(monkeypatch):
    rec = Rec(6); rec.robot_mode = 5
    ur(monkeypatch, rec)
    with pytest.raises(ArmHardwareError, match='power it on'):
        open_arm('ur5e', 'left', '10.0.0.2')
    assert not rec.changes


def test_trossen_error_text_is_a_fault_but_no_error_is_not(monkeypatch):
    from roborun_sdk.arms.vendor_trossen import TrossenArm
    assert TrossenArm._fault(NS(get_error_information=lambda: 'No error')) is None
    assert TrossenArm._fault(NS(get_error_information=lambda: '')) is None
    assert TrossenArm._fault(NS(get_error_information=lambda: 'joint 3 over temperature'))


def test_unitree_refuses_while_the_motion_service_runs(monkeypatch):
    rec = Rec(7); rec.service = 'ai'
    unitree(monkeypatch, rec)
    arm = open_arm('unitree_r1_a7', 'left', 'eth0')
    assert wait(lambda: arm._error is not None)
    assert 'motion service' in arm._error and not rec.moves and not rec.bad
    arm.close()


def test_right_arms_use_their_own_limits(monkeypatch):
    rec = Rec(7)
    unitree(monkeypatch, rec)
    arm = open_arm('unitree_r1_a7', 'right', 'eth0', connect=False)
    assert arm.limits[1] == (-2.4785, 0.2268) and open_arm('unitree_r1_a7', 'left', 'eth0', connect=False).limits[1] == (-0.2269, 2.4784)


def test_kinova_continuous_joint_crosses_zero_without_a_jump(monkeypatch):
    rec = Rec(7)
    kinova(monkeypatch, rec)
    rec.q = [-0.01, .1, .2, .3, .4, .5, .6]
    arm = open_arm('kinova_gen3', 'left', '10.0.0.3')
    try:
        assert wait(lambda: rec.moves)
        first = arm.read()['positions_rad'][0]
        assert first == pytest.approx(-.01, abs=1e-6)                    # 359.4 degrees on the wire
        rec.q[0] = .01
        assert wait(lambda: abs(arm.read()['positions_rad'][0] - .01) < 1e-6)
    finally:
        arm.close()


def test_a_busy_gripper_still_gets_the_latest_target(monkeypatch):
    rec = Rec(7); rec.q = [0, 0, 0, -1.5, 0, 1.5, 0]; rec.gripper_busy = True
    franky(monkeypatch, rec)
    arm = open_arm('franka_fr3', 'left', '10.0.0.5')
    try:
        assert wait(lambda: rec.moves)
        arm.jaw_speed = 50.0
        arm.command(rec.q, 1.0); assert wait(lambda: len(rec.jaw) == 1)
        arm.command(rec.q, 0.0); time.sleep(.3)
        assert len(rec.jaw) == 1                                         # still moving: nothing new is sent
        rec.gripper_busy = False
        assert wait(lambda: rec.jaw[-1] == pytest.approx(0.0, abs=1e-6))
    finally:
        arm.close()


def test_seven_joints_survive_the_station_or_it_refuses(monkeypatch):
    """Today's station keeps 6 joints. A 7-joint arm must then refuse to move, never run with its last joint dropped."""
    from roborun_sdk.embodiment import _station_classes
    rec = Rec(7); rec.q = [.1, .2, .3, 1.0, .5, .6, .7]
    xarm(monkeypatch, rec)
    arm = open_arm('xarm7', 'left', '10.0.0.4')
    try:
        assert wait(lambda: rec.moves)
        station = _station_classes()(arm, None, lease_free_hold=True, predict=lambda body: {})
        for _ in range(5):
            station.left.step(time.monotonic()); time.sleep(.01)
        assert station.left.errors > 0 or len(station.left.q) == 7
        time.sleep(.03)
        assert all(len(q) == 7 and q[6] == pytest.approx(.7, abs=1e-3) for q in list(rec.moves))
        assert arm._error is None
    finally:
        arm.close()


# ---- the real packages, when installed: do the names and arguments we use exist? ------------------------------------
REAL = [
    ('trossen_arm', 'TrossenArmDriver', ['configure', 'set_all_modes', 'set_arm_positions', 'set_gripper_position', 'get_all_positions',
                                         'get_all_velocities', 'get_error_information', 'get_controller_version', 'cleanup'],
     ['goal_time', 'blocking']),
    ('piper_sdk', 'C_PiperInterface_V2', ['ConnectPort', 'DisconnectPort', 'isOk', 'GetArmJointMsgs', 'GetArmStatus', 'GetArmGripperMsgs',
                                          'EnablePiper', 'DisableArm', 'MotionCtrl_2', 'JointCtrl', 'GripperCtrl',
                                          'GetPiperFirmwareVersion', 'GetArmEnableStatus'], []),
    ('xarm.wrapper', 'XArmAPI', ['get_servo_angle', 'get_gripper_position', 'motion_enable', 'set_mode', 'set_state', 'set_gripper_mode',
                                 'set_gripper_enable', 'set_gripper_position', 'set_servo_angle_j', 'disconnect', 'connected', 'axis',
                                 'error_code', 'warn_code', 'mode', 'state', 'version_number'], ['is_radian']),
    ('rtde_receive', 'RTDEReceiveInterface', ['getRobotMode', 'getSafetyMode', 'isConnected', 'isProtectiveStopped', 'isEmergencyStopped',
                                              'getActualQ', 'getActualQd', 'getJointTemperatures', 'disconnect'], []),
    ('rtde_control', 'RTDEControlInterface', ['servoJ', 'servoStop', 'stopScript', 'disconnect', 'isProgramRunning'], []),
    ('franky', 'Robot', ['move', 'stop', 'poll_motion', 'has_errors', 'state'], []),
    ('franky', 'Gripper', ['move_async', 'width', 'max_width'], []),
    ('franky', 'BoolFuture', ['wait'], ['timeout']),
    ('franky', 'JointMotion', [], ['return_when_finished']),
    ('abb_robot_client.egm', 'EGM', ['receive_from_robot', 'send_to_robot', 'close'], ['external_joints', 'timeout']),
    ('roslibpy', 'Ros', ['run', 'close', 'is_connected'], []),
    ('roslibpy', 'Topic', ['subscribe', 'unsubscribe', 'advertise', 'unadvertise', 'publish'], []),
    ('kortex_api.autogen.client_stubs.BaseClientRpc', 'BaseClient', ['SetServoingMode'], []),
    ('kortex_api.autogen.client_stubs.BaseCyclicClientRpc', 'BaseCyclicClient', ['Refresh', 'RefreshFeedback'], []),
]


@pytest.mark.parametrize('module_name,cls,names,args', REAL, ids=[f'{m}.{c}' for m, c, _, _ in REAL])
def test_real_sdk_has_what_we_call(monkeypatch, module_name, cls, names, args):
    kind = getattr(pytest.importorskip(module_name, exc_type=ImportError), cls)
    missing = [n for n in names if not hasattr(kind, n) and n not in getattr(kind, '__annotations__', {})]
    if cls == 'XArmAPI':                 # plain attributes are properties here, checked the same way
        missing = [n for n in names if not hasattr(kind, n)]
    assert not missing
    text = ' '.join(str(getattr(getattr(kind, n, None), '__doc__', '') or '') for n in names + ['__init__'])
    for arg in args:
        found = arg in text
        for n in names + ['__init__']:
            try:
                found = found or arg in inspect.signature(getattr(kind, n)).parameters
            except (TypeError, ValueError, AttributeError):
                pass
        assert found, f'{cls} takes no {arg}'


def test_real_sdk_objects_take_our_arguments():
    """Offline calls into the real packages that get past argument checking and stop only for lack of a robot."""
    trossen_sdk = pytest.importorskip('trossen_arm', exc_type=ImportError)
    with pytest.raises(Exception, match='not configured'):
        trossen_sdk.TrossenArmDriver().set_arm_positions([0.0] * 6, goal_time=.01, blocking=False)
    franky_sdk = pytest.importorskip('franky', exc_type=ImportError)
    franky_sdk.JointMotion([0.0] * 7, return_when_finished=False)
    assert franky_sdk.RealtimeConfig.Ignore is not None
    arm = pytest.importorskip('piper_sdk', exc_type=ImportError).C_PiperInterface_V2('none', judge_flag=False, can_auto_init=False)
    assert int(arm.GetArmStatus().arm_status.arm_status) == 0 and arm.GetArmJointMsgs().joint_state.joint_6 == 0
    assert arm.GetArmGripperMsgs().gripper_state.grippers_angle == 0
    egm = pytest.importorskip('abb_robot_client.egm', exc_type=ImportError).EGM(port=0)
    assert egm.receive_from_robot(timeout=0) == (False, None) and egm.send_to_robot([0.0] * 6, external_joints=[0.0]) is False
    egm.close()


def test_station_ends_a_lease_when_the_rules_file_keeps_refusing_it():
    from roborun_sdk.embodiment import KinematicArm, _station_classes

    class Refused(KinematicArm):
        refusing = None
        def rule_refusal(self): return self.refusing

    arm = Refused(home=[0.0] * 6)
    station = _station_classes()(arm, None, lease_free_hold=True)
    station.observations['o'] = station.clock()
    sid = station.register_session('s')
    lease = station.action(dict(observation_id='o', seconds=4, reference=dict(kind='video_motion', session_id=sid)))['lease']
    station.tick(); assert station.lease['active']
    arm.refusing = 'your rules file refused this motion of the left arm'
    station.tick()
    assert not station.lease['active'] and station.lease['reason'] == arm.refusing and station.state == 'idle' and lease['id'] == station.lease['id']


def test_a_motion_that_ends_by_itself_lets_the_jaw_finish_and_a_stop_does_not():
    from roborun_sdk.embodiment import KinematicArm, _station_classes
    def leased():
        arm = KinematicArm(home=[0.0] * 6); arm.jaw_speed = .01
        station = _station_classes()(arm, None, lease_free_hold=True)
        station.observations['o'] = station.clock(); sid = station.register_session('s')
        lease = station.action(dict(observation_id='o', seconds=4, reference=dict(kind='video_motion', session_id=sid)))['lease']
        station.left.command(jaw=.4); station.tick()
        return station, lease
    station, lease = leased()
    station.supervisor_stop(dict(lease_id=lease['id']))
    assert station.left.jaw_cmd == .4
    station, lease = leased()
    station.hold(dict(wait_s=0))
    assert station.left.jaw_cmd == pytest.approx(station.left.jaw, abs=.02) and station.left.jaw_cmd > .9
