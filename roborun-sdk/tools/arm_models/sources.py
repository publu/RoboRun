"""Where each catalog arm's model comes from and how its parts map onto the planner's names. Checked-out copies live under --src/<repo>."""
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

REPOS = {
    'menagerie': dict(url='https://github.com/google-deepmind/mujoco_menagerie', commit='370c43965323c0944b433b202aac5058b5f907a9', license='per model'),
    'SO-ARM100': dict(url='https://github.com/TheRobotStudio/SO-ARM100', commit='a758567c3978dfeefe282ede0500085a48fe8f78', license='Apache-2.0'),
    'robotis_mujoco_menagerie': dict(url='https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie', commit='d8344c0dbe7a00208d0301111523dde65efc174a', license='Apache-2.0'),
    'openarm_description': dict(url='https://github.com/enactic/openarm_description', commit='73dda1b097de26634909e06c2935aa21895af080', license='Apache-2.0'),
    'yumi': dict(url='https://github.com/OrebroUniversity/yumi', commit='fb7f57df05444e6a28df6eda26034de730b8f8fe', license='BSD-2-Clause'),
    'unitree_ros': dict(url='https://github.com/unitreerobotics/unitree_ros', commit='157ff950351e5f535e03c5d85a16b7d594b3dbcc', license='BSD-3-Clause'),
    'baxter_common': dict(url='https://github.com/RethinkRobotics/baxter_common', commit='6c4b0f375fe4e356a3b12df26ef7c0d5e58df86e', license='BSD-3-Clause'),
    'ARX_X5': dict(url='https://github.com/ARXroboticsX/ARX_X5', commit='c78328785ea23a81d908e1dcc551eca992f2f1e9', license='BSD-3-Clause'),
    'pr2_common': dict(url='https://github.com/PR2/pr2_common', commit='9a8e4fbcf66523ca14faf651c23513ee1ac29ccb', license='BSD-3-Clause'),
    'makermodslab': dict(url='https://github.com/makermods-robotics/makermodslab', commit='e377149419eebef7fa0f694b0d616c9c2cd46231', license='Apache-2.0 (the Metal URDF folder: MIT)'),
    'bw-mantis-sdk': dict(url='https://pypi.org/project/bw-mantis-sdk/1.3.9/', commit='1.3.9 (PyPI wheel)', license='MIT'),
    'i2rt': dict(url='https://github.com/i2rt-robotics/i2rt', commit='234d895 (vendored at roborun_sdk/third_party/i2rt)', license='MIT'),
}


def _mesh_index(root):
    out = {}
    for p in Path(root).rglob('*'):
        if p.suffix.lower() in ('.stl', '.dae', '.obj') and p.is_file(): out.setdefault(p.name.lower(), []).append(p)
    return out


def _find_mesh(ref, root, index):
    rel = re.sub(r'^(package|file|model)://', '', ref).lstrip('/')
    parts = Path(rel).parts
    for skip in range(len(parts)):
        p = Path(root).joinpath(*parts[skip:])
        if p.is_file(): return p
    hits = index.get(Path(rel).name.lower(), [])
    if not hits: raise SystemExit(f'mesh {ref} not found under {root}')
    return sorted(hits, key=lambda h: (len(h.parts), str(h)))[0]


def urdf_spec(text, root, base_dir=None, skip_missing=False):
    """A URDF (text) -> MjSpec: its meshes copied beside it as STL (Collada converted), package:// resolved inside the repo."""
    import trimesh
    tmp = Path(tempfile.mkdtemp(prefix='arm-urdf-')); index = _mesh_index(root); r = ET.fromstring(text); done = {}
    for parent in list(r.iter()):
        for kind in ('visual', 'collision'):
            for el in list(parent.findall(kind)):
                m = el.find('geometry/mesh')
                if m is None: continue
                try: _find_mesh(m.get('filename'), root, index)
                except SystemExit:
                    if not skip_missing: raise
                    parent.remove(el)                  # a part the repository does not carry (a sensor from another package, or no meshes at all)
    for m in r.iter('mesh'):
        src = _find_mesh(m.get('filename'), root, index)
        key = (src, m.get('scale'))                    # one file at two scales (a mirrored left/right part) is two assets to MuJoCo
        if key not in done:
            name = f'm{len(done)}.stl'
            if src.suffix.lower() == '.stl': (tmp / name).write_bytes(src.read_bytes())
            else:
                mesh = trimesh.load(str(src), force='mesh'); mesh.export(str(tmp / name))
            done[key] = name
        m.set('filename', done[key])
    extra = ET.SubElement(r, 'mujoco'); ET.SubElement(extra, 'compiler', dict(meshdir=str(tmp), strippath='true', balanceinertia='true', boundmass='.001', boundinertia='.00001', fusestatic='false', discardvisual='false'))
    path = tmp / 'robot.urdf'; path.write_text(ET.tostring(r, encoding='unicode'))
    return mujoco.MjSpec.from_file(str(path))


def xacro_text(path, root, packages, args):
    import xacro, xacro.substitution_args as sa
    roots = {k: str(Path(root) / v) for k, v in packages.items()}
    sa._eval_find = lambda pkg: roots[pkg]
    sa._eval_dict['find'] = sa._eval_find if hasattr(sa, '_eval_dict') else None
    return xacro.process_file(str(path), mappings=args).toxml()


def resolve(a, src):
    """(MjSpec, path) of an arm's source model. source = '<repo>:<file>'; an i2rt arm = 'i2rt:<arm type>+<gripper type>'."""
    repo, _, rel = a['source'].partition(':'); root = Path(src) / repo
    if repo == 'i2rt':
        from unittest import mock
        sys.path.insert(0, str(Path(src).resolve() / 'i2rt').rsplit('/i2rt', 1)[0])
        while True:                                       # (the model code uses none of the bus and CLI packages its modules import)
            try:
                from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml; break
            except ModuleNotFoundError as exc:
                sys.modules[exc.name] = mock.MagicMock()
        arm, _, grip = rel.partition('+')
        made = combine_arm_and_gripper_xml(ArmType.from_string_name(arm), GripperType.from_string_name(grip))
        made = Path(made); return mujoco.MjSpec.from_file(str(made)), made
    path = root / rel
    if path.suffix == '.xml': return mujoco.MjSpec.from_file(str(path)), path
    text = xacro_text(path, root, a.get('xacro_packages', {}), a.get('xacro_args', {})) if '.xacro' in path.name else path.read_text()
    return urdf_spec(text, root, base_dir=path.parent, skip_missing=a.get('skip_missing_meshes', False)), path


def J(n, pre='joint', start=1, post=''): return [f'{pre}{i}{post}' for i in range(start, start + n)]


def rq2f85(to, pos=(0, 0, 0), quat=(1, 0, 0, 0)):
    """The Robotiq 2F-85 from the menagerie, for an arm that comes without a gripper in its public model (its pads move like a parallel jaw here)."""
    return dict(attach=[dict(source='menagerie:robotiq_2f85/2f85.xml', to=to, pos=list(pos), quat=list(quat), prefix='g_', license_file='robotiq_2f85/LICENSE')],
                gripper=dict(type='linkage', left=['g_left_driver', 'g_left_spring_link'], right=['g_right_driver', 'g_right_spring_link'], stroke=.0425), attached_gripper='Robotiq 2F-85 (menagerie, BSD-2-Clause)')


SO101J = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
PANDA_HOME = [0, -.7854, 0, -2.3562, 0, 1.5708, .7854]
M = lambda rel, lic, f: dict(source=f'menagerie:{rel}/{f}', license=lic, license_file=f'{rel}/LICENSE')

ARMS = {
    'franka_panda': dict(M('franka_emika_panda', 'Apache-2.0', 'panda.xml'), joints=J(7), home=PANDA_HOME,
                         gripper=dict(type='slide', left=['left_finger'], right=['right_finger'], joints=['finger_joint1', 'finger_joint2'], open_pose=dict(finger_joint1=.04, finger_joint2=.04))),
    'franka_fr3': dict(M('franka_fr3', 'Apache-2.0', 'fr3.xml'), joints=J(7, 'fr3_joint'), home=PANDA_HOME,
                       attach=[dict(source='menagerie:franka_emika_panda/hand.xml', to='fr3_link7', pos=[0, 0, .107], quat=[.92388, 0, 0, -.38268], prefix='g_', license_file='franka_emika_panda/LICENSE')],
                       gripper=dict(type='slide', left=['g_left_finger'], right=['g_right_finger'], joints=['g_finger_joint1', 'g_finger_joint2'], open_pose=dict(g_finger_joint1=.04, g_finger_joint2=.04)), attached_gripper='Franka Hand (menagerie panda)'),
    'kinova_gen3': dict(M('kinova_gen3', 'BSD-3-Clause', 'gen3.xml'), joints=J(7, 'joint_'), home=[0, .26, 3.14, -2.27, 0, .96, 1.57], **rq2f85('bracelet_link', (0, 0, -.0615), (0, 1, 0, 0))),
    'kuka_iiwa14': dict(M('kuka_iiwa_14', 'BSD-3-Clause', 'iiwa14.xml'), joints=J(7), home=[0, .6, 0, -1.4, 0, 1.1, 0], **rq2f85('link7', (0, 0, .045))),
    'ur5e': dict(M('universal_robots_ur5e', 'BSD-3-Clause', 'ur5e.xml'), joints=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                 home=[0, -1.5708, 1.5708, -1.5708, -1.5708, 0], **rq2f85('wrist_3_link', (0, .1, 0), (-.70711, .70711, 0, 0))),
    'xarm7': dict(M('ufactory_xarm7', 'BSD-3-Clause', 'xarm7.xml'), joints=J(7), home=[0, -.3, 0, .8, 0, 1.1, 0],
                  gripper=dict(type='linkage', left=['left_outer_knuckle', 'left_inner_knuckle'], right=['right_outer_knuckle', 'right_inner_knuckle'], stroke=.0425)),
    'rethink_sawyer': dict(M('rethink_robotics_sawyer', 'Apache-2.0', 'sawyer.xml'), joints=J(7, 'right_j', 0), home=[0, -1.18, 0, 2.18, 0, .57, 3.14], **rq2f85('right_l6', (0, 0, .0245), (0, 0, 0, 1))),
    'viperx_300s': dict(M('trossen_vx300s', 'BSD-3-Clause', 'vx300s.xml'), joints=['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate'], home=[0, -1.65, 1.55, 0, .8, 0],
                        gripper=dict(type='slide', left=['left_finger_link'], right=['right_finger_link'], joints=['left_finger', 'right_finger'], open_pose=dict(left_finger=.057, right_finger=-.057))),
    'widowx_250s': dict(M('trossen_wx250s', 'BSD-3-Clause', 'wx250s.xml'), joints=['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate'], home=[0, -1.8, 1.55, 0, .8, 0],
                        gripper=dict(type='slide', left=['wx250s/left_finger_link'], right=['wx250s/right_finger_link'], joints=['left_finger', 'right_finger'], open_pose=dict(left_finger=.037, right_finger=-.037))),
    'trossen_wxai': dict(M('trossen_wxai', 'BSD-3-Clause', 'wxai_follower.xml'), joints=J(6, 'joint_', 0), home=[0, 1.5708, 1.5708, 0, 0, 0],
                         gripper=dict(type='slide', left=['carriage_left'], right=['carriage_right'], stroke=.045, open_pose=dict(left_carriage_joint=.045, right_carriage_joint=.045), drop=['camera_mount_d405', 'camera_link'])),
    'piper': dict(M('agilex_piper', 'MIT', 'piper.xml'), joints=J(6), home=[0, .9, -.9, 0, 0, 0],
                  gripper=dict(type='slide', left=['link7'], right=['link8'], joints=['joint7', 'joint8'], open_pose=dict(joint7=.035, joint8=-.035))),
    'rebot_b601_dm': dict(M('seeed_rebot_devarm', 'MIT', 'seeed_rebot_devarm.xml'), joints=J(6), home=[0, -1.4, -1.4, 0, 0, 0],
                          gripper=dict(type='slide', left=['gripper_left'], right=['gripper_right'], joints=['joint_left', 'joint_right'], open_pose=dict(joint_left=.05, joint_right=.05))),
    'so101': dict(M('robotstudio_so101', 'Apache-2.0', 'so101.xml'), joints=SO101J, home=[0, 0, 0, 0, 0],
                  gripper=dict(type='revolute', joint='gripper', closed=-.1745, open=1.7453, drop=['camera_mount'])),
    'so100': dict(source='SO-ARM100:Simulation/SO100/so100.urdf', license='Apache-2.0', license_file='LICENSE', joints=SO101J, home=[0, 0, 0, 0, 0], zero=dict(shoulder_lift=1.75, elbow_flex=-1.5708, wrist_flex=-.65),
                  gripper=dict(type='revolute', joint='gripper', closed=-.2, open=2.0)),
    'koch_v1_1': dict(M('low_cost_robot_arm', 'Apache-2.0', 'low_cost_robot_arm.xml'), joints=['base_rotation', 'pitch', 'elbow', 'wrist_pitch', 'wrist_roll'], home=[0, 0, 0, 0, 0], zero=dict(pitch=-.485, elbow=-.06),
                      gripper=dict(type='revolute', joint='gripper', closed=.032, open=-1.6)),
    'omx': dict(source='robotis_mujoco_menagerie:robotis_omx/omx.xml', license='Apache-2.0', license_file='LICENSE', joints=J(5, 'Joint'), home=[0, 0, 0, 0, 0], zero=dict(Joint1=.7854, Joint2=-.2618, Joint3=-.2618),
                gripper=dict(type='revolute', joint='Gripper', closed=0., open=1.745329, follower='Gripper_mimic', follower_ratio=-1.)),
    'yam': dict(source='i2rt:yam+linear_4310', license='MIT', license_file='LICENSE', joints=J(6), home=[1.5708, .3, .3, 0, 0, 0],
                gripper=dict(type='slide', left=['tip_left'], right=['tip_right'], joints=['joint7', 'joint8'], open_pose=dict(joint7=.0475, joint8=.0475))),
    'yam_pro': dict(source='i2rt:yam_pro+linear_4310', license='MIT', license_file='LICENSE', joints=J(6), home=[1.5708, .3, .3, 0, 0, 0],
                    gripper=dict(type='slide', left=['tip_left'], right=['tip_right'], joints=['joint7', 'joint8'], open_pose=dict(joint7=.0475, joint8=.0475))),
    'yam_ultra': dict(source='i2rt:yam_ultra+linear_4310', license='MIT', license_file='LICENSE', joints=J(6), home=[1.5708, .3, .3, 0, 0, 0],
                      gripper=dict(type='slide', left=['tip_left'], right=['tip_right'], joints=['joint7', 'joint8'], open_pose=dict(joint7=.0475, joint8=.0475))),
    'big_yam': dict(source='i2rt:big_yam+linear_4310', license='MIT', license_file='LICENSE', joints=J(6), home=[1.5708, .3, .3, 0, 0, 0],
                    gripper=dict(type='slide', left=['tip_left'], right=['tip_right'], joints=['joint7', 'joint8'], open_pose=dict(joint7=.0475, joint8=.0475))),
    'maker_arm': dict(source='makermodslab:frontend/dist/maker-urdf/robot.urdf', license='Apache-2.0', license_file='LICENSE', joints=[f'link_00{i}_joint' for i in range(2, 8)], home=[0, -.9, 1.1, 0, 0, 0],
                      gripper=dict(type='slide', left=['gripper_left'], right=['gripper_right'], joints=['gripper_left_joint', 'gripper_right_joint'], open_pose=dict(gripper_left_joint=.0524, gripper_right_joint=.0524),
                                   tcp_body='grasp_center')),    # the maker's frame: the area centroid of the closing faces (revision_report.json grasp_frame)
    'metal_arm': dict(object_r=.32, source='makermodslab:frontend/dist/metal-urdf/metal_with_gripper.urdf', license='MIT', license_file='frontend/dist/metal-urdf/LICENSE', joints=J(6), home=[1.366871, -.9, 1.1, .1, 0, 0],
                      gripper=dict(type='slide', left=['link7'], right=['link8'], joints=['joint7', 'joint8'], open_pose=dict(joint7=-.05, joint8=.05))),

    # --- arms of two-armed robots: the base frame is the robot's torso; left and right are the robot's own, each built from its side of the model
    'unitree_r1_a7': dict(source='unitree_ros:robots/r1_a7_description/r1_a7.urdf', license='BSD-3-Clause', license_file='LICENSE', base_frame='waist_yaw_link', base_bodies=[], recenter=False, shared_base=True, shoulder_height=.34, auto_home=True, object_r=.27,
                          home=[0, .3, 0, .9, 0, 0, 0], sides=dict(left=dict(joints=[f'left_{j}_joint' for j in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')], **rq2f85('left_wrist_yaw_link', (.05, 0, 0), (.70711, 0, .70711, 0))),
                                                                     right=dict(joints=[f'right_{j}_joint' for j in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')], home=[0, -.3, 0, .9, 0, 0, 0], **rq2f85('right_wrist_yaw_link', (.05, 0, 0), (.70711, 0, .70711, 0))))),
    'rethink_baxter': dict(source='baxter_common:baxter_description/urdf/baxter.urdf', license='BSD-3-Clause', license_file='LICENSE', base_frame='torso', base_bodies=[], recenter=False, shared_base=True, base_z=-.05,
                           home=[0, -.55, 0, .75, 0, 1.26, 0], sides=dict(left=dict(joints=[f'left_{j}' for j in ('s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2')], **rq2f85('left_hand')), right=dict(joints=[f'right_{j}' for j in ('s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2')], **rq2f85('right_hand')))),
    'abb_yumi': dict(source='yumi:yumi_description/urdf/yumi.urdf', license='BSD-2-Clause', license_file='license.dat', base_frame='yumi_base_link', base_bodies=[], recenter=False, shared_base=True, base_z=0.,
                     home=[0, -.8, 0, .5, 0, .8, 0], sides=dict(left=dict(joints=[f'yumi_joint_{i}_l' for i in (1, 2, 7, 3, 4, 5, 6)], gripper=dict(type='slide', left=['gripper_l_finger_r'], right=['gripper_l_finger_l'], joints=['gripper_l_joint', 'gripper_l_joint_m'], open_pose=dict(gripper_l_joint=.025, gripper_l_joint_m=.025))),
                                                                  right=dict(joints=[f'yumi_joint_{i}_r' for i in (1, 2, 7, 3, 4, 5, 6)], gripper=dict(type='slide', left=['gripper_r_finger_r'], right=['gripper_r_finger_l'], joints=['gripper_r_joint', 'gripper_r_joint_m'], open_pose=dict(gripper_r_joint=.025, gripper_r_joint_m=.025))))),
    'blueworm_mantis': dict(source='bw-mantis-sdk:mantis/model/urdf/mantis.urdf', license='MIT', license_file='bw_mantis_sdk-1.3.9.dist-info/licenses/LICENSE', base_frame='Waist_Link', base_bodies=[], recenter=False, shared_base=True, shoulder_height=.38, auto_home=True, forward=(-1., 0.),
                            home=[.3, .1, 0, .6, 0, 0, 0], sides=dict(left=dict(joints=[f'L_{j}_Joint' for j in ('Shoulder_Pitch', 'Shoulder_Yaw', 'Shoulder_Roll', 'Elbow_Pitch', 'Wrist_Roll', 'Wrist_Pitch', 'Wrist_Yaw')], gripper=dict(type='slide', left=['L_Hand_L_Link'], right=['L_Hand_R_Link'], joints=['L_Hand_L_Joint', 'L_Hand_R_Joint'], open_pose=dict(L_Hand_L_Joint=.04, L_Hand_R_Joint=.04))),
                                                                       right=dict(joints=[f'R_{j}_Joint' for j in ('Shoulder_Pitch', 'Shoulder_Yaw', 'Shoulder_Roll', 'Elbow_Pitch', 'Wrist_Roll', 'Wrist_Pitch', 'Wrist_Yaw')], gripper=dict(type='slide', left=['R_Hand_L_Link'], right=['R_Hand_R_Link'], joints=['R_Hand_L_Joint', 'R_Hand_R_Joint'], open_pose=dict(R_Hand_L_Joint=.04, R_Hand_R_Joint=.04))))),
    'robotis_ai_worker': dict(source='robotis_mujoco_menagerie:robotis_ffw/ffw_bg2.xml', license='Apache-2.0', license_file='LICENSE', base_frame='arm_base_link', base_bodies=[], recenter=False, shared_base=True, shoulder_height=.4, auto_home=True,
                              home=[0, .6, 0, -1.2, 0, 0, 0], sides=dict(left=dict(joints=J(7, 'arm_l_joint'), gripper=dict(type='linkage', left=['gripper_l_l1'], right=['gripper_l_r1'], stroke=.05)),
                                                                          right=dict(joints=J(7, 'arm_r_joint'), home=[0, -.6, 0, -1.2, 0, 0, 0], gripper=dict(type='linkage', left=['gripper_r_l1'], right=['gripper_r_r1'], stroke=.05)))),
    'arx_x5': dict(source='ARX_X5:ROS2/X5_ws/src/arx_x5_ros2/arx_x5_controller/x5_2025.urdf', license='BSD-3-Clause', license_file='LICENSE', joints=J(6), base_z=.07, home=[0, 1., 1., 0, 0, 0], limits_override=[[-3.14159, 3.14159]] * 6, auto_home=True, ignore_catalog_limits=True, skip_missing_meshes=True, capsules=.022,
                   notes=['Stands on a 7 cm riser in the profile: with capsule links the shoulder link has no clearance to the table otherwise', 'ARX publishes the kinematics but not the meshes (the URDF names files its repo does not hold): the links are capsules along the arm', 'ARX publishes no joint limits (its URDF says +-10 rad); the model uses +-pi and the driver refuses to start until you pass limits'], **rq2f85('link6', (.08, 0, 0), (.70711, 0, .70711, 0))),
    'openarm_v2': dict(source='openarm_description:assets/robot/openarm_v2.0/urdf/example/v2.urdf', license='Apache-2.0', license_file='LICENSE.txt', copyright='Copyright 2026 Enactic, Inc.', skip_missing_meshes=True, base_bodies=[], recenter=False, shared_base=True, shoulder_height=.36, auto_home=True, object_r=.27, home=[-.5, -.1, 0, 1.0, 0, 0, 0], notes=['all 7 joints are planned: the v2 URDF is the arm (the driver now drives joint 3 too)'],
                       sides=dict(left=dict(joints=[f'openarm_left_joint{i}' for i in range(1, 8)], gripper=dict(type='revolute', joint='openarm_left_finger_joint1', closed=0., open=.7854, follower='openarm_left_finger_joint2', follower_ratio=1.)),
                                  right=dict(joints=[f'openarm_right_joint{i}' for i in range(1, 8)], home=[.5, .1, 0, 1.0, 0, 0, 0], gripper=dict(type='revolute', joint='openarm_right_finger_joint1', closed=0., open=-.7854, follower='openarm_right_finger_joint2', follower_ratio=1.)))),
    'openarm_v1': dict(source='openarm_description:assets/robot/openarm_v1.0/urdf/example/v1.urdf', license='Apache-2.0', license_file='LICENSE.txt', copyright='Copyright 2026 Enactic, Inc.', skip_missing_meshes=True, base_bodies=[], recenter=False, shared_base=True, shoulder_height=.36, auto_home=True, object_r=.27, home=[-.5, -.1, 0, 1.0, 0, 0, 0],
                       notes=['OpenArm 1.0 (Enactic; also sold built by Anvil Robotics): joints 1-5 match the v2.0 description, the wrist (6, 7) and the parallel jaw are its own'],
                       sides=dict(left=dict(joints=[f'openarm_left_joint{i}' for i in range(1, 8)], gripper=dict(type='slide', left=['openarm_left_left_finger'], right=['openarm_left_right_finger'], joints=['openarm_left_finger_joint1', 'openarm_left_finger_joint2'], open_pose=dict(openarm_left_finger_joint1=.044, openarm_left_finger_joint2=.044))),
                                  right=dict(joints=[f'openarm_right_joint{i}' for i in range(1, 8)], home=[.5, .1, 0, 1.0, 0, 0, 0], gripper=dict(type='slide', left=['openarm_right_left_finger'], right=['openarm_right_right_finger'], joints=['openarm_right_finger_joint1', 'openarm_right_finger_joint2'], open_pose=dict(openarm_right_finger_joint1=.044, openarm_right_finger_joint2=.044))))),
    'pr2': dict(source='pr2_common:pr2_description/robots/pr2.urdf.xacro', license='BSD-3-Clause', license_file='pr2_description/package.xml', license_text='licenses/pr2_description.BSD.txt', xacro_packages=dict(pr2_description='pr2_description'), base_frame='torso_lift_link', base_bodies=[], recenter=False, shared_base=True, shoulder_height=.42, auto_home=True, skip_missing_meshes=True,
                home=[.5, 0, 0, -1.2, 0, -.5, 0], sides=dict(left=dict(joints=[f'l_{j}_joint' for j in ('shoulder_pan', 'shoulder_lift', 'upper_arm_roll', 'elbow_flex', 'forearm_roll', 'wrist_flex', 'wrist_roll')], gripper=dict(type='linkage', left=['l_gripper_l_finger_link'], right=['l_gripper_r_finger_link'], stroke=.043)),
                                                               right=dict(joints=[f'r_{j}_joint' for j in ('shoulder_pan', 'shoulder_lift', 'upper_arm_roll', 'elbow_flex', 'forearm_roll', 'wrist_flex', 'wrist_roll')], home=[-.5, 0, 0, -1.2, 0, -.5, 0], gripper=dict(type='linkage', left=['r_gripper_l_finger_link'], right=['r_gripper_r_finger_link'], stroke=.043)))),
}
