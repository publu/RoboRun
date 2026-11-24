"""robot.yaml: everything about this robot's arms and cameras, in one local file that plain `roborun up` reads."""
import getpass
import json
import os
import re
import socket
import sys
from pathlib import Path

TEMPLATE = """\
# Your robot's local settings. `roborun setup` fills these in; edit anything, then run: roborun up
# Nothing here leaves this computer. `roborun hw arms` lists arms; `roborun hw detect` shows adapters, addresses and cameras.

# robot: my-robot            # the name shown in Roborun (default: this computer's name)
# profile: yam               # the arm shape Manifest plans for: yam (one YAM), yam_dual (two); roborun profiles

# arms:
#   left:
#     arm: yam               # or your own driver: my_arm.py:MyArm
#     address: slcan://SERIAL
#     # Anything else here goes to the arm's driver:
#     # joint_speed_rad_s: 0.6
#     # joint_accel_rad_s2: 2.0
#     # joint_jerk_rad_s3: 20
#     # jaw_speed: 0.6
#     # limits: [[-3, 3], [0, 3], [0, 3], [-1.5, 1.5], [-1.5, 1.5], [-2, 2]]
#     # home: [0, 0, 0, 0, 0, 0]
#     # rest_pose: [0, 0, 0, 0, 0, 0]    # where it can rest unpowered: parked and confirmed before motors go off
#     # home_before_motors_off: true     # default: true when rest_pose is set
#     # payload_kg: 0.2                  # YAM: what the gripper carries
#     # gravity_comp_factor: [1, 1.1, 1.1, 1.2, 1, 1]   # YAM
#     # gripper_open_rad: -3.5           # jaw motor angles, where the driver can use them (set both)
#     # gripper_closed_rad: 3.0
#   right:
#     arm: yam
#     address: gs_usb://SERIAL

# cameras:
#   top:
#     source: realsense:SERIAL
#     # width: 640
#     # height: 360
#     # fps: 30
#     # rotate: 0
#   left: usb:0

# dry_run: true              # read the arms and log what would be sent: nothing is enabled or written

# sim: cup                   # run a simulated arm instead of real ones (scenes: cup, pickup-bench, mixed-objects)

# motion:                    # defaults for every arm; an arm's own value wins. Unknown names are reported, then passed to your driver.
#   joint_speed_rad_s: 0.6
#   rate_hz: 100
#   command_timeout_s: 0.5   # commands older than this end in a hold
#   ramp_s: 0.5              # stiffness ramp after enabling
#   max_tool_speed_m_s: 0.5  # needs the arm's model
#   max_jaw_step: 0.006      # per stream cycle
#   temp_warn_c: 70          # default: by motor type; a number, or one per motor
#   temp_stop_c: 80
#   temperature_export: temps.jsonl
#   stall_guard: true
#   stall_error_rad: 0.2
#   stall_velocity_rad_s: 0.05
#   stall_time_s: 0.5
#   jaw_guard: true
#   gripper_effort: 1.0      # share of the jaw force to use
#   gripper_force_cap: 0.9   # effort at which a still jaw counts as gripping
#   gripper_hold_effort: 0.7 # share kept once gripping (0.4 or more: weaker reads as an empty jaw)
#   gripper_open_margin: 0.003
#   powered_at_connect: hold # or refuse

# body: my_rover.py:MyRover  # your own mobile base or humanoid: its class (BaseAdapter + HeadAdapter) and, with profile:,
#                           # its profile in ~/.config/roborun/bodies/<profile>.yaml. Or a section: adapter:, profile:, and
#                           # anything else for the class's constructor. Used for that profile only (--profile of
#                           # another body or of arms leaves it out). See BUILDING.md, "Your own robot body"

# privacy:                 # faces are blurred on every image that leaves this computer; false sends frames as they are
#                          # to the motion service and keeps history images here
#   face_blur: true

# policy:                  # policy runs (roborun policy run): the tool speed cap the station gives the arms while a
#   max_tool_speed_m_s: 0.25  # policy drives them; at most the arm's own max_tool_speed_m_s

# Your own rules, in Python: keep-out zones, speed rules, anything. See `check` below.
# rules: my_rules.py
#
#   def check(side, positions_rad, gripper, state):
#       # state: positions_rad, velocities_rad_s, temperatures_c, gripper, gripper_effort, commanded_rad, jaw_state,
#       #        thermal, tool_pose and tool_position_m (None without the arm's model)
#       if positions_rad[1] > 2.5:
#           return False                               # refuse: the arm holds where it is
#       return positions_rad, gripper                  # or return changed values; None also means "fine"
"""
KNOWN = {'robot', 'profile', 'token_file', 'arms', 'cameras', 'motion', 'privacy', 'rules', 'dry_run', 'sim', 'body', 'policy'}


def path() -> Path:
    from .. import robot
    return Path(robot.HOME) / 'robot.yaml'


def ensure_file() -> Path:
    file = path()
    if not file.exists():
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(TEMPLATE)
    return file


def load(warn=print) -> dict:
    import yaml
    file = path()
    if not file.exists():
        return {}
    try:
        value = yaml.safe_load(file.read_text())
    except yaml.YAMLError as exc:
        raise SystemExit(f'{file} is not valid: {exc}')
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise SystemExit(f'{file}: expected settings like "arms:" at the top level')
    for key in sorted(set(value) - KNOWN):
        warn(f'{file.name}: ignoring unknown setting {key!r}')
    return value


def _section(s, name):
    value = s.get(name) or {}
    if not isinstance(value, dict):
        raise SystemExit(f'{path().name}: {name} must be a section')
    return value


def _local(value):
    p = Path(str(value)).expanduser()
    return str(p if p.is_absolute() else (path().parent / p).resolve())


def to_hardware(s: dict):
    """The station's and camera service's hardware description, or None when no arm is set."""
    arms = _section(s, 'arms')
    motion = _section(s, 'motion')
    out = {}
    for side, row in arms.items():
        if side not in ('left', 'right'):
            raise SystemExit(f'{path().name}: arms are "left" and "right", not {side!r}')
        if row is None:
            continue
        if not isinstance(row, dict) or not row.get('arm'):
            raise SystemExit(f'{path().name}: arms.{side} needs "arm:"')
        row = dict(row)
        arm = str(row.pop('arm'))
        where, sep, cls = arm.partition(':')
        if sep and where.endswith('.py'):
            arm = f'{_local(where)}:{cls}'
        bus = row.pop('address', None)
        options = dict(motion, **row)
        if s.get('rules'):
            options.setdefault('rules', _local(s['rules']))
        if s.get('dry_run'):
            options.setdefault('dry_run', True)
        if options.get('temperature_export'):
            options['temperature_export'] = _local(options['temperature_export'])
        model = options.get('tool_model')
        if model:
            model = dict(xml=model) if isinstance(model, str) else dict(model)
            options['tool_model'] = dict(model, xml=_local(model['xml']))
        out[side] = dict(arm=arm, bus=None if bus is None else str(bus), options=options)
    if not out:
        return None
    cameras = _section(s, 'cameras')
    if cameras:
        out['cameras'] = cameras
    if s.get('privacy') is not None:
        out['privacy'] = _section(s, 'privacy')
    return one_arm_on_the_left(out)


def effective(stored=None):
    """What runs: robot.yaml when it names arms, else what earlier flags stored."""
    return to_hardware(load(warn=lambda _: None)) or stored


def store_token(file: Path, token: str):
    file.parent.mkdir(parents=True, exist_ok=True)
    file.touch(mode=0o600, exist_ok=True); os.chmod(file, 0o600)
    file.write_text(token + '\n')


def token_path(a, s=None) -> Path:
    from .. import robot
    s = load(warn=lambda _: None) if s is None else s
    return Path(getattr(a, 'key_file', None) or s.get('token_file') or Path(robot.HOME) / 'token').expanduser()


def _token_server(a=None):
    from .. import robot
    return ((a is not None and getattr(a, 'server', None)) or robot.SERVER).rstrip('/')


def given_token():
    """A key given in the environment: ROBORUN_API_KEY (a Roborun key, rr_live_...), or ROBORUN_TOKEN (also an older access token)."""
    return (os.environ.get('ROBORUN_API_KEY') or os.environ.get('ROBORUN_TOKEN') or '').strip()


def _ask_token(file: Path, server=None):
    from .. import robot
    token = given_token()
    if not token:
        from .. import login as signin
        started = signin.pending(file)
        if started is not None:                      # `roborun login` printed the link: wait for its approval here
            print(f"Waiting for this computer's sign-in to be approved: {started['url']} (code {started['user_code']})", flush=True)
            signin.finish(started)
            return
    if not token and sys.stdin.isatty():
        token = getpass.getpass('Paste your Roborun key (it will not show), or press Enter to sign in with your browser: ').strip()
        if not token:                                # the device sign-in of `roborun login`, right here
            from ..login import login
            login(server or _token_server(), file)
            return
    if not token:
        raise SystemExit(f'no key: run roborun login, then roborun up again (the key is saved in {file}). Or pass --key-file FILE (a file holding your '
                         'rr_live_... key, chmod 600), or set ROBORUN_API_KEY. Roborun is invite-only: '
                         "if you don't have a key, email support@roborun.dev")
    if robot.check_token(server or _token_server(), token) is False:         # never saved: the right one must still be able to follow
        raise SystemExit('that access token was not accepted: check it and try again')
    store_token(file, token)


def settle_token(file, server):
    """A configured robot: a ROBORUN_API_KEY / ROBORUN_TOKEN that works replaces the stored one; a stored one the service refuses stops here."""
    from .. import robot
    file = Path(file)
    given = given_token()
    if not given and not file.is_file():
        from .. import login as signin
        started = signin.pending(file)
        if started is not None:                      # `roborun login` printed the link: wait for its approval here
            print(f"Waiting for this computer's sign-in to be approved: {started['url']} (code {started['user_code']})", flush=True)
            signin.finish(started)
    stored = file.read_text().strip() if file.is_file() else ''
    if given and given != stored:
        if robot.check_token(server, given) is False:
            raise SystemExit('the key in ROBORUN_API_KEY / ROBORUN_TOKEN was not accepted: check it and try again')
        store_token(file, given)
        return
    if stored and robot.check_token(server, stored) is False:
        raise SystemExit(f'the access token in {file} is no longer accepted. Next: run `roborun up` with the new token in '
                         'ROBORUN_API_KEY, run roborun login, or replace that file')


def _profile_for(hardware):
    """The profile name for any one or two arms, in any mix and on either side: two YAMs (yam_dual) and YAM + Metal keep their
    tuned profiles, everything else is put together from the arm catalog (arm_profiles). A name people read: robot.yaml holds
    it, and arm_profiles.runtime_profile gives the runtime's id."""
    from . import catalog, is_own_driver
    from ..arm_profiles import models, name_for
    rows = catalog()
    sides = [side for side in ('left', 'right') if hardware.get(side)]
    if not sides:
        raise SystemExit('no arms: name at least one (roborun up --left yam@can0, or --sim with --left yam)')
    arms = [hardware[side]['arm'] for side in sides]
    if any(is_own_driver(a) for a in arms):
        raise SystemExit(f'set "profile:" in {path()}: your own arm drivers need a profile (roborun profiles create --urdf ...)')
    have = set(models())
    names = [a if a in have else rows.get(a, {}).get('profile') for a in arms]
    if names == ['yam', 'yam']:
        return 'yam_dual'
    if names == ['yam', 'metal_arm']:
        return 'yam_metal'
    missing = [a for a, n in zip(arms, names) if n not in have]
    if missing:
        raise SystemExit(f"no model for {', '.join(missing)} yet: roborun hw arms lists the arms that have one; "
                         f'for other arms set "profile:" in {path()} (roborun profiles, or roborun profiles create --urdf ...)')
    from ..arm_profiles import shown_profile
    return shown_profile(name_for(*names))                     # one YAM is `yam` (the runtime's yam_solo)


def one_arm_on_the_left(hardware):
    """A single arm runs in the first slot whichever side it was given on (the slots are just names for the station)."""
    if isinstance(hardware, dict) and hardware.get('right') and not hardware.get('left'):
        hardware = dict(hardware); hardware['left'] = hardware.pop('right')
        cams = hardware.get('cameras')
        if isinstance(cams, dict) and cams.get('right') and not cams.get('left'):   # its wrist camera goes with it
            cams = dict(cams); cams['left'] = cams.pop('right'); hardware['cameras'] = cams
        if isinstance(cams, dict):                        # and so does every card written for the right gripper
            from ..camera_card import move_right_to_left
            kept = {k: v for k, v in cams.items() if k == 'right'}     # a right camera that stayed keeps its own card
            moved = move_right_to_left({k: v for k, v in cams.items() if k != 'right'})
            hardware['cameras'] = {k: kept[k] if k in kept else moved[k] for k in cams}
    return hardware


def scene_of(value):
    return 'cup' if value is True else str(value)


def _wants_setup(a, s):
    """Nothing says what to run: no arms in robot.yaml, no flags, no simulation asked for, and no earlier setup."""
    from .. import robot
    if to_hardware(s) or a.left or a.right or a.sim or a.rig or a.profile or a.camera or getattr(a, 'arm_options', None) \
            or getattr(a, 'body', None) or getattr(a, 'body_file', None):
        return False
    return not robot.CONFIG.exists()


def body_setting(s):
    """robot.yaml's body: (your own body class) as up() takes it: {adapter, base_dir, profile?, options...}, or None."""
    value = s.get('body')
    if not value:
        return None
    if isinstance(value, str):
        value = dict(adapter=value)
    if not isinstance(value, dict) or not value.get('adapter'):
        raise SystemExit(f'{path().name}: body is your body class (body: my_body.py:MyBody), or a section with "adapter:"')
    out = dict(value, base_dir=str(path().parent))
    if not out.get('profile') and s.get('profile'):
        out['top_profile'] = str(s['profile'])   # robot.yaml's profile: names the body's profile when neither it nor the class does
    return out


def arms_named(a):
    """True when the command line names arms: --rig, --left, --right, or --sim with an arm scene (cup, ...; not a body's room or
    empty). robot.yaml's `sim:` does not count: it says how this robot runs, not what to run now."""
    from ..body.profiles import SCENES
    sim = getattr(a, 'sim', None)
    return bool(getattr(a, 'rig', None) or getattr(a, 'left', None) or getattr(a, 'right', None)
                or (sim and sim not in SCENES and not getattr(a, 'sim_from_file', False)))


def _apply_file(a, s):
    """robot.yaml's settings where no flag gives them. Its body: goes on a.body_file, not a.body, and its section's profile is not
    put on a.profile here: it is the body class of its own profile only, and body.up.from_file() decides whether this `roborun up`
    runs it (--profile of another body, or arms named: no). With arms named on the command line, a body profile in robot.yaml's
    profile: is left out as well, so --rig, --left/--right and --sim cup run arms."""
    from ..arm_profiles import user_profile
    from ..body.profiles import is_body
    a.body_file = body_setting(s)
    if not a.profile and s.get('profile'):
        name = user_profile(str(s['profile']))
        if not (arms_named(a) and is_body(name)):
            a.profile = name
    if s.get('sim') and not a.sim and not getattr(a, 'no_sim', False):
        a.sim = scene_of(s['sim'])
        a.sim_from_file = True
    return s


def setup_if_needed(a):
    """The guided setup, when nothing says what to run. It comes before every check `roborun up` makes."""
    for name in ('profile', 'rig', 'robot', 'key_file', 'sim', 'left', 'right', 'camera', 'body', 'body_file'):
        if not hasattr(a, name):
            setattr(a, name, None)
    s = _apply_file(a, load())
    if _wants_setup(a, s):
        from ..guided import enter
        enter(a)
        _apply_file(a, load())


def first_run(a):
    """What a bare `roborun up` needs. With no arms named: the guided setup on a terminal, written instructions without one.
    A simulated arm starts only when --sim or `sim:` in robot.yaml asks for it."""
    from .. import robot
    created = not path().exists()
    file = ensure_file()
    setup_if_needed(a)
    s = _apply_file(a, load())
    if robot.CONFIG.exists():
        return
    a.robot = a.robot or s.get('robot') or default_robot_name()
    key = token_path(a, s)
    if key.is_file():
        settle_token(key, _token_server(a))
    else:
        _ask_token(key, _token_server(a))
    a.key_file = str(key)
    if a.profile or a.rig or getattr(a, 'body', None) or (getattr(a, 'body_file', None) and not arms_named(a)):
        return
    hardware = to_hardware(s)
    if hardware:
        a.profile = _profile_for(hardware)
    elif a.sim and (a.left or a.right):             # simulate the arms named: --sim cup --left yam --right maker_arm
        from . import parse_spec
        a.profile = _profile_for(one_arm_on_the_left({side: dict(arm=parse_spec(getattr(a, side))[0]) for side in ('left', 'right')
                                                      if getattr(a, side)}))
    elif a.sim:
        from ..arm_profiles import SIM_DEFAULT
        a.profile = SIM_DEFAULT                     # two simulated YAMs: the scene every task is proven on
    if created:
        print(f'Your settings: {file}')


def default_robot_name():
    return re.sub(r'[^a-z0-9-]+', '-', socket.gethostname().split('.')[0].lower()).strip('-') or 'my-robot'


# -- editing robot.yaml: only what the guided setup was told, everything else stays as the person left it ---------------------
def _scalar(value):
    import yaml
    return yaml.safe_dump(value, default_flow_style=True).replace('\n...', '').strip()


def _indent(text):
    return len(text) - len(text.lstrip(' '))


def _is_filler(text):
    return not text.strip() or text.lstrip().startswith('#')


def _block(lines, key, start=0, end=None, indent=0):
    """(first, past-last) of the live `key:` entry at `indent`; trailing comments and blank lines stay outside it."""
    end = len(lines) if end is None else end
    pattern = re.compile(' ' * indent + re.escape(key) + r'\s*:(\s|$)')
    for i in range(start, end):
        if pattern.match(lines[i]):
            last = i
            for j in range(i + 1, end):
                if _is_filler(lines[j]):
                    continue
                if _indent(lines[j]) <= indent:
                    break
                last = j
            return i, last + 1
    return None


def _set_value(line, value):
    head, _, rest = line.partition(':')
    match = re.search(r'\s+#.*$', rest)
    return f'{head}: {_scalar(value)}' + (match[0] if match else '')


def _comment_out(lines, first, last):
    lines[first:last] = ['# ' + text if text.strip() else text for text in lines[first:last]]


def _set_top(lines, key, value):
    found = _block(lines, key)
    if value is None:
        if found:
            _comment_out(lines, *found)
    elif found:
        lines[found[0]] = _set_value(lines[found[0]], value)
    else:
        sample = next((i for i, text in enumerate(lines) if re.match(rf'#\s*{key}\s*:', text)), None)
        lines.insert(len(lines) if sample is None else sample + 1, f'{key}: {_scalar(value)}')


def _child_indent(lines, first, last, default):
    return next((_indent(lines[i]) for i in range(first + 1, last) if not _is_filler(lines[i])), default)


def _set_section(lines, key, changes):
    """arms / cameras: per entry, only the named fields change."""
    found = _block(lines, key)
    if found is None:
        kept = {k: v for k, v in changes.items() if v is not None}
        if not kept:
            return
        sample = next((i for i, text in enumerate(lines) if re.match(rf'#\s*{key}\s*:', text)), None)
        at = len(lines)
        if sample is not None:
            at = next((i for i in range(sample, len(lines)) if not lines[i].startswith('#')), len(lines))
        lines[at:at] = ['', *_dump({key: kept}).splitlines()]
        return
    for name, value in changes.items():
        first, last = _block(lines, key)
        width = _child_indent(lines, first, last, 2)
        entry = _block(lines, name, first + 1, last, width)
        if value is None:
            if entry:
                _comment_out(lines, *entry)
        elif entry is None:
            lines[last:last] = [' ' * width + text for text in _dump({name: value}).splitlines()]
        elif isinstance(value, dict):
            at, end = entry
            inner = _child_indent(lines, at, end, width + 2)
            for field, item in value.items():
                row = _block(lines, field, at + 1, end, inner)
                if row:
                    lines[row[0]] = _set_value(lines[row[0]], item)
                else:
                    lines.insert(at + 1, f"{' ' * inner}{field}: {_scalar(item)}")
                    end += 1
        else:
            at, end = entry
            source = _block(lines, 'source', at + 1, end, _child_indent(lines, at, end, width + 2)) if end > at + 1 else None
            lines[source[0] if source else at] = _set_value(lines[source[0] if source else at], value)


def _dump(value):
    import yaml
    return yaml.safe_dump(value, default_flow_style=False, sort_keys=False)


def _merged(old, changes):
    out = json.loads(json.dumps(old))
    for key, value in changes.items():
        if key not in ('arms', 'cameras'):
            out.pop(key, None) if value is None else out.__setitem__(key, value)
            continue
        section = dict(out.get(key) or {})
        for name, item in value.items():
            if item is None:
                section.pop(name, None)
            elif isinstance(item, dict):
                section[name] = dict(section.get(name) or {}, **item)
            elif isinstance(section.get(name), dict):
                section[name] = dict(section[name], source=item)
            else:
                section[name] = item
        if section:
            out[key] = section
        else:
            out.pop(key, None)
    return out


def _clean(value):
    return {k: v for k, v in (value or {}).items() if v not in (None, {})}


def update_file(changes):
    """Applies {robot, profile, sim: value or None; arms: {side: {arm, address} or None}; cameras: {role: source or None}} to
    robot.yaml. Comments and every other setting stay. The previous file is kept as robot.yaml.before-setup."""
    import yaml
    file = ensure_file()
    before = file.read_text()
    old = yaml.safe_load(before) or {}
    wanted = _clean(_merged(old, changes))
    lines = before.splitlines()
    for key, value in changes.items():
        (_set_section if key in ('arms', 'cameras') else _set_top)(lines, key, value)
    text = '\n'.join(lines) + '\n'
    def good(candidate):
        try:
            return _clean(yaml.safe_load(candidate)) == wanted
        except yaml.YAMLError:
            return False
    if not good(text):                                     # a hand-written shape (flow style...): those sections are rewritten whole
        lines = before.splitlines()
        for key in changes:
            found = _block(lines, key)
            if key in wanted:
                block = _dump({key: wanted[key]}).splitlines()
                if found:
                    lines[found[0]:found[1]] = block
                else:
                    lines += block
            elif found:
                _comment_out(lines, *found)
        text = '\n'.join(lines) + '\n'
        if not good(text):
            raise SystemExit(f'{file} could not be updated safely; it was left as it is. Please add these settings by hand:\n'
                             + _dump(_clean(dict((k, v) for k, v in wanted.items() if k in changes))))
    if text != before:
        file.with_name(file.name + '.before-setup').write_text(before)
        temp = file.with_name(file.name + '.tmp')
        temp.write_text(text)
        temp.replace(file)
    return file


def cmd_settings(a) -> int:
    file = ensure_file()
    if a.edit:
        import shlex
        import shutil
        editor = shlex.split(os.environ.get('VISUAL') or os.environ.get('EDITOR') or 'nano')
        if not shutil.which(editor[0]):
            raise SystemExit(f'no editor {editor[0]!r} here. Set EDITOR, or open this file yourself: {file}')
        os.execvp(editor[0], [*editor, str(file)])
    if a.show:
        print(json.dumps(effective(), indent=1))
        return 0
    print(file)
    return 0


def add_parser(sub):
    s = sub.add_parser('settings', help='this robot\'s local settings file (arms, cameras, limits, your own rules)')
    s.add_argument('--edit', action='store_true', help='open it in your editor')
    s.add_argument('--show', action='store_true', help='print what roborun up will use')
    s.set_defaults(func=cmd_settings)
