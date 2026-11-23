"""Drives the real `roborun` CLI through a pseudo-terminal, next to the simulated hardware process."""
import fcntl
import json
import os
import pty
import re
import select
import shlex
import signal
import subprocess
import sys
import termios
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SDK = HERE.parents[1]
PACKAGES = SDK.parent
PY = sys.executable
CLI = shlex.split(os.environ['GUIDED_CLI']) if os.environ.get('GUIDED_CLI') else [PY, '-m', 'roborun_sdk.cli']   # an installed `roborun`, or this tree
TOKEN = 'rr-test-token'


class Rig:
    def __init__(self, root, arms=(), cameras=None):
        self.root = Path(root); self.root.mkdir(parents=True, exist_ok=True)
        self.home = self.root / 'home'; self.home.mkdir(exist_ok=True)
        self.config = self.home / '.config' / 'roborun'
        (self.root / 'tmp').mkdir(exist_ok=True); (self.root / 'run').mkdir(exist_ok=True)
        spec = self.root / 'spec.json'
        spec.write_text(json.dumps(dict(arms=list(arms), cameras=cameras or {})))
        self.base_env = dict(PATH=os.environ['PATH'], HOME=str(self.home), TMPDIR=str(self.root / 'tmp'), LANG='C.UTF-8', TERM='dumb',
                             XDG_RUNTIME_DIR=str(self.root / 'run'), ROBORUN_FAKE_HW=str(self.root), PYTHONUNBUFFERED='1',
                             PYTHONPATH=os.pathsep.join(map(str, (HERE, HERE.parent) if os.environ.get('GUIDED_CLI')
                                                            else (HERE, SDK / 'src', PACKAGES / 'roborun-station' / 'src', SDK / 'tests'))))
        self.hw = subprocess.Popen([PY, str(HERE / 'hw.py'), str(spec), str(self.root)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=open(self.root / 'hw.log', 'w'), text=True, env=self.base_env)
        first = self.hw.stdout.readline()
        if not first:
            raise RuntimeError('hardware did not start: ' + (self.root / 'hw.log').read_text()[-2000:])
        self.addresses = json.loads(first)['addresses']
        self.has_cameras = bool(cameras)
        self.arm_at = {self.addresses[row['name']]: row['arm'] for row in arms}
        self.api = subprocess.Popen([PY, str(HERE / 'fake_api.py'), '0'], stdout=subprocess.PIPE, text=True, env=self.base_env)
        self.env = dict(self.base_env, ROBORUN_SERVER=f'http://127.0.0.1:{self.api.stdout.readline().strip()}')
        self.sessions = []

    def cmd(self, text):
        self.hw.stdin.write(text + '\n'); self.hw.stdin.flush()
        return json.loads(self.hw.stdout.readline())

    def approve(self):
        """The person opens the sign-in link and approves this computer."""
        import urllib.request
        with urllib.request.urlopen(self.env['ROBORUN_SERVER'] + '/approve?code=ABCD-1234', timeout=10) as r:
            return json.load(r)

    def sign_ins(self):
        import urllib.request
        with urllib.request.urlopen(self.env['ROBORUN_SERVER'] + '/test/sign-in', timeout=10) as r:
            return json.load(r)

    def enabled(self):
        return {k: v for k, v in self.cmd('state')['enabled'].items() if v}

    def enables(self):
        return self.cmd('state')['enables']

    def direct_frames(self):
        return self.cmd('state')['direct']

    def roborun(self, *args, env=None):
        s = Session([*CLI, *args], dict(self.env, **(env or {})), self.home)
        self.sessions.append(s)
        return s

    def plain(self, *args, env=None, stdin=subprocess.DEVNULL, timeout=120):
        return subprocess.run([*CLI, *args], env=dict(self.env, **(env or {})), cwd=self.home, stdin=stdin,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)

    def yaml(self):
        import yaml
        return yaml.safe_load((self.config / 'robot.yaml').read_text()) or {}

    def down(self):
        self.plain('down', timeout=90)

    def close(self):
        for s in self.sessions:
            s.kill()
        try:
            self.down()
            # `down` keeps a raised arm powered (never drops it) and its station running: these arms are simulated, so the
            # test lets them go, or every such test leaves a hardware-station process behind
            self.plain('down', '--release', timeout=60)
        except Exception:
            pass
        for link in [a for a in self.addresses.values() if a.startswith('vcan')]:
            subprocess.run(['ip', 'link', 'del', link], capture_output=True)
        for p in (self.hw, self.api):
            try:
                if p is self.hw: p.stdin.close()
                p.terminate(); p.wait(5)
            except Exception:
                p.kill()


class Session:
    def __init__(self, argv, env, cwd):
        master, slave = pty.openpty()
        def child():
            os.setsid()
            fcntl.ioctl(slave, termios.TIOCSCTTY, 0)
        self.proc = subprocess.Popen(argv, stdin=slave, stdout=slave, stderr=slave, env=env, cwd=cwd, preexec_fn=child, close_fds=True)
        os.close(slave)
        self.master, self.seen, self.at = master, '', 0
        self.sends = 0                                     # every answer typed: the number of prompts the person saw

    def _pump(self, wait):
        ready, _, _ = select.select([self.master], [], [], wait)
        if not ready:
            return False
        try:
            data = os.read(self.master, 65536)
        except OSError:
            return False
        if not data:
            return False
        self.seen += data.decode(errors='replace').replace('\r\n', '\n')
        return True

    def expect(self, pattern, timeout=60):
        rx = re.compile(pattern)
        end = time.monotonic() + timeout
        while True:
            m = rx.search(self.seen, self.at)
            if m:
                self.at = m.end()
                return m
            if time.monotonic() > end:
                raise AssertionError(f'timed out waiting for {pattern!r}\n--- screen tail ---\n{self.seen[-2500:]}')
            if not self._pump(.1) and self.proc.poll() is not None:
                while self._pump(0):
                    pass
                if (m := rx.search(self.seen, self.at)):
                    self.at = m.end()
                    return m
                raise AssertionError(f'exited ({self.proc.returncode}) waiting for {pattern!r}\n--- screen tail ---\n{self.seen[-2500:]}')

    def send(self, text=''):
        self.sends += 1
        os.write(self.master, (text + '\n').encode())

    def answer(self, pattern, text='', timeout=60):
        m = self.expect(pattern, timeout)
        self.send(text)
        return m

    def ctrl_c(self):
        time.sleep(.3)                                     # a person is never faster than the prompt reaching its read()
        os.write(self.master, b'\x03')

    def wait(self, timeout=60):
        end = time.monotonic() + timeout
        while self.proc.poll() is None and time.monotonic() < end:
            self._pump(.1)
        while self._pump(.05):
            pass
        if self.proc.poll() is None:
            raise AssertionError('still running\n' + self.seen[-2000:])
        return self.proc.returncode

    def kill(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGKILL); self.proc.wait()
        try:
            os.close(self.master)
        except OSError:
            pass
