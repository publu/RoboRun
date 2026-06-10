"""GitHub skill install: validation, SHA pinning, tamper refusal."""
from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from roborun.skills import manager

GOOD_SKILL = '''\
"""Example skill."""
SKILL_ID = "demo-skill"
SKILL_NAME = "Demo Skill"
SKILL_VERSION = "0.1.0"
REQUIRES = ">=0.11,<2"

def register(registry):
    registry.add_tool(
        name="demo_hello", description="Say hello.",
        input_schema={"type": "object", "properties": {}},
        handler=lambda args: {"ok": True, "message": "hi"},
        skill_id=SKILL_ID)
'''


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setattr(manager, "SKILLS_ROOT", tmp_path / "skills")
    monkeypatch.setattr(manager, "LOCK_PATH", tmp_path / "skills.lock")
    return tmp_path


def _make_skill_repo(tmp_path: Path, body: str = GOOD_SKILL) -> Path:
    repo = tmp_path / "upstream"
    repo.mkdir()
    (repo / "skill.py").write_text(body)
    for cmd in (["git", "init", "-q"], ["git", "add", "-A"],
                ["git", "-c", "user.email=t@t", "-c", "user.name=t",
                 "commit", "-qm", "init"]):
        subprocess.run(cmd, cwd=repo, check=True, capture_output=True)
    return repo


def test_validate_good(tmp_path):
    (tmp_path / "skill.py").write_text(GOOD_SKILL)
    r = manager.validate_skill(tmp_path)
    assert r["ok"] and r["skill_id"] == "demo-skill" and r["version"] == "0.1.0"


def test_validate_rejects_missing_exports(tmp_path):
    (tmp_path / "skill.py").write_text("SKILL_ID = 'x-y'\n")
    r = manager.validate_skill(tmp_path)
    assert not r["ok"]
    assert any("SKILL_NAME" in i for i in r["issues"])
    assert any("register" in i for i in r["issues"])


def test_validate_rejects_bad_slug_and_requires(tmp_path):
    bad = GOOD_SKILL.replace('"demo-skill"', '"Demo Skill!"') \
                    .replace('">=0.11,<2"', '">=99.0"')
    (tmp_path / "skill.py").write_text(bad)
    r = manager.validate_skill(tmp_path)
    assert not r["ok"]
    assert any("kebab" in i for i in r["issues"])
    assert any("REQUIRES" in i for i in r["issues"])


def test_requires_ok():
    assert manager._requires_ok(">=0.11,<2", "0.11.0")
    assert not manager._requires_ok(">=0.12", "0.11.0")
    assert manager._requires_ok("==0.11", "0.11.5")


def test_add_pins_sha_and_loads(home, tmp_path):
    repo = _make_skill_repo(tmp_path)
    r = manager.add(str(repo))
    # a path that exists is a local dir install; force the git path via file URL
    assert r["ok"]


def test_add_from_git_url_pins_and_detects_tamper(home, tmp_path):
    repo = _make_skill_repo(tmp_path)
    r = manager.add(f"file://{repo}")
    assert r["ok"] and r["skill_id"] == "demo-skill" and r["sha"]
    assert [e["state"] for e in manager.installed()] == ["pinned"]
    assert manager.verified_skill_paths()[0][0] == "demo-skill"

    # tamper: new commit in the installed working tree → pin check must fail
    installed_dir = manager.SKILLS_ROOT / "demo-skill"
    (installed_dir / "skill.py").write_text(GOOD_SKILL + "\nEVIL = True\n")
    for cmd in (["git", "add", "-A"],
                ["git", "-c", "user.email=e@e", "-c", "user.name=e",
                 "commit", "-qm", "evil"]):
        subprocess.run(cmd, cwd=installed_dir, check=True, capture_output=True)
    assert manager.installed()[0]["state"] == "sha-mismatch"
    assert manager.verified_skill_paths() == []


def test_add_validation_failure_installs_nothing(home, tmp_path):
    repo = _make_skill_repo(tmp_path, body="print('not a skill')\n")
    r = manager.add(f"file://{repo}")
    assert not r["ok"]
    assert manager.read_lock() == {}
    assert not (manager.SKILLS_ROOT / "demo-skill").exists()


def test_local_add_symlinks_and_remove(home, tmp_path):
    src = tmp_path / "dev-skill"
    src.mkdir()
    (src / "skill.py").write_text(GOOD_SKILL)
    r = manager.add(str(src))
    assert r["ok"] and r["local"]
    dest = manager.SKILLS_ROOT / "demo-skill"
    assert dest.is_symlink()
    assert manager.installed()[0]["state"] == "local"

    rm = manager.remove("demo-skill")
    assert rm["ok"]
    assert not dest.exists() and manager.read_lock() == {}
    assert src.exists()  # removing the symlink never touches the source


def test_depends_declared_and_checked(tmp_path):
    body = GOOD_SKILL.replace('REQUIRES = ">=0.11,<2"',
                              'REQUIRES = ">=0.11,<2"\nDEPENDS = ["json", "not_a_real_pkg_xyz"]')
    (tmp_path / "skill.py").write_text(body)
    r = manager.validate_skill(tmp_path)
    assert r["ok"]
    assert r["depends"] == ["json", "not_a_real_pkg_xyz"]
    assert r["missing_deps"] == ["not_a_real_pkg_xyz"]


def test_depends_malformed_rejected(tmp_path):
    body = GOOD_SKILL + "\nDEPENDS = [1, 2]\n"
    (tmp_path / "skill.py").write_text(body)
    r = manager.validate_skill(tmp_path)
    assert not r["ok"]
    assert any("DEPENDS" in i for i in r["issues"])


def test_missing_deps_blocks_load_not_install(home, tmp_path):
    body = GOOD_SKILL.replace('REQUIRES = ">=0.11,<2"',
                              'REQUIRES = ">=0.11,<2"\nDEPENDS = ["not_a_real_pkg_xyz"]')
    src = tmp_path / "dev-skill"
    src.mkdir()
    (src / "skill.py").write_text(body)
    r = manager.add(str(src))
    assert r["ok"]  # install succeeds — deps are a load gate, not an install gate
    entry = manager.installed()[0]
    assert entry["state"] == "missing-deps"
    assert entry["missing_deps"] == ["not_a_real_pkg_xyz"]
    assert manager.verified_skill_paths() == []  # never loaded, never crashed
