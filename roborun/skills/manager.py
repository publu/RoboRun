"""Install RoboRun skills from GitHub — fork, vibecode, `skill add`.

There is no registry to publish to: a skill is a GitHub repo (fork of
roborun-skill-template) with a `skill.py` at its root. Install pins the
exact commit; the lockfile is the trust boundary.

    ros-agent skill add  owner/repo            # or a full URL, or @ref
    ros-agent skill add  ./local-skill-dir     # symlinked, for development
    ros-agent skill list
    ros-agent skill update <id>
    ros-agent skill remove <id>

Layout:
    ~/.roborun/skills/<skill_id>/     git clone, checked out at the pinned SHA
    ~/.roborun/skills.lock            JSON: id → {repo, sha, version, file}

Validation never executes the skill. The skill file is parsed (AST only)
for the required exports (SKILL_ID, SKILL_NAME, SKILL_VERSION, register())
and the optional REQUIRES constraint against the running roborun version.
Execution happens where it always did — at load time, in load_skills() —
and only after the working tree still matches the pinned SHA.
"""
from __future__ import annotations

import ast
import json
import re
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

from roborun import __version__

SKILLS_ROOT = Path.home() / ".roborun" / "skills"
LOCK_PATH = Path.home() / ".roborun" / "skills.lock"

_GH_SPEC = re.compile(r"^[\w.-]+/[\w.-]+(@[\w./-]+)?$")


# ── lockfile ─────────────────────────────────────────────────────────────

def read_lock() -> dict[str, dict]:
    if not LOCK_PATH.exists():
        return {}
    try:
        return json.loads(LOCK_PATH.read_text())
    except Exception:
        return {}


def _write_lock(lock: dict[str, dict]) -> None:
    LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    LOCK_PATH.write_text(json.dumps(lock, indent=1, sort_keys=True))


# ── validation (AST only — installing must not run the code) ─────────────

def _version_tuple(v: str) -> tuple[int, ...]:
    return tuple(int(x) for x in re.findall(r"\d+", v)[:3] or [0])


def _requires_ok(constraint: str, version: str = __version__) -> bool:
    """Minimal '>=0.11,<1' style check against the running roborun."""
    have = _version_tuple(version)
    for clause in constraint.split(","):
        m = re.match(r"\s*(>=|<=|==|<|>)\s*([\d.]+)\s*$", clause)
        if not m:
            continue  # unknown clause: don't block on it
        op, want_s = m.groups()
        want = _version_tuple(want_s)
        ok = {">=": have >= want, "<=": have <= want, "==": have[:len(want)] == want,
              "<": have < want, ">": have > want}[op]
        if not ok:
            return False
    return True


def find_skill_file(root: Path) -> Path | None:
    """`skill.py` at the repo root, else the first root .py exporting SKILL_ID."""
    direct = root / "skill.py"
    if direct.exists():
        return direct
    for p in sorted(root.glob("*.py")):
        if "SKILL_ID" in p.read_text(errors="ignore"):
            return p
    return None


def validate_skill(root: Path) -> dict[str, Any]:
    """Structural validation of a skill working tree; returns a report."""
    issues: list[str] = []
    skill_file = find_skill_file(root)
    if skill_file is None:
        return {"ok": False, "issues": ["no skill.py (or any root .py exporting SKILL_ID)"]}
    try:
        tree = ast.parse(skill_file.read_text())
    except SyntaxError as exc:
        return {"ok": False, "file": skill_file.name,
                "issues": [f"syntax error: {exc}"]}

    consts: dict[str, Any] = {}
    has_register = False
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name) \
                and isinstance(node.value, ast.Constant):
            consts[node.targets[0].id] = node.value.value
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) \
                and node.name == "register":
            has_register = True

    for key in ("SKILL_ID", "SKILL_NAME", "SKILL_VERSION"):
        if not isinstance(consts.get(key), str):
            issues.append(f"missing required constant {key}")
    if not has_register:
        issues.append("missing register(registry) function")
    if isinstance(consts.get("SKILL_ID"), str) and \
            not re.match(r"^[a-z0-9][a-z0-9-]*$", consts["SKILL_ID"]):
        issues.append("SKILL_ID must be a lowercase-kebab slug")

    requires = consts.get("REQUIRES")
    if isinstance(requires, str) and not _requires_ok(requires):
        issues.append(f"REQUIRES '{requires}' not satisfied by roborun {__version__}")

    return {"ok": not issues, "skill_id": consts.get("SKILL_ID"),
            "name": consts.get("SKILL_NAME"), "version": consts.get("SKILL_VERSION"),
            "requires": requires, "file": skill_file.name, "issues": issues}


# ── git plumbing ─────────────────────────────────────────────────────────

def _git(*args: str, cwd: Path | None = None) -> str:
    out = subprocess.run(["git", *args], cwd=cwd, capture_output=True,
                         text=True, timeout=120)
    if out.returncode != 0:
        raise RuntimeError(out.stderr.strip() or f"git {' '.join(args)} failed")
    return out.stdout.strip()


def head_sha(repo_dir: Path) -> str | None:
    try:
        return _git("rev-parse", "HEAD", cwd=repo_dir)
    except Exception:
        return None


def _resolve_url(spec: str) -> tuple[str, str | None]:
    """spec → (clone_url, ref). Accepts owner/repo[@ref] and full URLs."""
    ref = None
    if "@" in spec and not spec.startswith("git@"):
        spec, ref = spec.rsplit("@", 1)
    if _GH_SPEC.match(spec + (f"@{ref}" if ref else "")) and "://" not in spec:
        return f"https://github.com/{spec}", ref
    return spec, ref


# ── operations ───────────────────────────────────────────────────────────

def add(spec: str) -> dict[str, Any]:
    """Install a skill from GitHub (pinned) or a local directory (symlinked)."""
    local = Path(spec).expanduser()
    if local.is_dir():
        return _add_local(local)

    url, ref = _resolve_url(spec)
    SKILLS_ROOT.mkdir(parents=True, exist_ok=True)
    tmp = SKILLS_ROOT / f"_incoming_{int(time.time())}"
    try:
        _git("clone", "--quiet", url, str(tmp))
        if ref:
            _git("checkout", "--quiet", ref, cwd=tmp)
        report = validate_skill(tmp)
        if not report["ok"]:
            return {"ok": False, "error": "validation failed", **report}
        skill_id = report["skill_id"]
        dest = SKILLS_ROOT / skill_id
        if dest.exists():
            shutil.rmtree(dest)
        tmp.rename(dest)
        sha = head_sha(dest)
        lock = read_lock()
        lock[skill_id] = {
            "repo": url, "ref": ref, "sha": sha,
            "version": report["version"], "name": report["name"],
            "file": report["file"], "requires": report.get("requires"),
            "added_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        _write_lock(lock)
        return {"ok": True, "skill_id": skill_id, "sha": sha, **report}
    finally:
        if tmp.exists():
            shutil.rmtree(tmp, ignore_errors=True)


def _add_local(path: Path) -> dict[str, Any]:
    """Dev install: symlink, no pin — marked local so load skips the SHA check."""
    report = validate_skill(path)
    if not report["ok"]:
        return {"ok": False, "error": "validation failed", **report}
    skill_id = report["skill_id"]
    SKILLS_ROOT.mkdir(parents=True, exist_ok=True)
    dest = SKILLS_ROOT / skill_id
    if dest.is_symlink() or dest.exists():
        dest.unlink() if dest.is_symlink() else shutil.rmtree(dest)
    dest.symlink_to(path.resolve())
    lock = read_lock()
    lock[skill_id] = {
        "repo": str(path.resolve()), "local": True, "sha": None,
        "version": report["version"], "name": report["name"], "file": report["file"],
        "added_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    _write_lock(lock)
    return {"ok": True, "skill_id": skill_id, "local": True, **report}


def remove(skill_id: str) -> dict[str, Any]:
    lock = read_lock()
    if skill_id not in lock:
        return {"ok": False, "error": f"not installed: {skill_id}"}
    dest = SKILLS_ROOT / skill_id
    if dest.is_symlink():
        dest.unlink()
    elif dest.exists():
        shutil.rmtree(dest)
    del lock[skill_id]
    _write_lock(lock)
    return {"ok": True, "removed": skill_id}


def update(skill_id: str) -> dict[str, Any]:
    """Fetch the skill's default branch, re-validate, re-pin."""
    lock = read_lock()
    entry = lock.get(skill_id)
    if entry is None:
        return {"ok": False, "error": f"not installed: {skill_id}"}
    if entry.get("local"):
        return {"ok": True, "skill_id": skill_id, "local": True,
                "note": "local symlink — nothing to update"}
    return add(entry["repo"] + (f"@{entry['ref']}" if entry.get("ref") else ""))


def installed() -> list[dict[str, Any]]:
    """Lock entries + an integrity verdict for each working tree."""
    out = []
    for skill_id, entry in sorted(read_lock().items()):
        dest = SKILLS_ROOT / skill_id
        if entry.get("local"):
            state = "local" if dest.exists() else "missing"
        elif not dest.exists():
            state = "missing"
        elif head_sha(dest) != entry.get("sha"):
            state = "sha-mismatch"
        else:
            state = "pinned"
        out.append({"id": skill_id, **entry, "state": state})
    return out


def verified_skill_paths() -> list[tuple[str, Path]]:
    """(skill_id, skill_file) for every installed skill that passes the pin
    check — the loader's entry point. Tampered or missing trees are skipped
    with a lock state the CLI surfaces, never silently loaded."""
    out = []
    for entry in installed():
        if entry["state"] not in ("pinned", "local"):
            continue
        path = SKILLS_ROOT / entry["id"] / entry["file"]
        if path.exists():
            out.append((entry["id"], path))
    return out


# ── CLI ──────────────────────────────────────────────────────────────────

def cli(argv: list[str] | None = None) -> int:
    import argparse
    p = argparse.ArgumentParser(
        prog="ros-agent skill",
        description="Install skills from GitHub: fork roborun-skill-template, "
                    "vibecode, `skill add owner/repo`.")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="installed skills + pin state")
    for name, hlp in (("add", "owner/repo[@ref], URL, or local dir"),
                      ("remove", "installed skill id"),
                      ("update", "installed skill id"),
                      ("validate", "path to a skill working tree")):
        sp = sub.add_parser(name, help=hlp)
        sp.add_argument("target")
    args = p.parse_args(argv)

    if args.cmd == "list":
        rows = installed()
        if not rows:
            print("no skills installed — try: ros-agent skill add "
                  "publu/roborun-skill-template")
            return 0
        for r in rows:
            pin = (r.get("sha") or "")[:12] or r["state"]
            print(f"{r['id']:<24} v{r.get('version', '?'):<8} [{r['state']}] {pin}")
        return 0
    if args.cmd == "validate":
        result = validate_skill(Path(args.target).expanduser())
    elif args.cmd == "add":
        result = add(args.target)
    elif args.cmd == "remove":
        result = remove(args.target)
    else:
        result = update(args.target)
    print(json.dumps(result, indent=1))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(cli())
