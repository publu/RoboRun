"""Skills plugin system for RoboRun.

Skills are Python packages or local directories that register MCP tools and
agent behaviors at startup. Each skill provides a `register(registry)` function
that adds tools, behaviors, and config to the shared registry.

Loading order:
  1. Built-in skills from roborun/skills/ directory
  2. GitHub-installed skills from ~/.roborun/skills/ (`ros-agent skill add`,
     pinned by commit SHA in ~/.roborun/skills.lock — see manager.py)
  3. pip-installed packages listed in ROBORUN_SKILL_PACKAGES env var
  4. Local directories listed in ROBORUN_SKILL_PATHS env var
  5. Skills listed in .roborun/skills.yaml config

Skill interface — a skill module must export:

    SKILL_ID: str           — unique slug e.g. "follow-me"
    SKILL_NAME: str         — display name
    SKILL_VERSION: str      — semver

    def register(registry: SkillRegistry) -> None:
        registry.add_tool(name, description, input_schema, handler)
        registry.add_behavior(name, description, handler)
"""

from __future__ import annotations

import importlib
import logging
import os
import sys
from pathlib import Path
from typing import Any, Callable

log = logging.getLogger(__name__)

# ── Skill Registry ─────────────────────────────────────────────────────────────


class SkillRegistry:
    def __init__(self) -> None:
        self._tools: dict[str, dict] = {}
        self._behaviors: dict[str, dict] = {}
        self._skills: dict[str, dict] = {}
        self._config: dict[str, dict] = {}

    def add_tool(self, name: str, description: str,
                 input_schema: dict, handler: Callable[[dict], dict],
                 skill_id: str = "") -> None:
        self._tools[name] = {
            "name": name,
            "description": description,
            "inputSchema": input_schema,
            "handler": handler,
            "skill_id": skill_id,
        }

    def add_behavior(self, name: str, description: str,
                     handler: Callable[[dict], Any],
                     skill_id: str = "") -> None:
        self._behaviors[name] = {
            "name": name,
            "description": description,
            "handler": handler,
            "skill_id": skill_id,
        }

    def register_skill(self, skill_id: str, name: str, version: str,
                       description: str = "") -> None:
        self._skills[skill_id] = {
            "id": skill_id,
            "name": name,
            "version": version,
            "description": description,
        }

    def set_config(self, skill_id: str, config: dict) -> None:
        self._config[skill_id] = config

    def get_config(self, skill_id: str) -> dict:
        return self._config.get(skill_id, {})

    @property
    def tools(self) -> dict[str, dict]:
        return dict(self._tools)

    @property
    def behaviors(self) -> dict[str, dict]:
        return dict(self._behaviors)

    @property
    def skills(self) -> dict[str, dict]:
        return dict(self._skills)

    def get_mcp_tools(self) -> list[dict]:
        return [
            {"name": t["name"], "description": t["description"],
             "inputSchema": t["inputSchema"]}
            for t in self._tools.values()
        ]

    def handle_tool_call(self, name: str, args: dict) -> dict | None:
        tool = self._tools.get(name)
        if not tool:
            return None
        try:
            return tool["handler"](args)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def execute_behavior(self, name: str, args: dict) -> Any | None:
        behavior = self._behaviors.get(name)
        if not behavior:
            return None
        return behavior["handler"](args)


# ── Global registry singleton ──────────────────────────────────────────────────

_registry = SkillRegistry()


def get_registry() -> SkillRegistry:
    return _registry


# ── Skill loading ──────────────────────────────────────────────────────────────

def _load_module_skill(module_name: str) -> bool:
    try:
        mod = importlib.import_module(module_name)
        skill_id = getattr(mod, "SKILL_ID", module_name.split(".")[-1])
        skill_name = getattr(mod, "SKILL_NAME", skill_id)
        skill_version = getattr(mod, "SKILL_VERSION", "0.0.0")
        _registry.register_skill(skill_id, skill_name, skill_version)

        register_fn = getattr(mod, "register", None)
        if callable(register_fn):
            register_fn(_registry)
            log.info("Loaded skill: %s v%s (%d tools)",
                     skill_name, skill_version,
                     sum(1 for t in _registry.tools.values() if t.get("skill_id") == skill_id))
            return True
        else:
            log.warning("Skill %s has no register() function", module_name)
            return False
    except Exception as exc:
        log.warning("Failed to load skill %s: %s", module_name, exc)
        return False


def _load_file_skill(path: Path, module_name: str) -> bool:
    """Load a skill file under a unique module name (every installed skill
    is a `skill.py`, so plain imports would collide)."""
    import importlib.util
    try:
        spec = importlib.util.spec_from_file_location(module_name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = mod
        spec.loader.exec_module(mod)
    except Exception as exc:
        log.warning("Failed to load skill %s: %s", path, exc)
        return False
    return _register_module(mod, module_name)


def _register_module(mod, fallback_id: str) -> bool:
    skill_id = getattr(mod, "SKILL_ID", fallback_id)
    skill_name = getattr(mod, "SKILL_NAME", skill_id)
    skill_version = getattr(mod, "SKILL_VERSION", "0.0.0")
    _registry.register_skill(skill_id, skill_name, skill_version)
    register_fn = getattr(mod, "register", None)
    if not callable(register_fn):
        log.warning("Skill %s has no register() function", skill_id)
        return False
    try:
        register_fn(_registry)
    except Exception as exc:
        log.warning("Skill %s register() failed: %s", skill_id, exc)
        return False
    log.info("Loaded skill: %s v%s", skill_name, skill_version)
    return True


def _load_path_skill(path: str) -> bool:
    p = Path(path).resolve()
    if not p.exists():
        log.warning("Skill path not found: %s", path)
        return False
    if p.is_dir():
        sys.path.insert(0, str(p.parent))
        return _load_module_skill(p.name)
    elif p.suffix == ".py":
        sys.path.insert(0, str(p.parent))
        return _load_module_skill(p.stem)
    return False


def load_skills() -> int:
    loaded = 0

    # 1. Built-in skills (sibling .py files in this package)
    builtin_dir = Path(__file__).parent
    for p in sorted(builtin_dir.glob("*.py")):
        if p.name.startswith("_") or p.name == "manager.py":
            continue
        if _load_module_skill(f"roborun.skills.{p.stem}"):
            loaded += 1

    # 2. GitHub-installed skills, pin-verified against the lockfile
    try:
        from roborun.skills.manager import verified_skill_paths
        for skill_id, path in verified_skill_paths():
            if _load_file_skill(path, f"roborun_skill_{skill_id.replace('-', '_')}"):
                loaded += 1
    except Exception as exc:
        log.warning("Installed-skill scan failed: %s", exc)

    # 3. pip-installed packages from env
    packages = os.environ.get("ROBORUN_SKILL_PACKAGES", "")
    for pkg in packages.split(","):
        pkg = pkg.strip()
        if pkg and _load_module_skill(pkg):
            loaded += 1

    # 4. Local paths from env
    paths = os.environ.get("ROBORUN_SKILL_PATHS", "")
    for path in paths.split(","):
        path = path.strip()
        if path and _load_path_skill(path):
            loaded += 1

    # 5. Config file
    config_file = Path.cwd() / ".roborun" / "skills.yaml"
    if config_file.exists():
        try:
            import yaml
            config = yaml.safe_load(config_file.read_text()) or {}
            for pkg in config.get("packages", []):
                if _load_module_skill(pkg):
                    loaded += 1
            for path in config.get("paths", []):
                if _load_path_skill(path):
                    loaded += 1
            for skill_id, skill_config in config.get("config", {}).items():
                _registry.set_config(skill_id, skill_config)
        except Exception as exc:
            log.warning("Failed to read skills.yaml: %s", exc)

    log.info("Loaded %d skill(s), %d total tools, %d behaviors",
             loaded, len(_registry.tools), len(_registry.behaviors))
    return loaded
