"""Launch/stop external robot stacks and MCP async call routes."""
from __future__ import annotations

import json
import os
import re
import shlex
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from roborun.routes import get, post, send_json, ApiError
from roborun.routes.dashboard import load_profile

ROOT = Path(__file__).resolve().parent.parent.parent
STATE_ROOT = ROOT / ".roborun"
JOB_ROOT = STATE_ROOT / "jobs"
IP_PATTERN = re.compile(r"^(?:(?:25[0-5]|2[0-4]\d|1?\d?\d)\.){3}(?:25[0-5]|2[0-4]\d|1?\d?\d)$")
NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.:/@-]+$")

JOBS: dict[str, dict[str, Any]] = {}
ACTIVE_DIMOS_JOB_ID: str | None = None

# ── MCP async tasks ──────────────────────────────────────────────────────────

_MCP_TASKS: dict[str, dict[str, Any]] = {}
_MCP_TASKS_LOCK = threading.Lock()
_MCP_TASK_TTL = 300.0


def _run_mcp_task(task_id: str, name: str, args: dict[str, Any]) -> None:
    import urllib.request
    try:
        body = json.dumps({
            "jsonrpc": "2.0", "id": 1, "method": "tools/call",
            "params": {"name": name, "arguments": args},
        }).encode()
        req = urllib.request.Request(
            "http://127.0.0.1:9990/mcp", data=body,
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            data = json.loads(resp.read().decode())
        if "error" in data:
            with _MCP_TASKS_LOCK:
                _MCP_TASKS[task_id].update(status="error", error=str(data["error"]), finished=time.time())
            return
        content = data.get("result", {}).get("content")
        if isinstance(content, list):
            text = "\n".join(
                (it.get("text", str(it)) if isinstance(it, dict) else str(it)) for it in content
            )
        elif isinstance(content, str):
            text = content
        else:
            text = json.dumps(content) if content is not None else json.dumps(data.get("result", {}))
        with _MCP_TASKS_LOCK:
            _MCP_TASKS[task_id].update(status="done", result=text, finished=time.time())
    except Exception as exc:
        with _MCP_TASKS_LOCK:
            _MCP_TASKS[task_id].update(status="error", error=f"MCP call failed: {exc}", finished=time.time())


# ── Helpers ──────────────────────────────────────────────────────────────────

def _command_env(env: dict[str, str] | None = None) -> dict[str, str]:
    merged = os.environ.copy()
    dimos_path = load_profile().get("dimosPath", "").strip()
    if dimos_path:
        venv_bin = Path(dimos_path) / ".venv" / "bin"
        if venv_bin.exists():
            merged["PATH"] = f"{venv_bin}{os.pathsep}{merged.get('PATH', '')}"
    if env:
        merged.update(env)
    return merged


def _run_command(args: list[str], env: dict[str, str] | None = None, timeout: int = 20) -> dict:
    merged = _command_env(env)
    try:
        completed = subprocess.run(args, cwd=str(ROOT), env=merged, text=True,
                                   stdin=subprocess.DEVNULL, capture_output=True, timeout=timeout, check=False)
        return {"ok": completed.returncode == 0, "code": completed.returncode,
                "command": " ".join(shlex.quote(a) for a in args),
                "stdout": completed.stdout[-6000:], "stderr": completed.stderr[-6000:]}
    except FileNotFoundError:
        return {"ok": False, "code": 127, "command": " ".join(shlex.quote(a) for a in args),
                "stdout": "", "stderr": f"{args[0]} not found on PATH."}
    except subprocess.TimeoutExpired:
        return {"ok": False, "code": 124, "command": " ".join(shlex.quote(a) for a in args),
                "stdout": "", "stderr": "Command timed out."}


def _start_job(name: str, args: list[str], env: dict[str, str] | None = None) -> dict:
    JOB_ROOT.mkdir(parents=True, exist_ok=True)
    job_id = f"{int(time.time())}-{name}"
    log_path = JOB_ROOT / f"{job_id}.log"
    log_file = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(args, cwd=str(ROOT), env=_command_env(env), text=True,
                            stdin=subprocess.DEVNULL, stdout=log_file, stderr=subprocess.STDOUT,
                            start_new_session=True)
    log_file.close()
    JOBS[job_id] = {"name": name, "process": proc, "log_path": log_path,
                    "command": " ".join(shlex.quote(a) for a in args), "started_at": time.time()}
    return {"ok": True, "started": True, "job_id": job_id, "pid": proc.pid,
            "stdout": f"Started {name}.", "stderr": ""}


# ── Routes ───────────────────────────────────────────────────────────────────

@post("/api/demo")
def demo(h, payload):
    global ACTIVE_DIMOS_JOB_ID
    result = _start_job("dimos-go2-replay", ["dimos", "--replay", "run", "unitree-go2",
                                              "-o", "rerunbridgemodule.rerun_open=none"])
    ACTIVE_DIMOS_JOB_ID = result["job_id"]
    send_json(h, 200, result)


@post("/api/launch")
def launch(h, payload):
    global ACTIVE_DIMOS_JOB_ID
    mode = str(payload.get("mode", "hardware")).strip()
    robot_ip = str(payload.get("robotIp", "")).strip()
    blueprint = str(payload.get("blueprint", "unitree-go2")).strip()
    viewer = str(payload.get("viewer", "rerun")).strip()
    args = ["dimos"]
    if mode == "replay":
        args.append("--replay")
    elif mode == "simulation":
        args.append("--simulation")
    args.extend(["--viewer", viewer, "run", blueprint])
    if payload.get("daemon", True):
        args.append("--daemon")
    env = {"ROBOT_IP": robot_ip} if mode == "hardware" and robot_ip else {}
    result = _start_job("go2-launch", args, env=env)
    ACTIVE_DIMOS_JOB_ID = result["job_id"]
    send_json(h, 200, result)


@post("/api/stop")
def stop(h, payload):
    global ACTIVE_DIMOS_JOB_ID
    if ACTIVE_DIMOS_JOB_ID and ACTIVE_DIMOS_JOB_ID in JOBS:
        job = JOBS[ACTIVE_DIMOS_JOB_ID]
        proc = job["process"]
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
            except ProcessLookupError:
                pass
    ACTIVE_DIMOS_JOB_ID = None
    result = _run_command(["dimos", "stop"], timeout=20)
    send_json(h, 200, {"ok": result.get("ok", False), "command": "stop dimOS",
                        "stdout": result.get("stdout", ""), "stderr": result.get("stderr", "")})


@post("/api/status")
def status(h, payload):
    send_json(h, 200, _run_command(["dimos", "status"], timeout=10))


@post("/api/ping")
def ping(h, payload):
    ip = str(payload.get("robotIp", "")).strip()
    if not IP_PATTERN.fullmatch(ip):
        raise ApiError(400, "Valid IP required")
    send_json(h, 200, _run_command(["ping", "-c", "2", "-W", "2", ip], timeout=8))


@post("/api/mcp/call")
def mcp_call(h, payload):
    name = str(payload.get("name", "")).strip()
    if not name:
        send_json(h, 400, {"ok": False, "error": "name required"})
        return
    args = payload.get("args") or {}
    task_id = uuid.uuid4().hex
    with _MCP_TASKS_LOCK:
        _MCP_TASKS[task_id] = {"status": "pending", "name": name, "started": time.time()}
    threading.Thread(target=_run_mcp_task, args=(task_id, name, args), daemon=True).start()
    send_json(h, 200, {"ok": True, "task_id": task_id})


@get("/api/mcp/result(?:\\?.*)?")
def mcp_result(h):
    from urllib.parse import parse_qs, urlparse
    qs = parse_qs(urlparse(h.path).query)
    tid = (qs.get("id") or [""])[0]
    if not tid:
        send_json(h, 400, {"ok": False, "error": "id required"})
        return
    cutoff = time.time() - _MCP_TASK_TTL
    with _MCP_TASKS_LOCK:
        stale = [k for k, v in _MCP_TASKS.items()
                 if v.get("status") in ("done", "error") and v.get("finished", 0) < cutoff]
        for k in stale:
            _MCP_TASKS.pop(k, None)
        state = _MCP_TASKS.get(tid)
        snapshot = dict(state) if state else None
    if snapshot is None:
        send_json(h, 404, {"ok": False, "error": "unknown task"})
        return
    snapshot["elapsed"] = round(time.time() - snapshot.get("started", time.time()), 2)
    send_json(h, 200, snapshot)


@get("/api/mcp/tools")
def mcp_tools(h):
    try:
        from roborun.ros_mcp import get_all_tools
        tools = get_all_tools()
        names = [t["name"] for t in tools]
        send_json(h, 200, {"ok": True, "count": len(names), "tools": names})
    except Exception as exc:
        send_json(h, 200, {"ok": False, "count": 0, "error": str(exc)})
