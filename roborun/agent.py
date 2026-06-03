"""Persistent Claude agent for robot control — stream-json subprocess protocol.

Adapted from the dimOS/RobotClaw agent. Uses the `claude` CLI with MCP servers
for both the RoboRun workbench and Daneel (dimOS robot control).
"""
from __future__ import annotations

import json
import subprocess
import threading
import time
from pathlib import Path
from typing import Iterator

ROOT = Path(__file__).resolve().parent.parent
SESSION_FILE = ROOT / ".roborun" / "agent_session.txt"

ROBOT_OPERATOR_CONTEXT = """You are the operator control agent for a robot running the dimOS stack.

## Architecture
- The robot runs **Daneel**, dimOS's built-in AI agent.
- You have MCP server connections to Daneel's live skill set (navigate, explore, speak, follow, patrol, etc.)
- These call the robot directly via dimOS's MCP server at port 9990.

## Key Skills
- `navigate_with_text` — go to a named place
- `begin_exploration` — autonomous frontier exploration
- `smart_follow_person` — follow people by description (YOLO+CLIP)
- `smart_follow_object` — follow objects by YOLO class
- `smart_find` — explore and search for something
- `query_scene` — what's visible right now
- `execute_sport_command` — RecoveryStand, FrontFlip, sit, etc.
- `speak` — robot speaks aloud
- `tag_location` — remember current location
- `where_am_i` — GPS and nearby landmarks

## Rules
1. Use skills for ALL physical actions — don't just describe, execute.
2. After commands, verify they worked.
3. Never claim the robot moved unless confirmed.
4. Be concise — this is a control panel.
"""


class RobotAgent:
    def __init__(self) -> None:
        self._proc: subprocess.Popen | None = None
        self._session_id: str | None = self._load_session()
        self._lock = threading.Lock()

    def _load_session(self) -> str | None:
        try:
            return SESSION_FILE.read_text().strip() or None
        except FileNotFoundError:
            return None

    def _save_session(self, sid: str) -> None:
        SESSION_FILE.parent.mkdir(parents=True, exist_ok=True)
        SESSION_FILE.write_text(sid)

    @property
    def is_alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _start(self) -> None:
        mcp_config = json.dumps({
            "mcpServers": {
                "daneel": {
                    "type": "http",
                    "url": "http://127.0.0.1:9990/mcp",
                },
            }
        })
        cmd = [
            "claude",
            "--input-format", "stream-json",
            "--output-format", "stream-json",
            "--verbose",
            "--model", "claude-sonnet-4-6",
            "--permission-mode", "bypassPermissions",
            "--mcp-config", mcp_config,
        ]
        if self._session_id:
            cmd += ["--resume", self._session_id]
        self._proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            cwd=str(ROOT), bufsize=0,
        )

    def _ensure_started(self) -> None:
        if not self.is_alive:
            self._start()

    def stop(self) -> None:
        if self._proc:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=3)
            except Exception:
                try: self._proc.kill()
                except Exception: pass
            self._proc = None

    def send(self, message: str) -> Iterator[dict]:
        with self._lock:
            try:
                self._ensure_started()
            except Exception as exc:
                yield {"type": "error", "error": f"Failed to start agent: {exc}"}
                return

            if self._session_id is None:
                instructed = f"{ROBOT_OPERATOR_CONTEXT}\n\nOperator request:\n{message}"
            else:
                instructed = message

            msg_json = json.dumps({
                "type": "user",
                "message": {"role": "user", "content": [{"type": "text", "text": instructed}]},
            }) + "\n"

            try:
                self._proc.stdin.write(msg_json.encode())
                self._proc.stdin.flush()
            except OSError as exc:
                self._proc = None
                yield {"type": "error", "error": f"Write failed: {exc}"}
                return

            accumulated = ""
            while True:
                try:
                    raw = self._proc.stdout.readline()
                except OSError:
                    break
                if not raw:
                    yield {"type": "error", "error": "Agent process ended"}
                    self._proc = None
                    return
                try:
                    event = json.loads(raw.decode())
                except json.JSONDecodeError:
                    continue

                etype = event.get("type")
                if etype == "system":
                    sid = event.get("session_id")
                    if sid:
                        self._session_id = sid
                        self._save_session(sid)
                elif etype == "assistant":
                    for block in event.get("message", {}).get("content", []):
                        btype = block.get("type")
                        if btype == "text":
                            chunk = block.get("text", "")
                            accumulated += chunk
                            yield {"type": "text", "text": chunk, "accumulated": accumulated}
                        elif btype == "tool_use":
                            yield {"type": "tool_use", "tool_id": block.get("id", ""),
                                   "tool_name": block.get("name", ""), "tool_input": block.get("input", {})}
                        elif btype == "thinking":
                            thinking = block.get("thinking", "")
                            if thinking:
                                yield {"type": "thinking", "thinking": thinking}
                elif etype == "user":
                    for block in event.get("message", {}).get("content", []):
                        if isinstance(block, dict) and block.get("type") == "tool_result":
                            content = block.get("content", [])
                            result_text = ""
                            if isinstance(content, list):
                                for c in content:
                                    if isinstance(c, dict) and c.get("type") == "text":
                                        result_text += c.get("text", "")
                            elif isinstance(content, str):
                                result_text = content
                            yield {"type": "tool_result", "tool_use_id": block.get("tool_use_id", ""),
                                   "result": result_text[:1200], "is_error": block.get("is_error", False)}
                elif etype == "result":
                    final = accumulated or event.get("result", "")
                    yield {"type": "done", "text": final, "cost": event.get("total_cost_usd", 0.0),
                           "error": event.get("result") if event.get("is_error") else None}
                    return

    def clear_session(self) -> None:
        self.stop()
        self._session_id = None
        try: SESSION_FILE.unlink()
        except FileNotFoundError: pass
