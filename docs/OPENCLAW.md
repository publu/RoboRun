# Make your robot OpenClaw-ready

"OpenClaw-ready" means your robot is a contact in your chat app: it messages
you when something needs a human, and you command it by replying. With
RoboRun that is one env var and one skill install — no SDK to embed, no
blockchain, no platform account. Concretely, RoboRun has two AI-facing
channels that complement each other:

```
            commands (pull)                      commands (chat)
 Claude ───────────────────► MCP /mcp   WhatsApp ──► OpenClaw ──► HTTP /api
 (Claude Code, Desktop, …)              ("start patrol")  skill

                         notifications (push)
 behavior code ──robot.notify()──► event bus ──bridge──► OpenClaw /hooks/agent
                                                          └──► your phone
```

- **MCP** (`http://localhost:8765/mcp`) is the command channel for AI coding
  clients. Pull-shaped: great for driving the robot, useless for reaching you
  when no session is open.
- **The OpenClaw bridge** (`roborun/openclaw.py`) is the push channel: the
  robot patrols a building at 2am, sees someone, and your phone buzzes —
  no laptop, no open session.
- **The OpenClaw skill** (`integrations/openclaw/SKILL.md`) closes the loop:
  you reply "go check again" in the same chat, and the assistant drives the
  robot over its HTTP API.

## 1. Gateway: enable hooks

In your OpenClaw gateway config:

```json5
{
  hooks: {
    enabled: true,
    token: "generate-a-dedicated-secret",   // not your gateway token
    path: "/hooks",
  },
}
```

Keep the endpoint on loopback or a tailnet; query-string tokens are rejected
by OpenClaw, and the bridge always sends `Authorization: Bearer`.

## 2. Robot: point RoboRun at the gateway

```bash
export OPENCLAW_HOOKS_URL=http://127.0.0.1:18789/hooks
export OPENCLAW_TOKEN=generate-a-dedicated-secret
export OPENCLAW_CHANNEL=whatsapp     # optional; gateway default otherwise
export OPENCLAW_TO=+15555550123      # optional delivery address
roborun
# ...
#   OpenClaw bridge:  notify() → http://127.0.0.1:18789/hooks
```

## 3. Behaviors: call `robot.notify()`

```python
@behavior(hz=5, autostart=False)
def sentry(robot):
    if robot.see("person"):
        robot.notify("person spotted near waypoint 4")
```

`notify()` is `log()` plus reach: it always lands in the (hash-chained)
timeline, and with the bridge configured it becomes a `POST /hooks/agent`
with `deliver` set — OpenClaw runs an isolated agent turn and relays the
message to your chat. `behaviors/sentry.py` is a complete example. MCP
clients get the same power via the `notify` tool.

## 4. Assistant: install the skill

Copy `integrations/openclaw/` into the gateway's skills directory (e.g.
`~/.openclaw/skills/roborun/`). If the gateway runs on a different machine
than the robot, set `ROBORUN_URL` in the gateway environment.

Now the full loop works from your phone:

> **robot** sentry: 1 person(s) in view
> **you** show me what's running
> **assistant** sentry is enabled at 5 Hz, 12,403 ticks, no errors. Last
> event: "lap 3 complete, all quiet."
> **you** ok, stop the patrol
> **assistant** Done — sentry disabled.

## Design notes

- The bridge subscribes to the same event bus as the dashboard SSE feed and
  the journal, so every notification is also part of the sealed, tamper-
  evident run — "the robot texted me" is a verifiable claim.
- Only `notify` events cross, and they are never throttled: the judgment
  "is this worth a human's attention?" belongs in the behavior, where the
  author has state and context (see sentry's 60s cooldown), not in a chat
  session sifting raw events. Raw event forwarding was deliberately left
  out for that reason.
- Delivery failures are logged edge-triggered to the timeline as
  `system/openclaw` events and never crash a behavior.
