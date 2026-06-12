---
name: roborun
description: Control and query the RoboRun robot server — start/stop patrols and other behaviors, check what the robot sees, read its event timeline. Use when the user mentions the robot, patrols, or asks what the robot is doing.
---

# RoboRun robot control

The robot runs a RoboRun server on the local network. Talk to it over HTTP.
Start every session by setting the base URL (honors an operator override):

```bash
ROBORUN_URL="${ROBORUN_URL:-http://127.0.0.1:8765}"
```

The robot also pushes notifications *to* you via the gateway's webhook
(`/hooks/agent`) — messages prefixed "RoboRun notification" arrived that way.
When relaying them, keep it to one short message.

## Is it up?

```bash
curl -s $ROBORUN_URL/api/health
```

## Behaviors (patrol, sentry, follow_person, ...)

List, with running state:

```bash
curl -s $ROBORUN_URL/api/behaviors
```

Start or stop one ("start the patrol" → enable `sentry` if present, else `patrol`):

```bash
curl -s -X POST $ROBORUN_URL/api/behaviors/enable  -H 'Content-Type: application/json' -d '{"name":"sentry"}'
curl -s -X POST $ROBORUN_URL/api/behaviors/disable -H 'Content-Type: application/json' -d '{"name":"sentry"}'
```

## What does the robot see / what happened?

Live detections and pose:

```bash
curl -s $ROBORUN_URL/api/state
```

Recent timeline events (detections, behavior logs, notifications):

```bash
curl -s "$ROBORUN_URL/api/run/events"
```

Camera snapshot metadata: `curl -s $ROBORUN_URL/api/camera`

## Rules

- Confirm a behavior actually flipped by reading the response `ok` field;
  report failures honestly ("no behavior named X — available: ...").
- Never enable more than one movement behavior at a time; disable the
  current one first.
- If the server is unreachable, say so — don't guess at robot state.
