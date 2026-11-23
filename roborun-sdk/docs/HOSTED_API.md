# Appendix: the hosted API

An appendix to [BUILDING.md](BUILDING.md), the Roborun reference. `https://api.roborun.app` turns a camera view and your
words into motion. You normally never call it yourself: the robot
runtime that `roborun up` starts calls it for every task, verifies what comes back, and plays it on your arms with Stop and
safety on your computer. Build on the local API (`http://127.0.0.1:8120`, see [BUILDING.md](BUILDING.md#the-local-api));
that is the interface Roborun keeps stable.

This page is for the cases where you want to look behind it: checking your key and usage from a script, or verifying a
signed result yourself.

## Keys

Every request carries your key: `Authorization: Bearer rr_live_...` (or an `rr_test_...` key). `roborun login` creates one and
saves it in `~/.config/roborun/token` (`roborun up` also reads one from `ROBORUN_API_KEY`); keys are also made and revoked on your account at <https://app.roborun.app>. A robot
needs a robot key: a read-only key can read your account and jobs but not run a robot.

Send a `User-Agent` header that names your program (for example `my-app/1.0`). Requests with a library's default
User-Agent may be refused before they reach the API.

## Errors

Errors are JSON: `{"error": {"code": "...", "message": "..."}, "type": "error"}`, sometimes with more fields inside `error`.

| Status | Meaning | What to do |
| --- | --- | --- |
| `400` | The request is not valid (`invalid_request`, `bad_month`, ...) | Change the request |
| `401` | No key, or a key the API does not know or that was revoked | `roborun login`, or a new key |
| `403` | The key cannot do this (a read-only key running a robot, a test key on account routes) | Use a robot key |
| `409` | A job id reused with a different body, a result that is not ready, missing uploads | See the message |
| `429` | Too many requests for this key | Wait `Retry-After` seconds |
| `502`, `503`, `504` | The API is busy or restarting | Try again after `Retry-After` seconds |

## Routes you may call

| Call | Answers |
| --- | --- |
| `GET /v1/health` | Is the API up (needs your key, like every route; `401` without one) |
| `GET /v1/account` | Whether your key is accepted, its account and scope |
| `GET /v1/usage[?month=YYYY-MM]` | This month's (or that month's) jobs by kind, for your account |
| `GET /v1/keys` | `{"keys": [{kid, alg, pub, not_after, use}]}`: the public keys that sign results |
| `GET /v1/motion/jobs?limit=N` | Your recent jobs, newest first |
| `GET /v1/motion/jobs/{id}` | One job: `status` (`queued`, then its working stages, then `done`, `failed` or `cancelled`), `stage`, `error` |
| `GET /v1/motion/jobs/{id}/result` | The signed result of a finished job (`409` until it is done) |
| `POST /v1/motion/jobs/{id}/cancel` | Cancel a job; cancelling a finished or cancelled job is fine |
| `GET /v1/robots/{robot}/episodes` | The robot's uploaded dataset (LeRobot v2.1): `dataset` (version, robot type, fps, totals) and its `episodes` (index, prompt, length, outcome, when) |
| `GET /v1/robots/{robot}/dataset.tar` | The whole dataset as one tar: unpack it and load the `dataset/` folder with LeRobot. Faces are blurred |
| `POST /v1/robots/{robot}/data/delete` | Delete everything kept for this robot (dataset, recordings, task records). Send `{"confirm": "<robot>"}`; anything else is refused. It cannot be undone from your side. Read-only keys cannot delete |

Jobs are submitted by the robot runtime (`POST /v1/motion/jobs`). The request format is the runtime's and changes with it;
a job id is chosen by the client, and sending the same id and body again returns the same job rather than a new one.
A result holds positions (or, for a humanoid, a locomotion timeline) and nothing else: how the service works them out
stays on the service.

## Verifying a result

Every result is signed with Ed25519 by one of the keys at `GET /v1/keys`, and names the request it answers
(`request_digest`). The runtime checks both before anything moves. To check a result yourself with the SDK's verifier:

```python
import json, os, urllib.request
from roborun_sdk.rt import signing

def get(path, key):
    request = urllib.request.Request('https://api.roborun.app' + path,
                                     headers={'Authorization': f'Bearer {key}', 'User-Agent': 'my-app/1.0'})
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)

key = open(os.path.expanduser('~/.config/roborun/token')).read().strip()
keys = signing.keys_from_trusted(get('/v1/keys', key)['keys'])      # {kid: public key}, the ones in date
result = get(f'/v1/motion/jobs/{job_id}/result', key)
ok, reason = signing.verify(result, result['type'], keys)           # (True, 'ok'), or (False, 'bad_signature' ...)
```

Fetch `/v1/keys` again when a result names a `kid` you do not have: keys are replaced over time, and each one says until
when it is valid (`not_after`).
