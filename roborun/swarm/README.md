# Fleet communication demos

One quadruped is the arena sandbox. A *fleet* of them is the hard part: how do
robots split a job and share what they find when the radio between them is
imperfect? This package is the runnable answer, and the exact twin of the live
sandbox at **`/fleet`** (`roborun/web/fleet.js`).

## The assumptions (the knobs)

Every robot is limited in the three ways a real one is:

| knob | what it means | sandbox label |
|---|---|---|
| **radio range** | two robots exchange a message only while within this many metres | *Radio range* |
| **airtime** | messages a robot may send per second — its slice of a shared band | *Airtime* |
| **onboard memory** | map cells it can remember (oldest forgotten first) | *Onboard memory* |
| **inbox depth** | unread messages it queues while finishing its current move | *Inbox depth* |
| **link reliability** | best-case delivery odds; messages drop toward the edge of range | *Link reliability* |

Plus the rule that ties them together: a robot does **one thing at a time** — it
commits to a goal cell and only re-tasks once it arrives.

### Data points and the base station

The swarm doesn't just map — it finds **data points** and relays them back to a
**base station** (the "main server" / sink). A discovered datum isn't done when
found, only when it reaches base. Robots route it there with **greedy geographic
routing**: hand it to an in-range neighbour closer to base, deliver straight to
base if it's in reach, or carry it like a **data mule** until a downhill peer
appears. Short range forces multi-hop relays — the swarm acting as one antenna.

```python
run_episode("gossip", range_m=11, targets=10)["data_home"]   # e.g. "10/10"
run_episode("auction", base=False)                           # no sink, find-only
```

### Delivered vs. dropped, and overlap

A message is **delivered** when it lands in a neighbour's inbox; it's **dropped**
when the radio link failed (more likely near the edge of range) *or* the
receiver's inbox was full. **Overlap** ("re-walks") counts cells a second robot
re-mapped — wasted work that better coordination avoids. The live sandbox at
`/fleet` shows all of this, plus a **hover-to-inspect** view of what any single
robot knows (sensed firsthand vs. only heard over the radio), and a CONCEPTS tab
that explains the libp2p/gossipsub and stigmergy ideas behind it.

## The strategies (`strategies.py`)

| id | name | idea |
|---|---|---|
| `independent` | Lone wolves | no radio; map your own nearest-unknown cell |
| `gossip` | Gossip | broadcast what you just mapped; skip ground a neighbour has |
| `auction` | Claim & yield | announce a claim on your next cell, yield to a closer robot |
| `leader` | One commander | lowest-id robot in range hands out non-overlapping targets |

## Run it

```bash
python -m roborun.swarm            # compare all four, headless
python -m roborun.swarm auction    # one strategy
```

```python
from roborun.swarm import run_episode
run_episode("auction", count=8, range_m=10, memory=40)
```

## What you'll see

Claim-and-yield finishes the map first: claiming *intent* (not just sharing the
map) de-conflicts who explores where. Plain gossip can backfire — once robots
agree on the same map they pick the same nearest frontier and clump. A single
commander is tidy until a robot drifts out of its range and goes quiet. Shrink
the radio range or starve memory/airtime and every strategy degrades — which is
the whole point.

## On a real robot

These are framework-free on purpose so they run in CI, but the shape is the
on-robot one: a strategy is a per-tick policy over a `radio` (send/recv with
range + airtime caps) and a `goto`. Wire `comms.Fleet` to real
rosbridge/DDS peers and the same four functions drive hardware — the same
"the file is the product" contract the single-robot behaviours use.
