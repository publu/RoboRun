# RoboRun Studio — sequential audit (User · Investor · Competitor)

Living document. Each pass walks every Studio page through three lenses in order.
Screenshots: `/tmp/studio-shots/pass1/*.png`. Server: `localhost:8765`, SPA at `/studio`.
Pages in nav order: **Live · Sims · Runs · Search** (primary) → **Analytics · Scenarios · Swarm Lab** (More).

Severity key: **P0** blocks/looks broken · **P1** real friction · **P2** polish · **P3** nice-to-have.

---

## PASS 1 — USER LENS

> Persona: a robotics dev trying RoboRun for the first time, no hardware, wants to see it work
> and understand how they'd build with it. Also a returning dev reviewing recorded runs.

### Shell / navigation (all pages)
- ✅ Consistent left rail (priority-flat: Live/Sims/Runs/Search, then More), header with project scope +
  Record, instrument-panel aesthetic, IBM Plex type. Reads as one product.
- **P1 — Record gives no feedback about *what* it's capturing.** Clicking ● Record turns it red but the
  user can't tell which source/robot is being recorded or that anything is happening. No elapsed timer,
  no "recording arena" label. Easy to think it did nothing.
- **P2 — Project scope is invisible value.** The chip says "scratch" but a newcomer has no idea what a
  project/environment *is* or why they'd switch. No first-run hint.
- **P2 — No global "what is this / help".** No onboarding, tour, or docs link anywhere in the shell.
- **P3 — Brand "RoboRun Studio" wordmark** isn't a home affordance beyond /live; fine.

### Live
- ✅ Camera / Events / Detections / Telemetry quadrant; events stream in real time; LIVE pill pulses;
  pause now freezes follow (fixed).
- ✅ Empty camera is now actionable: "○ no camera — Start a sim in Sims or connect a robot."
- **P1 — On a cold start the page is 3/4 empty** (no camera, "nothing detected", "Time: --" plot). The one
  actionable hint is inside the camera tile; Detections/Telemetry just look dead. Newcomer's first
  impression is "nothing works." Needs a single clear primary CTA ("▶ Start a sim") at the top of Live.
- **P1 — Telemetry plot shows "Time: --" with no series** when nothing publishes `velocity`. Reads as
  broken rather than "no data." Needs an empty state like the camera has.
- **P2 — Camera polls `/api/camera/frame` at 10Hz forever even while 503ing.** Wasteful; should back off
  when there's no source. (Confirmed in server log: continuous 503s.)
- **P2 — No way to tell *which* source Live is showing** (webcam? sim? which robot?). No source label.

### Sims — Arena
- ✅ Slim target switcher (Arena/Fleet/Data/Real), one "how to use" line, single native picker, code on
  demand. Iframes keep-mounted so switching no longer flickers (fixed).
- **P1 — Two "how do I drive it?" stories collide.** The strip says "drive with WASD," but the arena's own
  "PICK A ROBOT & TASK" modal is the first thing you see; WASD only works after you dismiss it and click
  into the canvas. The hand-off isn't explained (does focus need to be in the iframe?).
- **P2 — First load of the arena is still heavy** (three.js + Rapier from CDN); a few seconds of dark
  canvas before the modal. No loading indicator in the Studio frame.
- **P2 — "how to code it" shows `follow_person.py` but there's no path from here to actually edit it.**
  The code is read-only; the user can't open `behaviors/` or an editor. The pitch ("change a number, save")
  has no in-app action.

### Sims — Fleet
- ✅ Renders the Rapier warehouse in-shell; sliders + play; auto-rotate now stops once you grab it (fixed).
- **P1 — "Play" then what?** Robots wander; the FOUND/COVERAGE stats tick, but the user isn't told the goal
  or how their behavior/strategy affects it. The "Swarm Lab →" link is the missing bridge but it's buried
  in the descriptive paragraph.
- **P2 — Controls density.** Robots/Floors/Speed + 4 stat tiles + a legend + per-floor counts + the 3D view
  is a lot at once for a first look.

### Sims — Data Sim
- ✅ Now loads without the state error; model dropdown shows real names; Start/Reset.
- **P1 — "frames stream into Live" but there's no link to go see them.** User presses Start here, nothing
  visibly happens on *this* page (the render is server-side); they must know to navigate to Live. Dead-end.
- **P2 — No feedback while a model boots** (MuJoCo load can take a moment).

### Sims — Real robot
- **P1 — Copy/ీreality mismatch.** The "how to use" promises "enter your rosbridge IP and Connect," but the
  panel is the full 4-step setup wizard starting at "Project." The promised IP field is several steps in.
- **P2 — For a no-hardware user this is a dead end** with no "I don't have a robot, show me a sim instead."

### Runs + Replay
- ✅ Run list → click flips to replay; scrubber switches to scrub mode; events fill by playhead; replay
  speed now realtime (fixed).
- **P1 — Run list is opaque.** Rows are `run_20260619_003800  arena  ·  277B` — timestamps + bytes. No
  human label, no thumbnail, no duration, no "what happened" summary. Hard to pick the run you want.
- **P1 — Replaying an event-only run shows "no camera frame" + "nothing detected" + empty plot** — looks
  broken even though it's working (those runs have no MCAP camera). No indication the run simply lacks video.
- **P2 — No seal/verify affordance in the replay UI.** The black-box/tamper-evidence story (a headline
  feature) is invisible here — no "VERIFIED ✓" badge, no verify button.
- **P2 — Can't delete/flag/export a run from the list.**

### Search
- ✅ Explains scope ("every recorded observation in scratch · across all runs"), defaults to recent
  results, by-meaning/by-object/recent chips, clicking a hit jumps into replay.
- **P1 — Result cards are text-only** ("yellow crate · dog-sim · timestamp"). For a *visual* search over
  what robots saw, the lack of thumbnails is a big miss — you can't recognize a result at a glance.
- **P2 — No result count / no "showing N of M".** Grid just fills; unclear if it's everything.
- **P2 — "by object" with a free-text query** — does it substring match labels? Behavior of each chip vs the
  text box isn't explained.

### Analytics
- ✅ Native panels, stat tiles, label bars, 24h chart, fleet activity. Consistent chrome.
- **P2 — "0 AI-searchable" and "0 robots online" read as failures**, not "nothing embedded yet / no live
  robots." Zero-states need framing.
- **P2 — Static snapshot** (no refresh / time-range control). "Last 24h" is fixed.
- **P3 — No drill-down** — clicking a label/robot doesn't filter Search/Runs.

### Scenarios
- ✅ Native suites + scenario cards + run buttons + results table; no more dead-whitespace void.
- **P1 — "run" gives little feedback.** Clicking run sets "running…" then a row appears in the table; no
  live progress, no link to the recorded run it produced, no pass/fail color until done.
- **P2 — "Runs · 0" + empty table** dominate the lower half before you've run anything.

### Swarm Lab (hosted)
- ✅ Dense control panel + 2D coordination map + live strategy code; badge suppressed in frame.
- **P1 — Heaviest cognitive load of any page** (sliders for radio range / link reliability / airtime /
  onboard memory / inbox depth…), unexplained jargon, and it's a hosted page so it visually diverges
  slightly (denser, flatter) from the native Studio pages.
- **P2 — Relationship to Fleet sim is unclear** — both are "fleet," two different pages.

### Cross-cutting (user)
- **P1 — No global loading states.** Iframes and fetches pop in; the app never says "loading."
- **P2 — Keyboard/focus:** scope menu has no Esc; sim target tabs have no arrow-key roving focus.
- **P2 — Not responsive** under ~900px (fixed 220px rail + grid). Fine for desktop control station, breaks
  on a laptop in a split window.
- **P2 — No dark/light or density options;** fine for the aesthetic, noting it.

---

## PASS 1 — INVESTOR LENS

> Persona: a seed/Series-A investor who knows robotics + dev-tools. Question on every page: *does this
> show a wedge, a moat, and a "holy shit" demo moment — or is it a nicer Foxglove?* RoboRun's thesis:
> one behavior file runs sim→real on any ROS robot; every run is sealed (tamper-evident) and searchable
> across a fleet; MCP-native so agents can drive robots.

### The thesis, surfaced (cross-page)
- **Strong wedge, weakly surfaced.** The three genuinely-differentiated assets — (1) the same `see/move/ask`
  file from sim→real, (2) sealed/anchored black-box runs, (3) cross-fleet semantic search — are all *present*
  but none is the hero of any page. A visitor could leave thinking "browser robot sim + a dashboard."
- **No "holy shit" moment staged.** The killer demo is: run a behavior in sim → record → it's sealed +
  searchable → replay the exact frame → prove it wasn't tampered. Studio has all the parts but no guided
  path that detonates that in 60 seconds. **This is the single highest-leverage investor fix.**

### Live — investor view
- ✅ "Robot doing something in my browser, no install" is a credible cold-open.
- ❌ On load it's mostly empty (no camera/detections) → reads as "pre-product." An investor's first 5
  seconds are wasted. **Demo-readiness P0:** Live must look alive on first paint (seed with the running
  arena or a demo run).
- Moat signal here: low. It looks like telemetry. The provenance/fleet story isn't visible.

### Sims — investor view
- ✅ **The strongest page for the thesis.** Sim→real switcher + "same code runs everywhere" + the
  `follow_person.py` snippet is *exactly* the pitch. A dev-investor gets it.
- ❌ But "Real robot" dead-ends in a setup wizard, and the code is read-only — so the "write once, run
  anywhere, edit live" claim isn't *demonstrated*, only stated. Investors discount stated-not-shown.
- Defensibility: the unified sim/real abstraction is real IP; make it the headline.

### Runs + Replay — investor view
- ✅ This is where the **moat** lives (sealed MCAP, Merkle root, RFC-3161 anchor, Ed25519). Foxglove can't
  prove a recording wasn't edited; RoboRun can.
- ❌ **The moat is completely invisible in the UI.** No VERIFIED badge, no merkle root, no "tamper → detect"
  demo. An investor reviewing Runs sees a worse Foxglove, not a defensible provenance product. **P0 for
  fundraising:** surface verify/seal state prominently and offer a one-click tamper-demo.

### Search — investor view
- ✅ "Search everything every robot ever saw, across the fleet, offline" is a fundable line — it's the
  data-network-effect story (more robots → more searchable history → more value).
- ❌ Text-only results undersell it; a wall of thumbnails of "what robots saw" is the screenshot that goes
  in the deck. Also nothing communicates *fleet-wide / cross-robot* — looks single-machine.

### Analytics — investor view
- ✅ Good "traction surrogate" for a demo (things seen, runs, storage). Cheap credibility.
- ❌ All zeros on AI-searchable / robots-online read as "no traction." For a demo, seed it.
- Not a moat; it's table stakes. Fine as supporting.

### Scenarios — investor view
- ✅ **Underrated wedge:** "scored, tamper-proof behavior tests" = CI for robots. That's a real category
  (robot eval/regression) investors like, and it compounds with the sealed-runs moat.
- ❌ Buried under "More", empty by default, no pass-rate trend. The story (regression-test your robot,
  provably) isn't told.

### Swarm Lab — investor view
- ✅ Multi-robot coordination is a sexy demo (swarms photograph well).
- ❌ Too complex/jargon-heavy to land in a pitch; reads as a research toy, not a product. Either simplify to
  a "watch the swarm cover the map" hero or de-emphasize for fundraising.

### Investor verdict (Pass 1)
- **Funded thesis is real; the product hides it.** Priorities to be fundable: (1) a guided 60-sec
  sim→record→seal→search→verify demo path; (2) surface provenance (VERIFIED badge + tamper demo) in Runs;
  (3) make Live/Analytics look alive (seed demo data — `roborun demo`); (4) thumbnails in Search; (5) lead
  Sims with "same file, sim→real" and make the code editable/runnable in-app.
- **Risk flags:** lots of breadth (7 pages) but each shallow; the differentiators are stated not shown;
  empty zero-states everywhere make it look pre-traction.

---

## PASS 1 — COMPETITOR LENS

> Benchmarks: **Foxglove Studio** (the incumbent robotics viz/replay), **BAGEL** (browser-only bag viewer,
> zero-install, 344 tests, polished panels), **Antioch** (north-star 5-stage agent loop). Question per page:
> *would a user pick this over the incumbent, and why?*

### Where RoboRun structurally wins (true on every page)
- **It runs robots** (sim + real + behaviors + MCP). Foxglove/BAGEL only *view* data. Different category.
- **Provenance**: sealed, anchored, verifiable runs. Neither Foxglove nor BAGEL can prove a recording is
  untampered. Unique.
- **Fleet-wide semantic search** over everything seen. Foxglove has no cross-run CLIP search.
- **Zero-install, offline, local-first** like BAGEL — but with the runtime attached.

### Live vs competitors
- Foxglove's live (rosbridge/ws) panels are far richer (plotting, image annotations, 3D, raw topic
  inspection). RoboRun Live is a fixed quadrant. **Behind on viewer depth.**
- But Foxglove can't *start* a robot/sim from the UI; RoboRun can (Sims). Net: different value.

### Sims vs competitors
- **No competitor has this.** Foxglove/BAGEL have no sim launcher; this is pure RoboRun. Strongest
  differentiator on any page. The threat is upstream (Gazebo/Isaac/MuJoCo Studio) not Foxglove.

### Runs + Replay vs competitors
- **This is the contested ground and RoboRun currently loses on polish.** BAGEL/Foxglove replay is mature:
  scrub, multi-panel sync, image distortion, TF tree, timeline bookmarks, frame-export, multi-bag overlay.
  RoboRun replay is a basic quadrant with text events + (often) no video.
- **RoboRun's only winning move here is provenance** (verify/seal) + "replay any sealed run from the fleet."
  That's not surfaced (see investor P0). Without it, this page reads as "worse Foxglove."
- BAGEL opens `.mcap`/`.db3`/`.bag`; RoboRun replay only opens its own runs. Foxglove opens RoboRun's MCAP
  natively — so today a serious user would record in RoboRun and *replay in Foxglove*. That's the gap to close.

### Search vs competitors
- **Category-defining, no direct competitor.** Foxglove/BAGEL have no semantic search across runs/fleet.
  This + provenance is the defensible combo. Just undersold (text-only, see user P1).

### Analytics vs competitors
- Foxglove has dashboards/Foxglove Studio layouts; this is lighter. Table stakes, not a battleground.

### Scenarios vs competitors
- Closest analog is robotics CI / eval harnesses (not Foxglove). "Scored + sealed behavior tests" is a real
  differentiator vs ad-hoc rosbag regression. Underexposed.

### Swarm Lab vs competitors
- Niche; competes with research/academic swarm sims, not the viewer incumbents. Not a commercial battleground.

### Antioch parity (north-star)
- Antioch's value is the tight 5-stage agent loop (perceive→decide→act→record→learn). RoboRun has the
  pieces (see/move/ask + MCP + record + search) but Studio doesn't *show the loop closing*. The agent story
  (MCP-native, `robot.delegate`, hot-reload) is invisible in Studio — there's no "agent is driving" view.
  **Gap: an agent/MCP activity surface** would directly target Antioch parity and is absent.

### Competitor verdict (Pass 1)
- **Win by category, not by polish.** Don't try to out-Foxglove Foxglove on replay panels. Double down on
  the three things no competitor has — sim→real runtime, provenance, fleet search — and make them the
  spine of Studio. Biggest competitive risk: Runs/Replay invites a direct Foxglove comparison RoboRun loses,
  while the winning features sit unsurfaced. Add an **agent/MCP view** to chase Antioch parity.

---

## CONSOLIDATED BACKLOG (Pass 1 synthesis)

Ranked by leverage across all three lenses. Already-fixed items (pause, sim/state, flicker, scope, dead CSS,
realtime replay) are excluded.

### P0 — fundraising / first-impression blockers
1. **Stage a guided "golden path" demo** (sim → record → seal → search → verify) so the differentiators
   detonate in ~60s. Touches Live/Sims/Runs/Search. *(investor P0, competitor verdict)*
2. **Surface provenance in Runs/Replay** — VERIFIED/ANCHORED badge, merkle root, one-click tamper→detect
   demo. Turns "worse Foxglove" into the moat. *(investor P0, user P2)*
3. **Make Live + Analytics look alive on first paint** — seed `roborun demo` data / auto-start a demo run so
   the landing isn't 3/4 empty zero-states. *(user P1, investor demo-readiness)*

### P1 — real friction / undersold value
4. **Thumbnails in Search results** (visual recall) — and signal it's fleet-wide. *(user+investor+competitor)*
5. **Human-readable Run list** — label, duration, thumbnail, sealed badge, "what happened" — not `ts · bytes`.
6. **Record feedback** — show what's being recorded + elapsed timer + where it lands.
7. **Live primary CTA + Telemetry empty-state** — "▶ Start a sim" at top; frame "no data" like the camera.
8. **Agent/MCP activity view** — show the agent driving (Antioch parity); currently invisible in Studio.
9. **Close the Sims loops** — Data Sim → "view in Live"; Real-robot copy matches the actual flow; an in-app
   path to edit/run a behavior (the "change a number, save" pitch is stated, not actionable).

### P2 — polish
10. Camera 503 back-off (stop 10Hz polling with no source). 11. Global loading states (iframes/fetches).
12. Zero-state framing on Analytics ("0" ≠ failure). 13. Search result count / "showing N of M".
14. Keyboard: Esc-close scope menu, arrow-key roving on tabs. 15. Fleet sim "Play → goal" explainer +
    clarify Fleet-vs-Swarm-Lab relationship. 16. Scenarios run progress + link to produced run.

### P3
17. Responsive layout < 900px. 18. Analytics refresh / time-range. 19. Analytics drill-down into Search/Runs.

**Top issues filed as tracked tasks** (TaskCreate) — items 1–9 above. The rest live in this doc as the
standing backlog.

---

## PASS 2 — DEEPER RE-WALK (interaction-level delta)

No backlog items were implemented between Pass 1 and Pass 2, so there is no "fixed" delta; Pass 2's value is
exercising real click-flows (Playwright) to surface what a static look missed.

### NEW — P1: Search → replay is frequently a *dead click*
- Probed live: of the 48 default ("recent") Search results, **zero contained "↳ open in replay"** — i.e.
  none were run-backed. Most indexed observations have `source = sim/stream` and **no `run_id`**, so their
  cards aren't openable. Clicking them calls `jump()` which returns silently (no `run_id`).
- Impact: the headline unification ("search anything → jump into the exact replay frame") is, with the data
  that's actually in the index, *usually unavailable*. It only works for observations extracted from a
  sealed MCAP (`source=mcap`). This was assumed-working in earlier verification but is data-dependent.
- Fix options: (a) link stream/sim observations back to their run at index time, or (b) clearly mark
  non-replayable results and disable/relabel their click.

### NEW — P2: silent dead clicks have no feedback
- Result cards look identical whether replayable or not; only a subtle accent line distinguishes them, and a
  non-replayable click does nothing with no toast/cursor/disabled state.

### CONFIRMED via interaction (promoted from "suspected")
- **Scope menu ignores Esc** — opened menu stayed open after Escape. Keyboard-trap-ish. (was user P2)
- **Scenarios "run" works** — clicking run populated result rows (2 rows appeared), so there *is* terminal
  feedback; but still no live progress and no link to the run it produced. (refines Pass-1 user P1)
- **Replay robustness is good** — opening an old/event-only run produced **0 page errors**; graceful empty
  states. (positive confirmation)

### Pass-2 verdict
The most important new finding is that **search→replay is mostly non-functional against real indexed data**
(missing `run_id` linkage), which undercuts the single most differentiated user flow. Promote to the P1 band
of the backlog and verify the indexing path (`StreamingExtractor` / `export`/`extract_run`) sets `run_id` for
live/sim observations, not just MCAP extraction.

### Pass 2 — per-page re-walk (completing the pages skipped above)
Interaction-probed the remaining pages so Pass 2 genuinely covers *every* page. **0 page errors across all.**

- **Live** — pause/resume verified at the DOM level (`LIVE → pause → PAUSED → resume`); the throttle didn't
  break following. Camera still polls 10Hz while 503ing (P1 stands). Telemetry plot did *not* show "Time: --"
  in this probe (data-dependent — shows the empty legend only when no `velocity` series), confirming the
  "looks broken when empty" risk is intermittent, not constant.
- **Sims · Arena** — selecting Arena shows only the `/sim` iframe; it stays mounted when you switch away and
  back (no reload). Keep-mounted fix confirmed live.
- **Sims · Fleet** — selecting Fleet shows `/fleet-sim`; mounts alongside Arena (both retained), instant
  switch, no errors. Confirmed.
- **Sims · Data Sim** — renders the native `DataSim` component (no iframe) and loads without the old
  `_drone_ctrl` crash (fixed). Start/Reset present. Dead-end-to-Live (P1) still stands.
- **Sims · Real robot** — shows the `/setup` iframe = the full 4-step wizard; confirms the copy/flow
  mismatch (P1) — the promised "IP + Connect" is several steps in.
- **Analytics** — 10 panels render; **no refresh / time-range control** (static snapshot confirmed). Zero
  framing on "0 AI-searchable / 0 robots online" still reads as failure (P2 stands).
- **Swarm Lab** — single hosted iframe loads cleanly with the floating badge suppressed; density/jargon
  concern (P1) stands; still visually flatter than native pages.

**Robustness positive:** the full deeper re-walk (Live + 4 Sims targets + Runs + Search + Scenarios +
Analytics + Swarm + scope menu) produced **zero page/console errors** — the app is stable; the gaps are
product/UX depth, not crashes.

---

## PASS 3 — IMPLEMENTED (first-run + ease-of-use overhaul)

Driven by the "it makes no sense when I open it" feedback. Root causes found and fixed:

- **"Why is it recording?"** — the run-list `recording` flag tracks the *always-open event journal*, not a
  real MCAP recorder, so the RecordButton showed red on every fresh load. Fixed: RecordButton now reads the
  true state from `/api/run/mcap.recording`; relabeled "● Record run" with a tooltip + live timer; run-list
  REC badge also uses the true active-recorder run (verified: 0 false REC badges, sealed run shows ANCHORED).
  Also stopped a stale recording left from testing.
- **Empty/jargon Live on open** — added a first-run **welcome hero** (value prop + "▶ Start a sim — no
  install" + 3-step guide) shown whenever nothing is live; swaps to panels once a webcam/sim/robot **or the
  browser arena** is producing data (active-detection includes fresh events, not just `/api/dashboard`).
- **"scratch" noise** — the project scope chip is now **hidden** until a project exists (newcomers see
  nothing); it appears once you create one.
- **Event-log noise** — filtered the "no actuator" nag, behavior-autoload lines, and per-frame camera
  hashes; collapse consecutive duplicates; source-aware empty states (live vs replay).
- **Runs legibility** — rows now show date · robot · "683 poses · 44 detections" · size, with SEALED/
  ANCHORED/REC badges instead of `ts · bytes`.
- **Provenance surfaced** — a replay shows a **✓ VERIFIED · ANCHORED + merkle-root** bar (the moat, finally
  visible). (Tamper→detect demo still pending.)
- **Search dead-clicks** — result count + "N replayable"; non-replayable hits dimmed + labeled "not in a
  recorded run" with a tooltip, instead of silent no-ops.
- **Sims loops** — Data Sim shows "→ watch it in Live" once running; Real-robot copy now matches the setup
  wizard. (In-app behavior editing still pending.)
- **Analytics zero-states** — "0 AI-searchable / 0 robots online" reframed ("install vision to embed" /
  "N seen, none live").

Still pending from the backlog (tracked tasks): guided golden-path demo (#24), tamper→detect demo (#25),
auto-seed demo data (#26), Search thumbnails (#27), in-app behavior editor (#32), backend run_id linkage so
*all* observations are replayable (#33), agent/MCP view (#31).

---

## PASS 4 — full re-walk (current state), found + fixed

Screenshotted every page again. Found two real bugs and two data findings:

- **FIXED — replay 500 on a corrupt run.** `/api/run/series` and `/api/run/frame` raised
  `RecordLengthLimitExceeded` (uncaught) on a malformed MCAP → HTTP 500, breaking the replay view. Added a
  `_safe_messages()` iterator in `run_series.py` that stops cleanly on a bad/partial file; series now
  returns graceful empty (200) and frame returns 404, both handled by the UI. Good runs unaffected.
- **FIXED — Scenarios results all showed "—".** The table mapped `scenario/result/scores`; the API returns
  `name/outcome/metrics`. Re-mapped → results now show "● passed · threshold_gain · demo".
- **FINDING — some demo runs have unreadable MCAPs.** `RunRecorder`/`_ChainedStream` occasionally writes an
  MCAP the stock reader can't summarize (corrupt record length), so those runs have no thumbnail/replay.
  Defensive handling now prevents breakage; the root recorder corruption is a backend bug (tracked).
- **CLARIFIED — "why is it recording" during a sim.** The browser arena auto-records its session via the
  pyodide shim (`web/py/shim.py`), so REC turns on when you run the arena. This is now *truthful* and
  labeled (real recorder state + timer + tooltip), unlike the original always-on false flag — acceptable
  under the "seal every run" design, though lingering recordings could auto-stop on leaving the sim.

### g14 resolution — the "corrupt MCAP" was unsealed partials, not a recorder bug
- A fresh `record/start → (sim data) → record/stop` run replays perfectly; **RunRecorder is fine for
  sealed runs.** The unreadable files (7 of 16) were all **unsealed partials** — arena auto-records that
  never closed cleanly (page closed / server restart mid-record). An MCAP is only valid once finalized.
- Done: (1) defensive `_safe_messages` contains any partial (no 500s); (2) cleaned up the 7 abandoned
  unsealed partials so they don't litter Runs/Search. Demo runs verified readable (27/27/21 msgs).
- Deeper improvement (tracked #34): the arena auto-record should seal on stop/unload, or the recorder
  should write a recoverable summary incrementally, so an interrupted run is still readable.

---

## PASS 5 — sign-off (full re-walk after all changes)

Screenshotted every page after the overhaul. **All pages clean of console/page errors** except one graceful
404 on Search (a thumbnail for a frameless run → onError hides it). Net state:

- **First-run**: welcome hero ("Run a robot. Watch it. Search everything it saw.") with Start-a-sim, the
  60-sec guided tour, and load-sample-data; no false REC; no "scratch" noise.
- **Live**: legible (filtered events, framed empty states, real camera when a sim runs).
- **Sims**: thin launcher; Real robot = native connect panel (no wizard); editable behaviors that hot-reload.
- **Runs**: human-readable list + provenance bar with live **verify** and a reversible **tamper demo**.
- **Search**: thumbnails + replayable affordance + count; hits jump into replay.
- **Analytics / Scenarios**: native, framed zero-states, real results.
- **Agent**: MCP connect commands + live agent activity (Antioch parity).
- **Guided tour**: run → sealed/tamper → search → agent, in 4 steps.
- **Robustness**: corrupt/partial runs contained (no 500s); abandoned partials cleaned; lazy LiveSource so
  non-live routes open no sockets.

Remaining tracked backend item (non-blocking): #34 — arena auto-record should seal on stop/unload so an
interrupted run stays readable (today contained gracefully).

---

## PASS 6 — embedding sweep (stale standalone chrome on hosted pages)

Hosted pages (arena `/sim`, fleet `/fleet-sim`, swarm `/fleet`) carried their own standalone navigation that
duplicated/contradicted Studio's. Swept systematically:

- **Arena (`arena.js` framed guard):** when embedded, removes `#ck-home`, `#ck-views` + `#tb-views` (the two
  ▤ VIEWS menus), `#projChip`, all `a[href="/setup"]`, `.server-only` deck links, and the ROS + Fleet picker
  cards; rewrites the picker subtitle. Keeps the real sim controls. Verified: 0 stale links, all nav chrome
  gone.
- **shell.js pages (fleet, fleet-sim) — generic fix:** when framed, shell.js now rewrites every
  `a[href^="/"]` to the matching **Studio** route and sets `target="_top"`, so cross-links (e.g. Fleet's
  "Fleet Sim →", Fleet-Sim's "Swarm Lab") land on a Studio tab instead of loading a standalone page in the
  frame. A MutationObserver re-applies it to late renders. Verified: Swarm's cross-link → `/studio/sims`.
- **Native sweep:** `ScopeSwitcher` "+ new sim / robot" repointed `/setup → /studio/sims` (the only
  app/src link that left Studio). All other internal links already target `/studio/*`.

**Rule (to prevent regression):** any page Studio embeds must, when framed (`window.self !== window.top`),
hide its own nav chrome and route internal links to the Studio tab. shell.js pages get this automatically;
non-shell pages (arena) self-guard at the top of their script.
