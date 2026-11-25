# Changelog: roborun-sdk

Every release of `roborun-sdk` (with the `roborun-station` of the same version), newest first. A version is
`0.<series>.<UTC build time>` and the build id names the same time ([Versions and upgrades](docs/BUILDING.md#versions-and-upgrades)).
`roborun docs changelog` prints this file.

Within a series, the stable column of [Versions and upgrades](docs/BUILDING.md#versions-and-upgrades) is only added to.
Anything in it that is removed or changed is listed here under **Removed** or **Changed**, named in backticks;
`python tools/stable_surface.py` (run by the test suite) fails when a stable name is removed or changed without an entry.

Releases before 0.1.20261121012822 were put together from the source history and the release builds. Remember that
`roborun upgrade` (or a release the hosted API no longer supports) changes the SDK your driver and your program run on.

## Unreleased

**Added**

- Camera cards (`robot.yaml` `card:` and `roborun_sdk.camera_card.outbound()`) now accept optional `description` (up to
  400 characters) and `fingers` (up to 200 characters) so custom cameras and wrist grippers describe themselves accurately
  to the motion service.
- `Robot.cameras()` (`GET /v1/cameras`) returns every camera card (delivered pixel size, mount, intrinsics, pose, depth
  and planning readiness). `examples/llm_tools.py` adds the `cameras` tool and exposes `camera` (`top`, `left`, `right`)
  on `run_task` so an LLM/VLM can plan from both overhead and wrist cameras with exact pixel dimensions for `marks`.
- Custom robot body profiles now accept `kind: quadruped`, `tracked` and `mobile` (in addition to `humanoid`, `wheeled`
  and `rover`), `base: tracked` (in addition to `legged` and `wheeled`), and up to 4 arms (`arms: 0..4`).
- `roborun up --server URL` now updates the saved motion service address in `robot.json` on an already configured robot
  instead of silently keeping the previous address.
- `Robot.clip()` is back as a stub so that a program written for 0.1.20261119203708 gets a clear error instead of
  `AttributeError`. It raises `RobotError` with status 410 and code `removed` and sends no request. The cameras' own
  recording of a task is `Robot.recording(task_id)`.
- This changelog. It ships in the wheel and `roborun docs changelog` prints it. Also new is the stable-surface check
  (`tools/stable_surface.py`, snapshot in `tools/stable_surface.json`).
- The wheels keep every docstring, so `help()` and editors show them. Before, only `client`, `errors` and `devkit` kept
  theirs. This includes the driver interfaces (`StreamedArm`, `ArmAdapter`), the camera source (`CustomSource`) and the
  body adapters (`BaseAdapter`, `HeadAdapter`).
- Your own robot body ([Your own robot body](docs/BUILDING.md#your-own-robot-body)). A mobile base or a humanoid profile
  is a file of yours in `<ROBORUN_HOME>/bodies/<id>.yaml` (or `.yml`, `.json`). It is checked when it is read, and a
  mistake names the file and the field. Its class is named with `roborun up --body file.py:Class` or
  `package.module:Class` (or `body:` in `robot.yaml`), and `roborun up` remembers it per profile. `roborun profiles`
  lists your bodies and says what is wrong with any. `/v1/status` gains `body_kind`. `examples/rover.py` (with
  `my_rover.yaml` and `my_rover.xml`) runs offline, in the MuJoCo room and under `roborun up --sim room`.
- Camera cards ([Describe your cameras](docs/BUILDING.md#describe-your-cameras)). Each camera is described by one card
  (`roborun.camera/1`): where it is mounted (`world`, `left.tool`, `right.tool`, or a body's head), the image it delivers
  (size after `rotate`, lens, pinhole, depth) and which task camera it answers as. Cameras in `robot.yaml` may have names
  of your own and an optional `card:` section (`alias`, `label`, `mount`, `pose`, `depth`, `rectified`, `sees`,
  `device`). `roborun cameras describe` (and `--json`) prints the cards and opens nothing. `roborun_sdk.camera_card`
  has the checks, the cards, the body head card and `outbound()`, the camera facts a wrist-camera job may send. A camera
  class's `open()` may return `rectified`; `outbound()` sends only rectified frames (intrinsics without lens distortion).
  In `roborun sim`, each camera with a `card:` section is drawn from its card: its size, its depth (a `depth: false`
  camera serves no depth and says so) and, for a wrist camera, the rig's `T_tool_camera`. `robot.yaml` camera entries
  without a card leave the simulation exactly as before (sizes, pixels, and no checks), and every arm keeps its wrist
  camera even when the profile lists no camera for it. The simulated camera service's status lists the cameras it
  draws, with their cards.

- Policy runs ([Run your own policy](docs/BUILDING.md#run-your-own-policy)): run your own policy, or a provider's, on your
  arms as a Roborun task. `roborun policy providers | check | run | runs | consent` and `roborun_sdk.policy` (`run`,
  `start`, `check`, `providers`, the `Provider` interface). The station gains a `policy` lease: the runner pushes timed
  chunks of joint and gripper rows, and the station checks each one (shape, age, seam) and plays it on its own tick with a
  per-tick clip, a tool speed cap per run (`policy: {max_tool_speed_m_s}` in `robot.yaml`, 0.25 m/s by default), a 1 s
  heartbeat and a hold whenever the policy is late. Stop ends a run on the next tick. Each run writes a task record, a
  recording and a dataset episode like any task (the episode follows the robot-wide setting, as every task's does: `roborun up --no-dataset` or `--no-data-sync`, or `config.json` `lerobot_export.enabled`; the run's own `--no-dataset` only turns it off); the provider's own timing stays in `log/policy/`, which is never
  uploaded. A policy run's own Stop (the page's Stop on the task, `Robot.stop_task`) stops it at the arms' station by its
  id, also while the model warms up (new station route `POST /supervisor/policy/stop`); `Robot.stop_status()`, `wait()` and
  `task()` count a running policy run, also while Roborun's backend is down. Looking for policy runs never holds up a Stop:
  `Robot.stop()` sends `POST /v1/stop` before it looks, and a damaged policy record or a `robot.json` that is not a JSON
  object sends every Stop (`stop()`, `stop_task()`, the page's Stop on a task, `roborun stop`) on to Roborun's backend. While the backend is down, has not answered within 2 s, or answers the Stop with anything but a 2xx JSON answer with no `error` in it (an error status, a 200 whose body carries an `error`, a broken or non-JSON reply), a robot-wide Stop (`Robot.stop()`, the page's and the viewer's Stop) takes the station's hold first, then stops each run on its own and names every one it could not; `stopped_tasks` also names a run that the Stop ended before it was answered (not one that finished on its own meanwhile). The viewer's "Now" line, `Robot.status()` and `roborun status` name a running policy run as the task. Whether a run writes a dataset episode is decided by the installed robot.json and its runtime's config.json only, never by a caller's own arguments. A run is recorded only when the robot records its tasks (the runtime's config.json `video_record`: enabled, its cameras, its disk caps), and its recording follows the same retention as every task's: before it starts, the oldest recordings (tasks' and policy runs' alike, only the recorder's own folders) are deleted until it fits `total_cap_mb`, by the installed runtime's own cleanup; during the run its frames stop at `run_cap_mb`, `day_cap_mb`, `min_free_mb` or a full disk, and the task record says which (`recording.capped`, `recording.why`) and lists what was deleted (`recording.cleanup`). With the recorder off, it leaves no recording and no episode, as every task; its episode goes into the installed robot's dataset, never one a caller's config= names. `--shadow` asks the policy and moves nothing. A remote provider needs consent before anything is sent
  (`--yes` for one run, `consent=True` in Python). Built-in providers: `hold` and `replay`; your own from a file, a
  module or a `roborun.policies` entry point. `examples/my_policy.py` sways a wrist. New in this release and not on the
  stable surface: it may change within 0.1.
- Run a DreamLabs policy ([Run a DreamLabs policy](docs/BUILDING.md#run-a-dreamlabs-policy)): the `dreamlabs` provider runs
  DreamLabs' Dream-1 (`dreamlabs/dream-1-beta`, on two YAM arms) or your own fine-tune through DreamLabs' client
  (`pip install 'roborun-sdk[dreamlabs]'`, `dreamlabs-client` 1.1.7 or later 1.x), with every Roborun limit, Stop and record
  of a policy run. Credentials stay in DreamLabs' own files and variables, and Roborun never reads the key. The consent text
  lists what goes to DreamLabs, and DreamLabs' client-side recorder stays off unless `--option record=true`. The base model
  runs only on arms whose joint convention was checked against DreamLabs' YAM configuration (the YAM today). dream-1-beta
  also gets each arm's tool pose, computed by Roborun in the tool frame of DreamLabs' own YAM kinematics (Apache-2.0,
  `policy/dreamlabs_yam.py`), and the consent names it. Requests are paced with `sync_above_ms` 3000 (an option), so a
  real round trip of about 0.6-1.0 s keeps the arms moving instead of stopping after every chunk.
  `examples/dreamlabs_policy.py`.
- `roborun dreamlabs export | upload | finetune | status` (`pip install 'roborun-sdk[dreamlabs-data]'`): your episodes in
  the dataset format DreamLabs accepts, checked with DreamLabs' own offline validator and with every video face-blurred.
  Each is uploaded to your own DreamLabs account only when you ask and confirm, can be fine-tuned there, and the
  fine-tuned model then runs through Roborun.
- Visio stereo cameras (General Intelligence Labs) are a camera source: `roborun_sdk.visio_camera:VisioCamera` reads a
  Visio device over USB serial, over its USB network link (`tcp://HOST:50001`) or from an MCAP recording, and serves the
  rectified left eye with depth from the device's own calibration ([Your own camera](docs/BUILDING.md#your-own-camera)).
  `pip install 'roborun-sdk[visio]'`. It never writes to the device.

**Changed**

- Data sync: when the API names a data store this robot has not synced to (`GET /v1/health` `store`), the robot uploads its
  history again, once. It asks at most every 10 minutes; an API that names no store, or does not answer, changes nothing.
- `card` is now a reserved key in a `robot.yaml` camera entry: it is the camera's card section. A camera class of your own
  that took a `card=` argument from `robot.yaml` no longer receives it; rename that setting (any other extra key still
  reaches your class's `__init__` as before).
- A camera in `robot.yaml` named other than `top`, `left` or `right` was refused (`unknown camera role`). It is now a
  camera of its own name: it is served and recorded under that name (the camera service lists it in a new `extra_roles`)
  and is not a task camera unless its card gives it an alias. A name that breaks the rule (`Top`, `overhead`) is refused
  in plain words. Frames gain `camera_id` and `rotate`. `roborun up --camera ROLE=SOURCE` takes ROLE from a camera of
  your own name for that run, and `roborun setup` leaves cameras of your own name, and any camera with a card, as you
  wrote them.
- `roborun stop` now also sends Stop straight to the arms' station, so it stops a policy run (or any lease) even while the
  Roborun service is down, and says which of the two it reached. It exits 2 when either refuses.
- An unknown body profile says which bodies there are instead of starting the arm runtime download. Arms named on the
  command line (`--rig`, `--left`/`--right`, `--sim` with an arm scene) and `--profile` of another body or of arms leave
  out `robot.yaml`'s `body:`, which is the class of its own profile only. `--body` refuses a class whose profile is not
  `--profile`, and a body class whose robot service does not start is stopped and not remembered.
- `pip install roborun-sdk` is now the light install: the client, your own drivers and rules and the `roborun` command
  (PyYAML, packaging, roborun-station). numpy, MuJoCo, OpenCV, the CAN/serial and YAM packages move to extras;
  `roborun-sdk[robot]` is the old full set with its pins, and the installer (and `roborun upgrade`) installs it. A missing
  package names its extra ([Install only what you need](docs/BUILDING.md#install-only-what-you-need)).
- OpenArm 1.0 (`openarm_v1`): the DM-J4340 speed range is now 10 rad/s, and only the jaw ends are calibrated (the joint
  zeros are the maker's).
- The history upload now also sends the robot's telemetry by default: one file per hour (`log/telemetry`, uploaded as
  `logs`) with the arms' joint positions, jaws, speeds and motor temperatures once a second, task or not, and events such
  as Home or a fault (no images). `roborun up --no-data-sync` turns the whole upload off, telemetry included
  ([Privacy and your data](docs/BUILDING.md#privacy-and-your-data)).
- A `~/roborun` that holds `datasets/` beside the runtime is now taken as Roborun's own install: runtime updates apply
  there (before, they were refused with "holds other files" or skipped) and keep `datasets/`.
- Because of that, `roborun uninstall --purge` (or yes when it asks) now deletes `~/roborun` with `~/roborun/datasets`.
  Before, an install with a dataset kept `~/roborun` whatever the answer. Without `--purge` (or with `--keep-data`) the
  dataset is still kept, as before.
- `Robot.start` and `Robot.run` take `camera=`: the camera a task's motion is generated from, `top` (the overhead camera,
  the default) or a wrist camera, `left` or `right`, which plans its own arm. `POST /v1/tasks` takes the same `camera`,
  and a task names its camera. The field is sent only when you give it, so a request without it is the same as before;
  a robot whose runtime predates it refuses the field.
- `Robot.start` and `Robot.run` take `marks=`: boxes `[x0, y0, x1, y1]` in the pixels of `frame('top')` for the object to
  pick (`target`) and where it goes (`place`). Use them when look-alikes stand together: the robot takes the marked object
  instead of choosing among them. `POST /v1/tasks` takes the same `marks`, and a task names its marks. The field is sent
  only when you give it; a robot whose runtime predates it refuses the field.

**Fixed**

- A simulation no longer runs in the live robot. Before, `roborun sim` used the runtime the robot had loaded, the robot's
  camera socket (which it deleted and bound again, even while the real camera service served it) and port 8091. It also
  wrote its exact overhead calibration into the live runtime's `log/`, where the real robot then took it as its newest
  calibration. `roborun up --sim` also rewrote the live runtime's `config.json` for the simulation. Now every simulation
  runs from its own copy of the runtime in `<Roborun home>/sim/`, with its own `config.json` and `log/`, made and
  refreshed from the live one, which is only read. `roborun up --sim` runs the robot's backend and the simulation from
  that copy on the robot's 8091 and camera socket. It refuses, with nothing changed, while the live robot's station or
  camera service runs. `roborun sim` alone uses its own station port (8191), its own `sim-camera.sock` and no backend
  (`--predict URL` to give one). Both refuse a port or camera socket that something serves. A left-over socket is
  removed only when a Roborun simulation made it (a `.owner` file beside it) or, under `roborun up --sim`, when it is
  the install's own camera socket and nothing serves it. `roborun sim` alone never uses port 8091 or a robot's
  `camera.sock`, even while they are free. `roborun sim --runtime FOLDER` runs from its own copy of that folder in
  `<Roborun home>/sim/runtimes/` and never writes into it, whether it is the live runtime or another install's. A
  humanoid simulation (`roborun up --profile booster_k1 --sim room`) is refused, with nothing stopped, while the live
  arms station, camera service or a real robot's service runs; before, it stopped them without parking the arms. `roborun sim --sessions DIR`
  refuses a folder inside the robot's runtime or its data sync's folders (those sessions would be uploaded as real
  ones). The simulated station's `/status` says `sim: true`. Simulated tasks are recorded in the copy's `log/`, and the history upload
  (task records, recordings and telemetry) follows that copy while simulating ([Profiles and simulation](docs/BUILDING.md#profiles-and-simulation)).
- A second Roborun install on the same computer no longer touches the first one. Its systemd units, sockets and data
  folders now follow its own Roborun home (`ROBORUN_HOME`, or a `$HOME` other than the account's), with a short suffix.
  The default install keeps its plain unit names, so upgrading does not leave units running without an owner.
  `roborun up`, `down`, `upgrade` and `uninstall` from a second install refuse paths that resolve into the default install
  ([Two installs on one computer](docs/BUILDING.md#two-installs-on-one-computer)). Still open: a second install whose
  files are deliberately linked into the default install's files one level deeper, and `hw calibrate`, `cameras` and
  `profile create`, which do not have this check yet.
- The camera socket now fits in a Unix socket path. A second install under a deep `XDG_RUNTIME_DIR` (a sandbox's, for
  example) used to get a `camera.sock` path longer than the 107 bytes Linux allows (103 on macOS). The camera service or
  simulation could not listen there, and a simulated task failed at 0 s. Now, when the install's socket folder has no
  room for its sockets (`camera.sock`, `sim-camera.sock` and the RealSense capture helper's `camera-capture.sock`), they
  go in `/tmp/rr-<uid>-<12 hex>/` instead, a folder only you can open (0700). The folder is per user, per install and
  per runtime folder, and it is refused if someone else made it. The camera service, the simulation, `roborun up`
  (which passes the path to the robot's runtime as `MANIFEST_CAMERA_SOCKET`) and every reader work out the path with
  the same rule. Short paths are unchanged.
- A simulation removes only the camera socket it made. Before, `roborun sim` recognised its socket by the inode number
  alone. Filesystems such as ext4 give a deleted file's number to the next file made, so on exit a simulation could
  delete a file, or another program's socket, made at the same path after its own was gone. It now also checks the
  device, the file times (including a timestamp it sets on its own socket) and that the path is still a socket.
- A CAN link that USB recreates (a hub replug or a power blip re-enumerates the adapter) comes back in the CAN FD mode it
  last ran in. Before, it was raised as classic CAN, and arms whose motors are set up for CAN FD (OpenArm) answered
  nothing until the link was set up again by hand. The mode each link last ran in is kept in
  `<Roborun home>/can-links.json`; a link never seen before still comes up classic, as before.

## 0.1.20261121012822 (build 20261121-012822-8f60cdaa99)

- OpenArm 1.0 (Enactic): the `openarm_v1` catalog arm and its model. Damiao arms follow the CAN link's mode, so motors
  set up for CAN FD answer.

## 0.1.20261120133512 (build 20261120-133512-7ce45000da)

- Setup no longer traps a person with no arm. Enter means simulated arms. A rig without a depth camera ends setup with
  one line.
- Docs: the `no_depth_camera` error code (tasks need a depth camera; a webcam only shows and records).

## 0.1.20261120113745 (build 20261120-113745-3b1e5d8486)

- `--profile yam` keeps two arms only on a robot that an older SDK set up as two YAMs. Otherwise it is one arm, as the
  docs say.

## 0.1.20261120113207 (build 20261120-113207-85e9a1251d)

- Profiles: `yam` is one YAM arm and `yam_dual` is two (like `so101` / `so101_dual`). `yam_single` and `makerarm` still
  work and say what they were renamed to.
- The simulator obeys the `rules:` file like a real arm. The station refuses a move on an arm that is not OK and gives the
  arm's own reason.
- `roborun --help` lists 14 commands. The hardware checks are under `roborun hw`. `roborun login` prints an approval
  link.
- Guided setup asks fewer questions. A Stop holds a freely moving jaw where it is measured.
- Datasets: columns a robot never reports are not written, and refused episodes are counted in `roborun status` and
  `/v1/status`.

## 0.1.20261120093759 (build 20261120-093759-ef6d6fb600)

**Changed**

- Humanoid profiles: a task starts with `POST /v1/tasks`, like on an arm profile (`POST /v1/body/move` stays as an alias).
  The humanoid local API's error codes are now the arm API's: `invalid`, `not_found` and `busy` replace
  `invalid_request`, `bad_json`, `unknown_move` and `move_running`. A program that branched on the old codes must change.

**Added**

- The robot's LeRobot v2.1 dataset: each task or humanoid move is an episode, stored in `~/roborun/datasets/<robot>/`
  (uninstall keeps it).

## 0.1.20261120085438 (build 20261120-085438-217073e79b)

- The robot's dataset can be synced to the account, listed by episode, downloaded and deleted by its owner.

## 0.1.20261120085045 (build 20261120-085045-b944fbc2d7)

- Humanoid room simulation: the body starts 2.6 m from the table, so a first "drive forward 1 meter" completes.

## 0.1.20261120084821 (build 20261120-084821-fb38181ecf)

- Humanoid moves say why the obstacle guard could not see, and how to reach something out of view.

## 0.1.20261120082915 (build 20261120-082915-015635251a)

- Humanoid guard: tall bodies drive their full distance, and Stop confirms.
- Setup stops a running Roborun through its station, as `roborun down` does. It keeps a camera you wrote yourself.
- Both arms set to one CAN or serial adapter is reported as a settings mistake before either arm is opened.

## 0.1.20261120081108 (build 20261120-081108-cc9b4e92d3)

- Humanoids get an obstacle guard: head depth stops a walk before it reaches something.
- Docs: `object_not_measured` and `task_failed`. A failed task always carries `error_info`.

## 0.1.20261120075628 (build 20261120-075628-27f9806995)

- The humanoid move record says what the body measured it did.

## 0.1.20261120075544 (build 20261120-075544-504cc56d0e)

- Humanoid moves land where they should, and the move record says what they did.

## 0.1.20261120070335 (build 20261120-070335-670d7c3d5d)

**Added**

- `Robot.body_run()`: a humanoid move, waited for.
- `roborun uninstall` and a local viewer (`roborun view`). Roborun keeps running after logout.
- Arm connection: Damiao mode repair, YAM teaching handle, LeRobot calibration import, CAN bus-load warning and
  `roborun motor`.
- The humanoid local API serves the body's 3-D display model (`GET /v1/body/model`).

## 0.1.20261119235416 (build 20261119-235416-9cb49890e1)

- Humanoid local API: `GET /v1/body` reports `arms_offered`.

## 0.1.20261119231551 (build 20261119-231551-06dca45424)

- Setup does not stop when no camera is plugged in. It says so in one line and carries on.

## 0.1.20261119224249 (build 20261119-224249-5f8e52cbb0)

**Removed** (a break within the 0.1 series)

- `Robot.clip()` and the `Task` field that went with it. A program that called `Robot.clip()` got `AttributeError` from
  this release on. The next release after 0.1.20261121012822 brings `Robot.clip()` back as a stub
  that raises a clear `RobotError` (code `removed`). For what the cameras saw during a task, use
  `Robot.recording(task_id)`.

## 0.1.20261119203708 (build 20261119-203708-36d943d3cd)

The first release in the 0.1 series on `https://roborun.app/dl/simple/`. Each release has a PEP 440 version and a build
id from the same time, and `roborun --version` prints the pin. It includes the typed client `roborun_sdk.client`, the
docs and examples shipped in the wheel (`roborun docs`), and `roborun new`.
