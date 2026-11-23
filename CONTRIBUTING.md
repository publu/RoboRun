# Contributing

Issues and pull requests are welcome. Contributions we especially want: **arm drivers, cameras, examples and docs**.

## Issues

Use the templates:

- **Bug report**: what you ran, what you expected, what happened, `roborun --version`, your OS and, for a real robot,
  the arm or camera model. Leave out keys and tokens (`~/.config/roborun/token`) and anything in your camera images you do
  not want to share.
- **New arm, camera or robot support**: the model, how it connects (USB serial, CAN, Ethernet, the maker's SDK), links to
  its SDK and URDF, and whether you can test on one.

Security problems go through [SECURITY.md](SECURITY.md), not issues or pull requests.

## Pull requests

1. For anything larger than a small fix, open an issue first so we can agree on the approach.
2. Keep a pull request to one change, with tests: new code should come with tests that run in simulation or against fakes
   (see `roborun-sdk/tests/` for drivers and cameras tested without hardware).
3. Run the tests (below) before you open it; CI runs the same selection on Linux (required) and macOS (reported, not
   yet required: see [Known test issues](#known-test-issues)).
4. Say in the description what you tested on real hardware, if anything (arm or camera model, OS).

Where things go:

- **Arm drivers**: `roborun-sdk/src/roborun_sdk/arms/`, with a catalog entry in `roborun-sdk/src/roborun_sdk/data/arms/`;
  the steps are in [Adding an arm to the catalog](roborun-sdk/DEVELOPING.md#adding-an-arm-to-the-catalog) and the
  interface in [`BUILDING.md`, your own arm driver](roborun-sdk/docs/BUILDING.md#your-own-arm-driver).
- **Cameras**: [`BUILDING.md`, your own camera](roborun-sdk/docs/BUILDING.md#your-own-camera).
- **Examples**: `roborun-sdk/examples/`, small and runnable without a key where possible.
- **Docs**: `roborun-sdk/docs/`, the package READMEs and `DEVELOPING.md` files.
- **Robot models** (URDF/MJCF and meshes): only with a license that allows redistribution; record the source, commit and
  license next to the files, as the existing folders in `roborun-sdk/src/roborun_sdk/data/arm_models/` do.

Safety: never weaken Stop, hold, limits or rules to make something work; follow the
[rules for changes](roborun-sdk/DEVELOPING.md#rules-for-changes). A change that touches motion must keep the safety tests
passing and should add one for the new case.

## License of contributions

This repository is licensed under the Apache License 2.0 ([LICENSE](LICENSE)). By submitting a contribution you agree
that it is licensed under the same license (inbound = outbound, as in section 5 of the license), and that you have the
right to submit it. Code you copy from elsewhere must keep its license and notices; add it to
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and the package's own `THIRD_PARTY_NOTICES.md`.

## Running the tests

Everything runs in simulation with fake hardware; no arm, camera or Roborun key is needed. From a clone, in a fresh
Python 3.10+ virtual environment:

```sh
pip install "./roborun-station" "./roborun-sdk[robot,test]" pytest-xdist pytest-timeout
cd roborun-sdk && python -m pytest -q -n auto --timeout=180 \
  --ignore=tests/test_guided_setup.py
cd ../roborun-station && python -m pytest -q
```

This is the selection [the CI workflow](.github/workflows/ci.yml) runs; [Tests in the README](README.md#tests) explains the
separate guided-setup suite, and
[`roborun-sdk/DEVELOPING.md`](roborun-sdk/DEVELOPING.md#run-the-tests) shows how to run them without network. If you work on the SDK from source while a real robot runs under
the same user, read [Run it from source](roborun-sdk/DEVELOPING.md#run-it-from-source) first.

## Known test issues

- **macOS.** The CI job on macOS runs the same selection as on Linux, but the suite has not passed on macOS yet, so that
  job does not fail the workflow for now (its result still shows in every run). If you work on a Mac and a test fails
  there, please open an issue with the output. Once the job is green it becomes required.
- **Parallel runs (`-n auto`).** Each test process gets its own temporary folder and the tests that open fake adapters
  use their own Roborun home, so the bus and arm locks of one test process never block another. If a test fails only
  under `-n auto` and passes when run alone, please open an issue with the output of both runs: it is a bug, not noise.
- **`tests/test_arm_yam.py` run on its own** prints `TypeError: 'NoneType' object does not support the context manager
  protocol` after the summary, at interpreter exit. It comes from python-can's virtual bus being garbage-collected after
  its module was torn down; the tests have already passed and the process exits normally.
