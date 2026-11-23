# Third-party notices

Third-party components in this repository keep their own licenses. Each package lists them:

- [`roborun-sdk/THIRD_PARTY_NOTICES.md`](roborun-sdk/THIRD_PARTY_NOTICES.md): the vendored i2rt YAM driver and models
  (MIT), the YuNet face detector (MIT), the Ed25519 fallback adapted from RFC 8032 section 6 (Revised BSD License,
  Copyright (c) 2017 IETF Trust and the persons identified as authors of the code;
  `roborun-sdk/src/roborun_sdk/rt/_ed25519.LICENSE`), the robot arm models (each folder in
  `roborun-sdk/src/roborun_sdk/data/arm_models/` names its source, commit and license, with the license text beside it,
  and, where attached, the Robotiq 2F-85 gripper or Franka hand from MuJoCo Menagerie in `LICENSE.gripper.txt`; the
  OpenArm v1 and v2 models are Copyright 2026 Enactic, Inc., Apache License 2.0) and the
  Booster K1 model (BSD 3-Clause), and the public-domain test photo.
  The arm models are modified: converted to MJCF by `roborun-sdk/tools/arm_models/build.py`, meshes converted to STL and
  simplified to about 500 faces per link, joints and frames renamed, and where `attached_gripper` is set a MuJoCo
  Menagerie gripper attached; each model file's first line says so.
- [`roborun-station/THIRD_PARTY_NOTICES.md`](roborun-station/THIRD_PARTY_NOTICES.md): no third-party code.
