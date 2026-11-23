# Third-party notices

- **i2rt** (YAM arm driver and models), MIT License, Copyright (c) I2RT Robotics. See
  `src/roborun_sdk/third_party/i2rt/LICENSE` and `VENDORED.md`.
- **YuNet face detector**, MIT License, Shiqi Yu and contributors, from OpenCV Zoo. See
  `src/roborun_sdk/data/models/face_detection_yunet.LICENSE`.
- **Ed25519 fallback** `src/roborun_sdk/rt/_ed25519.py`: adapted from the Python reference implementation in
  RFC 8032, section 6. Revised BSD License, Copyright (c) 2017 IETF Trust and the persons identified as authors of the
  code. See `src/roborun_sdk/rt/_ed25519.LICENSE`.
- **Robot arm models** in `src/roborun_sdk/data/arm_models/`: each folder has an `arm.json` naming its source, commit and
  license, with the license text beside it. These models are modified: converted to MJCF by
  `tools/arm_models/build.py`, meshes converted to STL and simplified to about 500 faces per link, joints and frames
  renamed, and, where `attached_gripper` is set, a gripper from MuJoCo Menagerie attached (its license is in
  `LICENSE.gripper.txt` beside the model).
  - **OpenArm v1 and v2 models** (`openarm_v1/`, `openarm_v2/`): Copyright 2026 Enactic, Inc., Apache License 2.0, from
    github.com/enactic/openarm_description. The notice is repeated in the first line of each model file and in `arm.json`.
- **Booster K1 model** in `src/roborun_sdk/data/body_models/booster_k1/`, BSD 3-Clause License, Copyright (c) 2025,
  BoosterRobotics, from github.com/BoosterRobotics/booster_assets (unmodified; `source.json` names the commit and each file's
  sha256, with the license text beside it). The Galaxea R1 simulation uses a stand-in made from plain shapes for this SDK
  (`body_models/galaxea_r1/source.json`).
- **Test photo** `tests/data/astronaut_face.jpg` (tests only, not in the installed package): taken from the "astronaut"
  sample image of scikit-image (`skimage.data.astronaut()`), a NASA photograph of Eileen Collins in the public domain.
