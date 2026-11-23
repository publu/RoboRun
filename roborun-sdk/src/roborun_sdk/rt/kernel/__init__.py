"""The local safety kernel K1-K8, one module per check:

- K1 ``stop``: latch first, every arm's ``hold()`` in parallel with a 300 ms budget, then cancel, then report per-arm
  acks from measured state (never an assumed ``holding: True``).
- K2 ``link``: the link watchdog (LINKED/LATE/LOST/HELD/BACK); grants live in the runtime.
- K3 ``limits``: joint limits -0.01 rad (inward-only past a limit, as the robot runtime's own check), per-tick steps, per-point
  clamp, EE speed, jaw step/lead/range.
- K4 ``proximity``: inter-arm >= 20 mm + 0.45 m/s x oldest sample age on fresh samples; ``MixedSafetyProximity`` runs
  the robot runtime's own mesh check (MuJoCo convex hulls) for the YAM/Metal rig, ``CapsuleProximity`` the pack's capsules.
- K5 ``reflexes``: segment triggers on locally computed signals, force cap, tracking pause.
- K6 ``health``: liveness (3 cycles / 150 ms), fault latch, thermal thresholds.
- K7 lives in ``roborun_sdk.rt.clock``.
- K8 ``interlocks``: ``held`` / ``pending_release`` per arm, persisted <= 12 h, seeded read-only from the robot runtime's
  jaw memory.
"""
