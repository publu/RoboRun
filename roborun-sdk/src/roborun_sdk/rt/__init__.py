"""roborun-rt: the motor-side process.

Holds the segment player, the safety kernel K1-K8, the Stop latch and listeners, and the arm connectors.
Must never hold torch, transformers or cv2 (``guard.install()`` blocks them; ``guard.loaded()`` proves it).
Nothing here imports a model, an IK solver, a calibration fitter or a cloud socket.
"""
