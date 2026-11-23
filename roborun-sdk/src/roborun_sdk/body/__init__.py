"""Humanoid bodies: a mobile base and a head camera, driven by the hosted API's body jobs.

    adapters.py   BaseAdapter, HeadAdapter (write your own), and the robot-side agent client for a real K1 or R1
    sim.py        the MuJoCo simulation of a body (the packaged models in data/body_models)
    profiles.py   the body profiles (data/bodies/<id>.json, and yours in <ROBORUN_HOME>/bodies/<id>.yaml) and their descriptor
    custom.py     your own body class: roborun up --body my_body.py:MyBody (or body: in robot.yaml)
    jobs.py       the body-job client: frame + prompt -> signed locomotion timeline, verified
    player.py     plays a timeline on the adapters at 50 Hz, Stop at any tick
    server.py     the local robot API of a humanoid profile (127.0.0.1:8120)
    up.py         roborun up --profile booster_k1 | galaxea_r1 | <your own>
"""
from .adapters import BaseAdapter, HeadAdapter  # noqa: F401
from .profiles import ProfileError, UnknownBody, descriptor, ids, is_body  # noqa: F401
