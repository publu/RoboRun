"""Fleet communication demos — coordinating many quadrupeds over an imperfect
radio, the runnable twin of the in-browser sandbox at ``/fleet``.

    from roborun.swarm import Fleet, STRATEGIES, run_episode

The model lives in ``comms.py`` (radio range, airtime, onboard memory, one goal
at a time); the four strategies live in ``strategies.py``. Run a headless
episode with::

    python -m roborun.swarm                 # compare all four strategies
    python -m roborun.swarm gossip          # just one, verbose
"""
from .comms import Fleet, Robot, Message, WORLD, CG, NCELL
from .strategies import STRATEGIES
from .runner import run_episode

__all__ = ["Fleet", "Robot", "Message", "STRATEGIES", "run_episode",
           "WORLD", "CG", "NCELL"]
