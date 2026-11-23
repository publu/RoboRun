"""Software exposure for a camera whose settings say `exposure: {managed: true}`.

The camera's own auto-exposure is turned off. While the robot is idle this loop nudges the exposure toward a target median
brightness; while a task runs it holds still, so brightness does not drift under the perception that is using the image.
"""
import math
import time

from ._optional import lazy

np = lazy('numpy', 'cameras', globals())

DEFAULTS = dict(target_median=110., limit_us=20000., seed_us=8000., smooth_idle=.15)
PERIOD_S = .5          # a new exposure takes a few frames to show up in the image
DEADBAND = .06         # closer than this to the target: leave it
MIN_US = 50.
SETTLE_GAIN = .7       # the first approach, before there is anything to keep steady


def median_brightness(rgb):
    return float(np.median(rgb[::8, ::8, 1]))          # green carries most of the luminance


class ExposureController:
    def __init__(self, settings, apply, busy=None, clock=time.monotonic):
        s = dict(DEFAULTS, **{k: v for k, v in (settings or {}).items() if v is not None})
        self.target, self.limit = float(s['target_median']), float(s['limit_us'])
        self.smooth = float(s['smooth_idle'])
        self.apply, self.busy, self.clock = apply, busy or (lambda: False), clock
        self.us = max(MIN_US, min(float(s['seed_us']), self.limit))
        self.at, self.settled, self.held, self.median = None, False, False, None

    def start(self):
        self.apply(self.us)
        self.at = self.clock()
        self.settled = False

    def update(self, rgb):
        now = self.clock()
        if self.at is None or now - self.at < PERIOD_S:
            return
        self.at = now
        self.median = median_brightness(rgb)
        self.held = bool(self.busy())
        if self.held:
            return
        error = math.log(self.target / max(self.median, 1.))
        if abs(error) < math.log1p(DEADBAND):
            self.settled = True
            return
        gain = self.smooth if self.settled else SETTLE_GAIN
        wanted = max(MIN_US, min(self.limit, self.us * math.exp(error * gain)))
        if abs(wanted - self.us) >= 1.:
            self.us = wanted
            self.apply(self.us)

    def status(self):
        return dict(managed=True, us=round(self.us), median=self.median, target=self.target, limit_us=self.limit, held=self.held)
