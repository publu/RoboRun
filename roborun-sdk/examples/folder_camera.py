"""Your own camera: any class with read() returning an RGB image. This one plays the JPEG/PNG images of a folder in a loop,
which is handy for replaying a recording; swap read() for your camera's SDK.

    cameras:
      top:
        source: /path/to/folder_camera.py:FolderCamera
        folder: /path/to/images        # every other key in the camera's section is passed to __init__
        fps: 10

Roborun constructs it with whichever of role, width, height, fps its __init__ names, plus the extra keys. Optional methods:
open() -> {serial, intrinsics, depth_scale_m}, close(), set_exposure_us(us). To give depth, return
{rgb, depth_z16} from read() and depth_scale_m from open(); the overhead camera needs depth.

    python folder_camera.py /path/to/images     # opens it the way Roborun does and reads three frames
"""
import time
from pathlib import Path

import cv2


class FolderCamera:
    def __init__(self, folder, role='top', fps=10):
        self.files = sorted(p for p in Path(folder).expanduser().iterdir() if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
        if not self.files:
            raise ValueError(f'no images in {folder}')
        self.role, self.period, self.index, self.next_at = role, 1.0 / float(fps), 0, 0.0

    def open(self):
        return dict(serial=f'folder:{self.files[0].parent.name}')

    def read(self):
        now = time.monotonic()
        if now < self.next_at:
            return None                                   # no new frame yet
        self.next_at = now + self.period
        image = cv2.imread(str(self.files[self.index % len(self.files)]))
        self.index += 1
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


if __name__ == '__main__':
    import sys

    from roborun_sdk.cameras import CustomSource

    source = CustomSource(f'{Path(__file__).resolve()}:FolderCamera', 'top', settings=dict(fps=10), extra=dict(folder=sys.argv[1]))
    model = source.open()
    print('opened:', {k: model[k] for k in ('serial', 'width', 'height') if k in model})
    frames = 0
    while frames < 3:
        item = source.read()
        if item is not None:
            frames += 1
            print('frame', item['sensor_frame'], item['rgb'].shape, item['rgb'].dtype)
        time.sleep(0.02)
    source.close()
