"""Face blur where frames leave this computer.

Inside this computer everything stays sharp: what the robot sees, recordings on disk and the local live view. Every image or
depth frame that leaves goes through this module first: motion job input frames, wrist looks, every other image sent to the API,
and the history upload. It fails closed: if the blur cannot run nothing is sent, while the robot keeps seeing and Stop keeps
working.

`privacy: {face_blur: false}` in robot.yaml is the user's explicit choice. With it, frames sent to the API for motion and looks
go unblurred, because the user chose that, and history images and depth are not uploaded at all (they are withheld and counted;
task records still upload). Recordings on disk are sharp either way.

The detector is YuNet (Shiqi Yu et al., MIT licence, data/models/face_detection_yunet.LICENSE), from the OpenCV Zoo
(models/face_detection_yunet/face_detection_yunet_2023mar.onnx), run through OpenCV. Its hash is checked at every load. It also
fires on the robot's own gripper, on cups and on shelves, and a blur on the object a task is about can change what the task
does. So a weak hit is dropped when it lies on the robot's own body (where its pose is known) or when aligned depth shows that
no head could be there, and only its own box is hidden when depth leaves the question open: see `head_share`. Without good
depth the hit is blurred in full.

Depth inside a blurred region is set to 0, the sensor's "no reading": the shape of a head in depth also identifies. A frame in
which nothing is found goes out byte for byte as it was.
"""
import collections
import hashlib
import io
import json
import os
import threading
from importlib import resources
from pathlib import Path

import numpy as np

MODEL_FILE = 'face_detection_yunet_2023mar.onnx'
MODEL_SHA256 = '8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4'
MIN_SCORE = 0.6        # measured on sample footage: real faces scored from 0.66, so a higher bar would let some through
NMS = 0.3
MAX_FACES = 50
DETECT_LONG_SIDE = 640 # larger frames are searched at this size; cost grows with the area
MARGIN = 0.3           # of the face box, on every side: hair, ears and a box that lags a moving head
HOLD_FRAMES = 3        # in a video, a face found in one of the last frames stays blurred through a one-frame miss
COVER_CELLS = 3        # the box is averaged down to this many cells across, then smoothed: on sample footage the detector
                       # could still find 194 of 329 faces under a finer 8-cell mosaic, and 2 of 329 under this
ROBOT_SHARE = 0.6      # a weak hit with at least this share of its box on the robot's own body is the robot, not a person
ROBOT_MAX_SCORE = 0.8  # measured on sample footage: hits on the arm stay below this, faces seen by the wrist cameras mostly above it
FACE_WIDTH_M = 0.09, 0.35 # the real width a face box can have, from a child's face turned away to a head with hair and margin
HEAD_DEPTH_M = 0.20    # how far front to back one head reaches
HEAD_SHARE = 0.65      # at this share of the box or more, one head-deep surface can be a head: blurred in full
NO_HEAD_SHARE = 0.40   # below this share nothing there can be a head. On 845 weak hits with depth from sample recordings, the real
                       # faces, some half behind a laptop, reached 0.60 and up; between the two, only the box itself is hidden
JPEG_QUALITY = 95
NO_BLUR = 'Face blur could not run, so no camera image was sent'


class Cleared(bytes):
    """Bytes that went through this module and may leave this computer. Senders accept nothing else."""


class BlurUnavailable(Exception):
    """The blur cannot run: nothing may be sent."""


def model_path():
    return Path(str(resources.files('roborun_sdk') / 'data' / 'models' / MODEL_FILE))


def verified_model(path=None):
    path = Path(path or model_path())
    try:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as exc:
        raise BlurUnavailable(f'the face detector model is missing ({exc})') from exc
    if digest != MODEL_SHA256:
        raise BlurUnavailable(f'the face detector model at {path} is not the expected file')
    return path


class FaceBlur:
    """The detector and the cover. It keeps per-image state: one thread at a time."""

    def __init__(self, path=None, min_score=MIN_SCORE):
        model = verified_model(path)
        try:
            import cv2
            cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
            self.detector = cv2.FaceDetectorYN.create(str(model), '', (320, 240), min_score, NMS, MAX_FACES)
        except Exception as exc:
            raise BlurUnavailable(f'the face detector could not start ({type(exc).__name__}: {exc})') from exc
        self.cv2, self.size = cv2, None
        self.recent = collections.deque(maxlen=HOLD_FRAMES)

    def detect(self, rgb):
        """Face boxes in this image's pixels, as [[x, y, w, h], ...]."""
        return self.detect_scored(rgb)[0]

    def detect_scored(self, rgb):
        """(boxes, their scores)."""
        cv2 = self.cv2
        h, w = rgb.shape[:2]
        scale = min(1., DETECT_LONG_SIDE / max(h, w))
        image = rgb if scale == 1. else cv2.resize(rgb, (max(1, round(w * scale)), max(1, round(h * scale))), interpolation=cv2.INTER_AREA)
        size = image.shape[1], image.shape[0]
        if size != self.size:
            self.detector.setInputSize(size)
            self.size = size
            self.recent.clear()
        faces = self.detector.detect(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))[1]
        return (np.zeros((0, 4)), np.zeros(0)) if faces is None else (faces[:, :4] / scale, faces[:, -1])

    def _hide(self, out, box):
        cv2 = self.cv2
        x0, y0, x1, y1 = covered_region(box, out.shape)
        if x1 <= x0 or y1 <= y0:
            return
        patch = out[y0:y1, x0:x1]
        size = x1 - x0, y1 - y0
        small = cv2.resize(patch, (COVER_CELLS, COVER_CELLS), interpolation=cv2.INTER_AREA)
        cover = cv2.resize(small, size, interpolation=cv2.INTER_NEAREST)
        out[y0:y1, x0:x1] = cv2.GaussianBlur(cover, (0, 0), max(size) / 3)

    def find(self, rgb, robot=None, depth=None, shape=None):
        """Face boxes, without the weak hits that are not faces. `robot`: a boolean image of the robot's body, any size. `depth`:
        what `depth_evidence` returned, for an image of size `shape` (default: this image). Either may be None when unknown."""
        if robot is None and depth is None:
            return self.detect(rgb)
        boxes, scores = self.detect_scored(rgb)
        target = shape or rgb.shape
        out = []
        for box, score in zip(boxes, scores):
            kind = weak_hit(box, score, target, robot, depth, (target[1] / rgb.shape[1], target[0] / rgb.shape[0]))
            if kind != 'not a face':
                out.append(tight(box) if kind == 'unsure' else box)
        return np.asarray(out, float).reshape(-1, 4)

    def cover(self, rgb, boxes):
        """A copy with these boxes hidden; the image itself when there are none."""
        if not len(boxes):
            return rgb
        out = rgb.copy()
        for box in boxes:
            self._hide(out, box)
        return out

    def track(self, rgb, robot=None):
        """(boxes to hide, faces found in this frame), for consecutive frames of one camera."""
        boxes = self.find(rgb, robot)
        self.recent.append(boxes)
        return [box for frame in self.recent for box in frame], len(boxes)

    def apply(self, rgb, robot=None):
        """(image with every face hidden, faces found in this frame), for consecutive frames of one camera."""
        hide, found = self.track(rgb, robot)
        return self.cover(rgb, hide), found


def on_robot(box, shape, robot):
    h, w = shape[:2]
    mask = np.asarray(robot, bool)
    sy, sx = mask.shape[0] / h, mask.shape[1] / w
    x0, y0 = max(0, int(box[0] * sx)), max(0, int(box[1] * sy))
    x1, y1 = min(mask.shape[1], int(np.ceil((box[0] + box[2]) * sx))), min(mask.shape[0], int(np.ceil((box[1] + box[3]) * sy)))
    return x1 > x0 and y1 > y0 and mask[y0:y1, x0:x1].mean() >= ROBOT_SHARE


def weak_hit(box, score, shape, robot=None, depth=None, scaled_by=(1., 1.)):
    """'face' (hide it with its margin), 'unsure' (hide exactly its box) or 'not a face' (leave it)."""
    if score >= ROBOT_MAX_SCORE:
        return 'face'
    box = box[0] * scaled_by[0], box[1] * scaled_by[1], box[2] * scaled_by[0], box[3] * scaled_by[1]
    if robot is not None and on_robot(box, shape, robot):
        return 'not a face'
    share = None if depth is None else head_share(box, shape, depth)
    if share is None or share >= HEAD_SHARE:
        return 'face'
    return 'unsure' if share >= NO_HEAD_SHARE else 'not a face'


def tight(box):
    """The box that, once grown by the margin, covers exactly `box`."""
    x, y, w, h = box
    grown = 1 + 2 * MARGIN
    return [x + MARGIN * w / grown, y + MARGIN * h / grown, w / grown, h / grown]


def depth_evidence(rgb_shape, depth, metadata):
    """(depth image, focal length in pixels, metres per unit) when this depth is aligned to this colour image and says how to
    read it; None otherwise, and then nothing is concluded from it."""
    try:
        intrinsics, scale = metadata['intrinsics'], float(metadata['depth_scale_m'])
        fx, width = float(intrinsics['fx']), int(intrinsics['width'])
        depth = np.asarray(depth)
    except (KeyError, TypeError, ValueError):
        return None
    good = depth.dtype == np.uint16 and depth.shape == tuple(rgb_shape[:2]) and width == rgb_shape[1]
    return (depth, fx, scale) if good and 0 < fx < 1e5 and 0 < scale < 1 else None


def head_share(box, shape, evidence):
    """How much of this box one head could fill: the largest share of it on a single surface, one head deep, at a distance where
    the box would be face-sized. Pixels without a reading count as possibly the head, so holes in the depth raise it."""
    depth, fx, scale = evidence
    x, y, w, h = box
    x0, y0, x1, y1 = max(0, int(x)), max(0, int(y)), min(shape[1], int(np.ceil(x + w))), min(shape[0], int(np.ceil(y + h)))
    if w <= 0 or x1 <= x0 or y1 <= y0:
        return None
    patch = depth[y0:y1, x0:x1]
    known = np.sort(patch[patch > 0].astype(float) * scale)
    unknown = patch.size - known.size
    near, far = FACE_WIDTH_M[0] * fx / w, FACE_WIDTH_M[1] * fx / w
    starts = known[(known >= near - HEAD_DEPTH_M) & (known <= far)]             # the fullest window starts on a reading
    most = int((np.searchsorted(known, starts + HEAD_DEPTH_M, side='right') - np.searchsorted(known, starts)).max()) if starts.size else 0
    return (most + unknown) / patch.size


def frame_metadata(arrays):
    raw = arrays.get('metadata')
    if raw is None:
        return {}
    try:
        return json.loads(raw.tobytes() if raw.dtype.kind in 'uS' else str(raw))
    except (ValueError, AttributeError):
        return {}


def outbound_enabled():
    """False only for the user's explicit privacy.face_blur: false (roborun up sets ROBORUN_FACE_BLUR from robot.yaml)."""
    return os.environ.get('ROBORUN_FACE_BLUR', '1') != '0'


def covered_region(box, shape):
    """The pixels (x0, y0, x1, y1) a box hides, in an image of `shape`."""
    h, w = shape[:2]
    x, y, bw, bh = box
    grow_x, grow_y = bw * MARGIN, bh * MARGIN
    return max(0, int(x - grow_x)), max(0, int(y - grow_y)), min(w, int(x + bw + grow_x + 1)), min(h, int(y + bh + grow_y + 1))


def scaled(boxes, source, target):
    """Boxes of an image of shape `source` in the pixels of an image of shape `target`."""
    if not len(boxes) or tuple(source[:2]) == tuple(target[:2]):
        return boxes
    fx, fy = target[1] / source[1], target[0] / source[0]
    return np.asarray(boxes, float) * [fx, fy, fx, fy]


def hide_depth(depth, boxes, rgb_shape):
    """A copy of a depth image with no reading inside the hidden regions."""
    out = np.array(depth)
    for box in scaled(boxes, rgb_shape, depth.shape):
        x0, y0, x1, y1 = covered_region(box, depth.shape)
        out[y0:y1, x0:x1] = 0
    return out


_lock = threading.Lock()
_shared = None


def _guarded(work):
    """One blur at a time on the shared detector; every failure reads as `BlurUnavailable`, so callers send nothing."""
    global _shared
    with _lock:
        try:
            if _shared is None:
                _shared = FaceBlur()
            return work(_shared)
        except BlurUnavailable:
            raise
        except Exception as exc:
            raise BlurUnavailable(f'{NO_BLUR} ({type(exc).__name__})') from exc


def decode_jpeg(data):
    import cv2
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise BlurUnavailable(f'{NO_BLUR}: an image could not be read')
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def encode_jpeg(rgb, quality=JPEG_QUALITY):
    import cv2
    ok, data = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise BlurUnavailable(f'{NO_BLUR}: an image could not be written')
    return data.tobytes()


def _read_npz(raw):
    with np.load(io.BytesIO(raw), allow_pickle=False) as data:
        return {k: data[k] for k in data.files}


def _write_npz(arrays):
    out = io.BytesIO()
    np.savez_compressed(out, **arrays)
    return out.getvalue()


def clear_arrays(rgb, depth=None, robot=None):
    """(rgb, depth) ready to send, as arrays."""
    if not outbound_enabled():
        return rgb, depth

    def work(blur):
        boxes = blur.find(rgb, robot)
        return blur.cover(rgb, boxes), None if depth is None else depth if not len(boxes) else hide_depth(depth, boxes, rgb.shape)
    return _guarded(work)


def clear_frame(raw_npz, image=None, robot=None, local=None):
    """A frame file (rgb, depth_z16, metadata) and, optionally, the JPEG made from it, ready to send: (npz, jpeg or None).
    Both get the same hidden regions: whatever is found in either image is hidden in both. `local`: (depth, metadata) of this
    frame when the file itself carries no depth; it is only read here and is not sent."""
    if not outbound_enabled():
        return Cleared(raw_npz), None if image is None else Cleared(image)

    def work(blur):
        arrays = _read_npz(raw_npz)
        rgb = arrays['rgb']
        depth, metadata = local or (arrays.get('depth_z16'), frame_metadata(arrays))
        evidence = None if depth is None else depth_evidence(rgb.shape, depth, metadata)
        boxes = blur.find(rgb, robot, evidence)
        picture = None if image is None else decode_jpeg(image)
        if picture is not None:
            more = scaled(blur.find(picture, robot, evidence, rgb.shape), picture.shape, rgb.shape)
            boxes = np.concatenate([boxes, more]) if len(more) else boxes
        if not len(boxes):
            return Cleared(raw_npz), None if image is None else Cleared(image)
        arrays['rgb'] = blur.cover(rgb, boxes)
        if arrays.get('depth_z16') is not None:
            arrays['depth_z16'] = hide_depth(arrays['depth_z16'], boxes, rgb.shape)
        sent = None if picture is None else Cleared(encode_jpeg(blur.cover(picture, scaled(boxes, rgb.shape, picture.shape))))
        return Cleared(_write_npz(arrays)), sent
    return _guarded(work)


def clear_image(data, robot=None):
    """A JPEG or PNG ready to send: blurred if a face is in it, otherwise these same bytes."""
    if not outbound_enabled():
        return Cleared(data)

    def work(blur):
        rgb = decode_jpeg(data)
        boxes = blur.find(rgb, robot)
        return Cleared(data) if not len(boxes) else Cleared(encode_jpeg(blur.cover(rgb, boxes)))
    return _guarded(work)


def clear_arrays_file(raw):
    """An .npz holding camera images (and their depth), ready to send: these same bytes if no face is in it. Colour is
    HxWx3 or NxHxWx3 uint8; depth is uint16, HxW or NxHxW, matched to the colour frames. Anything else that looks like a
    picture cannot be cleared."""
    try:
        arrays = _read_npz(raw)
    except Exception as exc:
        raise BlurUnavailable(f'{NO_BLUR}: this file could not be read') from exc
    is_rgb = lambda a: a.dtype == np.uint8 and a.ndim in (3, 4) and a.shape[-1] == 3
    is_depth = lambda a: a.dtype.kind == 'u' and a.dtype.itemsize == 2 and a.ndim in (2, 3) and min(a.shape[-2:]) > 16
    colour = [k for k, a in arrays.items() if is_rgb(a)]
    depth = [k for k, a in arrays.items() if is_depth(a)]
    other = [k for k, a in arrays.items() if k not in colour + depth and a.dtype in (np.uint8, np.int8) and a.ndim >= 3]
    if other or (depth and not colour) or len(colour) > 1 and depth:
        raise BlurUnavailable(f'{NO_BLUR}: this file holds images in a form that cannot be blurred')

    def work(blur):
        changed = False
        for key in colour:
            stack = arrays[key] if arrays[key].ndim == 4 else arrays[key][None]
            found = [blur.find(frame) for frame in stack]
            if not any(len(b) for b in found):
                continue
            changed = True
            out = np.stack([blur.cover(frame, b) for frame, b in zip(stack, found)])
            arrays[key] = out if arrays[key].ndim == 4 else out[0]
            for name in depth:
                d = arrays[name] if arrays[name].ndim == 3 else arrays[name][None]
                if len(d) != len(stack):
                    raise BlurUnavailable(f'{NO_BLUR}: depth and colour frames do not match')
                done = np.stack([hide_depth(x, b, stack.shape[1:]) for x, b in zip(d, found)])
                arrays[name] = done if arrays[name].ndim == 3 else done[0]
        return Cleared(_write_npz(arrays)) if changed else Cleared(raw)
    return _guarded(work)


def clear_recording(folder, out, wait=lambda: None):
    """A recording folder rewritten for upload into `out`: <cam>.mjpg, <cam>.depth and frames.jsonl, whose byte offsets follow
    the new files. Frames with no face keep their exact bytes. Only what frames.jsonl lists goes out. Returns the names written."""
    import cv2
    folder, out = Path(folder), Path(out)
    rows, sinks, sources, blurs = [], {}, {}, {}

    def sink(name):
        if name not in sinks:
            sinks[name] = open(out / name, 'wb')
            sources[name] = open(folder / name, 'rb')
        return sinks[name], sources[name]

    def copy(name, span, change=None):
        to, frm = sink(name)
        frm.seek(span[0])
        data = frm.read(span[1])
        if len(data) != span[1]:
            raise BlurUnavailable(f'{NO_BLUR}: {name} is shorter than its index')
        data = change(data) if change else data
        at = to.tell()
        to.write(data)
        return [at, len(data)]

    try:
        for n, line in enumerate((folder / 'frames.jsonl').read_text().splitlines()):
            try:
                row = json.loads(line)
            except ValueError:
                continue                                      # a line cut off by a crash: its frame is not sent
            cam = row.get('cam')
            if cam and row.get('jpg'):
                if n % 30 == 0:
                    wait()
                data = (sources.get(f'{cam}.mjpg') or sink(f'{cam}.mjpg')[1])
                data.seek(row['jpg'][0])
                rgb = decode_jpeg(data.read(row['jpg'][1]))
                blur = blurs.setdefault(cam, FaceBlur())
                hide = blur.track(rgb)[0]
                row['jpg'] = copy(f'{cam}.mjpg', row['jpg'], (lambda _: encode_jpeg(blur.cover(rgb, hide))) if hide else None)
                if row.get('png'):
                    def clean(png):
                        depth = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
                        ok, again = cv2.imencode('.png', hide_depth(depth, hide, rgb.shape))
                        if not ok:
                            raise BlurUnavailable(f'{NO_BLUR}: a depth image could not be written')
                        return again.tobytes()
                    row['png'] = copy(f'{cam}.depth', row['png'], clean if hide else None)
            elif row.get('jpg') or row.get('png'):
                raise BlurUnavailable(f'{NO_BLUR}: a recording frame names no camera')
            rows.append(json.dumps(row))
    except BlurUnavailable:
        raise
    except Exception as exc:
        raise BlurUnavailable(f'{NO_BLUR} ({type(exc).__name__})') from exc
    finally:
        for f in [*sinks.values(), *sources.values()]:
            f.close()
    (out / 'frames.jsonl').write_text(''.join(r + '\n' for r in rows))
    return {*sinks, 'frames.jsonl'}


def video_hits(path, wait=lambda: None):
    """(per frame: the boxes to hide, frame shape, fps) of a video, tracked as one camera (HOLD_FRAMES), as clear_video hides
    them. Raises BlurUnavailable when the video cannot be read."""
    import cv2
    source = cv2.VideoCapture(str(path))
    if not source.isOpened():
        raise BlurUnavailable(f'{NO_BLUR}: a video could not be read')
    blur, hits, shape = FaceBlur(), [], None
    try:
        fps = source.get(cv2.CAP_PROP_FPS) or 24.
        while True:
            ok, bgr = source.read()
            if not ok:
                break
            if len(hits) % 30 == 0:
                wait()
            shape = bgr.shape
            hits.append(blur.track(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))[0])
    finally:
        source.release()
    if not hits:
        raise BlurUnavailable(f'{NO_BLUR}: a video could not be read')
    return hits, shape, fps


def _h264_writer(out, fps, shape):
    """A PyAV H.264 (yuv420p) writer when PyAV with libx264 is installed (the dataset extra), else None. Frame i is at i/fps,
    so a LeRobot dataset's timestamps still match the re-encoded video."""
    try:
        import fractions
        import av
        av.codec.Codec('libx264', 'w')
    except Exception:
        return None
    h, w = shape[0] - shape[0] % 2, shape[1] - shape[1] % 2
    if (h, w) != tuple(shape[:2]):
        return None
    rate = fractions.Fraction(fps).limit_denominator(1001)
    box = av.open(str(out), 'w', format='mp4')
    stream = box.add_stream('libx264', rate=rate)
    stream.width, stream.height, stream.pix_fmt = w, h, 'yuv420p'
    stream.codec_context.time_base = 1 / rate
    stream.codec_context.gop_size = 2
    stream.options = {'crf': '23'}
    state = dict(n=0)

    def write(bgr):
        frame = av.VideoFrame.from_ndarray(np.ascontiguousarray(bgr), format='bgr24').reformat(format='yuv420p')
        frame.pts = state['n']; state['n'] += 1
        for packet in stream.encode(frame):
            box.mux(packet)

    def close():
        for packet in stream.encode():
            box.mux(packet)
        box.close()
    return write, close


def clear_video(path, out, wait=lambda: None):
    """A video file ready to send: `path` itself if no face is in it, else `out`, re-encoded with the faces hidden: H.264 when
    PyAV is installed (the dataset extra), else mp4v."""
    import cv2
    hits, shape, fps = video_hits(path, wait)
    if not any(len(h) for h in hits):
        return Path(path)
    source = cv2.VideoCapture(str(path))
    if not source.isOpened():
        raise BlurUnavailable(f'{NO_BLUR}: a video could not be read')
    blur, writer, close = FaceBlur(), None, None
    try:
        for hide in hits:
            ok, bgr = source.read()
            if not ok:
                raise BlurUnavailable(f'{NO_BLUR}: a video could not be read twice')
            if writer is None and close is None:
                made = _h264_writer(out, fps, bgr.shape)
                if made is not None:
                    writer, close = made
                else:
                    cv = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'mp4v'), fps, (bgr.shape[1], bgr.shape[0]))
                    if not cv.isOpened():
                        raise BlurUnavailable(f'{NO_BLUR}: a video could not be written')
                    writer, close = cv.write, cv.release
            writer(blur.cover(bgr, hide))
        return Path(out)
    except BlurUnavailable:
        raise
    except Exception as exc:
        raise BlurUnavailable(f'{NO_BLUR}: a video could not be written ({type(exc).__name__})') from exc
    finally:
        source.release()
        if close is not None:
            try:
                close()
            except Exception:
                pass


def clear_depth_png(data, boxes, rgb_shape):
    """A 16-bit single-channel depth PNG ready to send, given the boxes hidden in its colour frame (of shape `rgb_shape`):
    these same bytes when there are none, else the PNG with no reading in those regions, still 16-bit and on the same scale.
    Anything that is not a 16-bit single-channel PNG cannot be cleared."""
    import cv2
    depth = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.dtype != np.uint16 or depth.ndim != 2:
        raise BlurUnavailable(f'{NO_BLUR}: a depth image is not a 16-bit single-channel PNG')
    if not len(boxes):
        return Cleared(data)
    ok, again = cv2.imencode('.png', hide_depth(depth, boxes, rgb_shape))
    if not ok:
        raise BlurUnavailable(f'{NO_BLUR}: a depth image could not be written')
    return Cleared(again.tobytes())
