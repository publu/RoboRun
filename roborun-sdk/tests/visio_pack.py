"""Pack a real calibrated stereo recording into a Visio MCAP: what a Visio device writes for one stereo pair.

The images and the calibration are the dataset's own; only the container is made here. Each eye is H.265-encoded like
the device does, with its intrinsics on /<root>/camera/<i>/intrinsics (foxglove.CameraCalibration) and the pose of
camera i in camera 0 on /<root>/camera/<i>/extrinsics (foxglove.FrameTransform).

    python visio_pack.py tumvi DATASET_DIR OUT.mcap [--frames N] [--start K]
    python visio_pack.py middlebury SCENE_DIR OUT.mcap

TUM VI (CC BY 4.0, vision.in.tum.de/data/datasets/visual-inertial-dataset): the 512x512 equidistant fisheye pair, cam0
left of cam1, calibrated with kalibr (dso/camchain.yaml). Middlebury 2014 (vision.middlebury.edu/stereo/data): a
rectified pinhole pair (im0 left, im1 right) with ground-truth disparity; packed with the LEFT eye as camera 1, Visio's
own stereo convention.
"""
import argparse
from fractions import Fraction
import sys
from pathlib import Path

import av
import cv2
import numpy as np
import yaml
from visio_schema import McapWriter, Message, make_channel
from visio_schema.foxglove.CameraCalibration_pb2 import CameraCalibration
from visio_schema.foxglove.CompressedVideo_pb2 import CompressedVideo
from visio_schema.foxglove.FrameTransform_pb2 import FrameTransform

VIDEO, CALIB, TF = 'foxglove.CompressedVideo', 'foxglove.CameraCalibration', 'foxglove.FrameTransform'


def quaternion(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def stamp(field, t_ns):
    field.seconds, field.nanos = divmod(int(t_ns), 1_000_000_000)


def encode(frames, fps=20):
    """RGB frames -> one H.265 packet per frame (the first carries VPS/SPS/PPS)."""
    h, w = frames[0].shape[:2]
    codec = av.CodecContext.create('libx265', 'w')
    codec.width, codec.height, codec.pix_fmt = w, h, 'yuv420p'
    codec.time_base = Fraction(1, fps)
    codec.options = {'x265-params': 'log-level=error:bframes=0:keyint=30:repeat-headers=1', 'crf': '12'}
    packets = []
    for i, rgb in enumerate(frames):
        frame = av.VideoFrame.from_ndarray(rgb, format='rgb24').reformat(format='yuv420p')
        frame.pts = i
        packets += [bytes(p) for p in codec.encode(frame)]
    packets += [bytes(p) for p in codec.encode(None)]
    if len(packets) != len(frames):
        raise RuntimeError(f'encoder gave {len(packets)} packets for {len(frames)} frames')
    return packets


def write(out, root, cameras, extrinsics, stamps_ns):
    """cameras: {index: (frames, width, height, model, K, D)}; extrinsics: {index: 4x4 pose of camera i in camera 0}."""
    stream = iter(range(16, 1000))
    with McapWriter(out) as w:
        t0 = stamps_ns[0]
        for index, (frames, width, height, model, K, D) in sorted(cameras.items()):
            cal = CameraCalibration(width=width, height=height, distortion_model=model, D=list(D),
                                    K=list(np.asarray(K, float).ravel()), frame_id=f'cam{index}')
            stamp(cal.timestamp, t0)
            ch = make_channel(f'/{root}/camera/{index}/intrinsics', CALIB, stream_id=next(stream))
            w.write(Message(stream_id=ch.id, payload=cal.SerializeToString(), seq=0, timestamp=cal.timestamp), ch)
        for index, T in sorted(extrinsics.items()):
            tf = FrameTransform(parent_frame_id='cam0', child_frame_id=f'cam{index}')
            stamp(tf.timestamp, t0)
            tf.translation.x, tf.translation.y, tf.translation.z = T[:3, 3]
            tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w = quaternion(T[:3, :3])
            ch = make_channel(f'/{root}/camera/{index}/extrinsics', TF, stream_id=next(stream))
            w.write(Message(stream_id=ch.id, payload=tf.SerializeToString(), seq=0, timestamp=tf.timestamp), ch)
        video = {i: (make_channel(f'/{root}/camera/{i}', VIDEO, stream_id=next(stream)), encode(c[0]))
                 for i, c in cameras.items()}
        for n, t_ns in enumerate(stamps_ns):
            for index, (ch, packets) in sorted(video.items()):
                msg = CompressedVideo(frame_id=f'cam{index}', format='h265', data=packets[n])
                stamp(msg.timestamp, t_ns)
                w.write(Message(stream_id=ch.id, payload=msg.SerializeToString(), seq=n, timestamp=msg.timestamp), ch)


def tumvi(dataset, out, frames, start):
    chain = yaml.safe_load((Path(dataset) / 'dso' / 'camchain.yaml').read_text())
    names = sorted(p.name for p in (Path(dataset) / 'mav0' / 'cam0' / 'data').iterdir())[start:start + frames]
    cameras = {}
    for index in (0, 1):
        c = chain[f'cam{index}']
        fx, fy, cx, cy = c['intrinsics']
        imgs = []
        for name in names:
            gray = cv2.imread(str(Path(dataset) / 'mav0' / f'cam{index}' / 'data' / name), cv2.IMREAD_UNCHANGED)
            gray = (gray >> 8).astype(np.uint8) if gray.dtype == np.uint16 else gray
            imgs.append(cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB))
        cameras[index] = (imgs, *c['resolution'], c['distortion_model'], [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
                          c['distortion_coeffs'])
    cam1_from_cam0 = np.array(chain['cam1']['T_cn_cnm1'])          # kalibr: p_cam1 = T @ p_cam0
    write(out, 'ego', cameras, {1: np.linalg.inv(cam1_from_cam0)}, [int(n.split('.')[0]) for n in names])


def middlebury(scene, out):
    calib = dict(line.split('=', 1) for line in (Path(scene) / 'calib.txt').read_text().splitlines() if line)
    K = {i: np.array([[float(v) for v in row.split()] for row in calib[f'cam{i}'].strip('[]').split(';')]) for i in (0, 1)}
    width, height, baseline = int(calib['width']), int(calib['height']), float(calib['baseline']) / 1000
    left, right = (cv2.cvtColor(cv2.imread(str(Path(scene) / f'im{i}.png')), cv2.COLOR_BGR2RGB) for i in (0, 1))
    # Visio indices: camera 1 = Middlebury's left (cam0), camera 0 = its right (cam1). The left eye sits `baseline` to the
    # left of the right eye: p_right = p_left - (baseline, 0, 0), so the pose of camera 1 in camera 0 is t = (-b, 0, 0).
    T = np.eye(4)
    T[0, 3] = -baseline
    cameras = {1: ([left] * 3, width, height, 'plumb_bob', K[0], [0.0] * 5),
               0: ([right] * 3, width, height, 'plumb_bob', K[1], [0.0] * 5)}
    write(out, 'ego', cameras, {1: T}, [1_000_000_000 + n * 50_000_000 for n in range(3)])


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('kind', choices=('tumvi', 'middlebury'))
    ap.add_argument('source')
    ap.add_argument('out')
    ap.add_argument('--frames', type=int, default=10)
    ap.add_argument('--start', type=int, default=0)
    a = ap.parse_args()
    if a.kind == 'tumvi':
        tumvi(a.source, a.out, a.frames, a.start)
    else:
        middlebury(a.source, a.out)
    print('wrote', a.out, Path(a.out).stat().st_size, 'bytes', file=sys.stderr)
