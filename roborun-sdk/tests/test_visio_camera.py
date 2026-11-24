"""The Visio stereo camera: wire decoding, and RGB + depth through CustomSource from a real calibrated fisheye pair.

data/visio/tumvi_calib_3pairs.mcap: three stereo pairs of the TUM VI calibration sequence (512x512 equidistant fisheye,
kalibr calibration, an AprilGrid of 88 mm tags at 0.3 spacing), packed as a Visio recording by visio_pack.py. Images and
calibration are TUM VI's own (CC BY 4.0). A recording of a real Visio device, when one is on this computer, is replayed too.
"""
import os
import time
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')
pytest.importorskip('av')
pytest.importorskip('mcap')
pytest.importorskip('google.protobuf')

from roborun_sdk import visio_camera as vc  # noqa: E402
from roborun_sdk.cameras import CustomSource  # noqa: E402

DATA = Path(__file__).parent / 'data' / 'visio'
TUMVI = DATA / 'tumvi_calib_3pairs.mcap'
DEVICE_RECORDING = Path(os.environ.get('VISIO_RECORDING', Path.home() / 'datasets/visio-device/ego_15s.mcap'))


def frames(n, **extra):
    settings = {k: extra.pop(k) for k in ('width', 'height') if k in extra}
    src = CustomSource('roborun_sdk.visio_camera:VisioCamera', 'top', settings=settings, extra=extra)
    model = src.open()
    items, deadline = [], time.monotonic() + 30
    while len(items) < n and time.monotonic() < deadline:
        item = src.read()
        if item is None:
            time.sleep(.005)
        else:
            items.append(item)
    src.close()
    return model, items, src.camera


# -- the wire ---------------------------------------------------------------------------------------------------------
def test_crc_is_ccitt_false():
    assert vc.crc16(b'123456789') == 0x29B1


def test_a_frame_decodes_and_a_damaged_one_is_dropped():
    header = bytes([0x08, 23, 0x10, 7])                       # stream_id 23, seq 7
    core = bytes([len(header)]) + header + b'\x00payload\x00'
    core += vc.crc16(core).to_bytes(2, 'little')
    encoded, block = bytearray(), bytearray()
    for b in core + b'\x00':                                  # COBS-encode, the way the device frames it
        if b == 0:
            encoded += bytes([len(block) + 1]) + block
            block = bytearray()
        else:
            block.append(b)
    assert vc.unframe(bytes(encoded)) == (23, b'\x00payload\x00')
    damaged = bytearray(encoded)
    damaged[5] ^= 0x40
    assert vc.unframe(bytes(damaged)) is None


def test_device_info_names_each_stream_and_its_schema():
    def field(number, payload):
        return bytes([number << 3 | 2, len(payload)]) + payload
    channel = bytes([0x08, 19]) + field(2, b'/ego/camera/1') + field(4, b'foxglove.CompressedVideo') + field(5, b'FDS')
    name, channels = vc.device_info(field(1, b'GILABS-test') + field(2, channel))
    assert name == 'GILABS-test' and channels == {19: ('/ego/camera/1', 'foxglove.CompressedVideo', b'FDS')}


# -- depth from a real calibrated fisheye pair ------------------------------------------------------------------------
def test_tumvi_fisheye_pair_gives_metric_depth_through_custom_source():
    model, items, cam = frames(3, mcap=str(TUMVI), width=512, height=512, pair=[0, 1], min_depth_m=0.3)
    k = model['intrinsics']
    assert model['depth_available'] and model['depth_scale_m'] == 0.001 and k['distortion_model'] == 'none'
    assert (k['width'], k['height']) == (512, 512) and k['fx'] == pytest.approx(190.7, abs=0.5)
    assert cam.rectifier.baseline_m == pytest.approx(0.10109, abs=2e-4)           # kalibr: 101.06 mm, slightly off-axis
    assert len(items) == 3 and all(np.diff([i['acquired_at_s'] for i in items]) > 0)
    item = items[-1]
    assert item['rgb'].shape == (512, 512, 3) and item['depth_z16'].dtype == np.uint16
    assert (item['depth_z16'] > 0).mean() > 0.4

    # Scale: triangulate AprilGrid tag centres from the rectified pair; neighbours are 88 mm x 1.3 = 114.4 mm apart.
    params = cv2.aruco.DetectorParameters()
    params.markerBorderBits = 2                                                    # kalibr AprilGrid tags
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
    # Every recorded pair (the replay loops), not whichever one happened to be newest.
    probe = vc.VisioCamera(mcap=str(TUMVI), width=512, height=512, pair=[0, 1], min_depth_m=0.3)
    probe.open()
    pairs = probe._fresh_pairs(3)
    probe.close()
    r = probe.rectifier
    spacing, rows, depth_checks = [], [], 0
    for raw_left, raw_right in pairs:
        left, right = r.rectify(raw_left, raw_right)
        found = []
        for img in (left, right):
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
            found.append({} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)})
        in_left, in_right = found
        centres = {}
        for tag, c in in_left.items():
            if tag in in_right:
                d = c[:, 0] - in_right[tag][:, 0]
                rows.append(np.abs(c[:, 1] - in_right[tag][:, 1]).mean())
                centres[tag] = np.stack([(c[:, 0] - r.cx) * r.baseline_m / d, (c[:, 1] - r.cy) * r.baseline_m / d,
                                         r.fx * r.baseline_m / d], 1).mean(0)
        spacing += [np.linalg.norm(centres[a] - centres[b]) for a in centres for b in (a + 1, a + 6)
                    if b in centres and not (b == a + 1 and a % 6 == 5)]
        # The matcher's depth at a tag agrees with the tag's own triangulated depth.
        depth = r.depth_mm(vc.sgbm(cv2.cvtColor(left, cv2.COLOR_RGB2GRAY), cv2.cvtColor(right, cv2.COLOR_RGB2GRAY),
                                   r.max_disparity(0.3)))
        for tag, centre in centres.items():
            x, y = in_left[tag].mean(0).astype(int)
            patch = depth[y - 2:y + 3, x - 2:x + 3]
            if (patch > 0).sum() > 5:
                assert np.median(patch[patch > 0]) == pytest.approx(centre[2] * 1000, rel=0.05)
                depth_checks += 1
    assert len(pairs) == 3 and len(spacing) >= 3 and np.median(rows) < 0.75 and depth_checks >= 3
    assert np.median(spacing) == pytest.approx(0.1144, rel=0.02)

def test_a_pair_in_the_wrong_order_is_refused_naming_the_fix():
    with pytest.raises(ValueError, match=r'set pair: \[0, 1\]'):
        frames(1, mcap=str(TUMVI), width=512, height=512, pair=[1, 0])


def test_a_reopen_after_a_failure_starts_clean(tmp_path):
    """The broker retries a failed camera with close() + open() on the same object; an old failure must not stick."""
    recording = tmp_path / 'run.mcap'
    recording.write_bytes(TUMVI.read_bytes()[:20000])         # cut short: the calibration is there, the video is not
    cam = vc.VisioCamera(mcap=str(recording), width=512, height=512, pair=[0, 1])
    vc_wait, vc.OPEN_WAIT_S = vc.OPEN_WAIT_S, 2.0
    try:
        with pytest.raises((TimeoutError, Exception)):
            cam.open()
        recording.write_bytes(TUMVI.read_bytes())             # the device is back
        model = cam.open()
    finally:
        vc.OPEN_WAIT_S = vc_wait
    assert model['depth_scale_m'] == 0.001 and cam.error is None
    deadline = time.monotonic() + 10
    item = None
    while item is None and time.monotonic() < deadline:
        item = cam.read() or time.sleep(.01)
    cam.close()
    assert item is not None and item['depth_z16'].shape == (512, 512)


def test_one_source_is_required():
    with pytest.raises(ValueError, match='exactly one of port'):
        vc.VisioCamera()
    with pytest.raises(ValueError, match='no recording'):
        vc.VisioCamera(mcap='/nonexistent.mcap')


@pytest.mark.skipif(not DEVICE_RECORDING.is_file(), reason='no Visio device recording on this computer')
def test_a_real_visio_device_recording_serves_rgb_and_depth():
    model, items, cam = frames(30, mcap=str(DEVICE_RECORDING), width=640, height=360)
    assert model['intrinsics']['fx'] > 0 and model['intrinsics']['ppx'] == 320.0
    assert 0.04 < cam.rectifier.baseline_m < 0.2
    assert len(items) == 30 and all((i['depth_z16'] > 0).mean() > 0.2 for i in items)
