"""Setup rewrites only the cameras it finds itself; a camera you wrote (your own class, a stream address) stays."""
from roborun_sdk.guided import _kept_camera


def test_your_own_camera_and_a_stream_stay():
    assert _kept_camera('my_cam.py:MyCamera') == 'my_cam.py:MyCamera'
    assert _kept_camera(dict(source='rtsp://cam.local/live', fps=15)) == dict(source='rtsp://cam.local/live', fps=15)


def test_a_camera_setup_finds_itself_is_replaced_by_what_it_finds_now():
    assert _kept_camera('usb:3') is None and _kept_camera(dict(source='realsense:123456789012')) is None
