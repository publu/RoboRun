"""What of a motion's plan goes up with the history, and what a body job's status passes on: the request's words and the
hosted service's own fields stay on this computer."""
import json

from roborun_sdk import datasync
from roborun_sdk.body import jobs


def test_strip_plan_keeps_the_robot_side_and_only_the_goal_summary():
    plan = {'id': 'p1', 'source_measured': {'left': [0.1]}, 'state': 'done', 'requested_by': 'me',
            'prompt': 'pick up the cup', 'prompt_extra': {'a': 1}, 'scene_request': {'text': 'secret'},
            'scene_result': {'text': 'secret', 'goal': {'used': 'the cup', 'why': 'it is in view', 'raw': 'secret'}},
            'model': 'm', 'resolution': 'r', 'media_url': 'https://example.com/v', 'request_id': 'q'}
    out = json.loads(datasync.strip_plan(json.dumps(plan).encode()))
    assert out == {'id': 'p1', 'source_measured': {'left': [0.1]}, 'state': 'done', 'requested_by': 'me',
                   'scene_result': {'goal': {'used': 'the cup', 'why': 'it is in view'}}}
    assert 'secret' not in json.dumps(out)


def test_strip_plan_sends_nothing_for_what_is_not_an_object():
    assert datasync.strip_plan(b'[1, 2]') == b'{}' and datasync.strip_plan(b'not json') == b'{}'


def test_a_body_status_passes_on_only_the_deferred_parts():
    assert jobs._deferred({'status': 'running', 'deferred': ['wave']}) == ['wave']
    assert jobs._deferred({'status': 'running', 'section': {'text': 'x', 'deferred': ['wave', 'bow']}}) == ['wave', 'bow']
    assert jobs._deferred({'status': 'running', 'section': {'text': 'x'}}) is None


def test_a_body_request_carries_no_service_choices():
    frame = dict(jpeg=b'\xff\xd8\xff', width=640, height=480, eye_height_m=1.2, pitch=0.0, yaw=0.0,
                 intrinsics=dict(width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0))
    body = jobs.build_request('booster_k1', frame, 'walk to the table', duration_s=4)
    assert body['options'] == {'duration_s': 4}
    assert set(body) == {'schema', 'id', 'mode', 'prompt', 'frame', 'camera', 'robot', 'options'}
