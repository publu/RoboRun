"""Robot tools for your LLM: the specs to give its tool-calling API, and one function that runs a call.

    from llm_tools import TOOLS, call_tool
    # 1. pass TOOLS to your LLM as its tools (name, description, input_schema: JSON Schema;
    #    rename input_schema to `parameters` if your provider calls it that)
    # 2. for each tool call it makes:  result = call_tool(name, arguments)
    # 3. send `result` back as the tool result. A camera image comes back as
    #    {"type": "image", "media_type": "image/jpeg", "data": <base64>}: pass it on as an image block

    roborun python llm_tools.py tools                         # the specs, as JSON
    roborun python llm_tools.py call look '{"camera": "top"}'  # run one call by hand

Errors never raise out of call_tool: they come back as {"error": {"code", "message", "retryable"}} so the model can read
them and decide (wait and retry when retryable, change the request when not). Stop is always available and is never
refused; tell your model to call it on any doubt.

Camera frames are face-blurred by the robot before they reach this code, so they may be sent to a hosted LLM. `look` only
sends the top, left and right cameras and refuses any other (a humanoid's head frame, for one, is not face-blurred yet).
"""
import base64
import json
import sys

from roborun_sdk.client import Robot, RobotError

LOOK_CAMERAS = ('top', 'left', 'right')    # the only cameras `look` sends to the model: their frames are face-blurred

TOOLS = [
    {'name': 'robot_status',
     'description': 'What the robot is: its profile, whether it is ready, which arms and cameras are connected, and the task '
                    'running now, if any. Call it first.',
     'input_schema': {'type': 'object', 'properties': {}, 'additionalProperties': False}},
    {'name': 'look',
     'description': 'See the table now: the newest image from a camera (top looks down on the whole table; left and right are '
                    'on the wrists). Faces are blurred.',
     'input_schema': {'type': 'object', 'properties': {'camera': {'type': 'string', 'enum': list(LOOK_CAMERAS), 'default': 'top'}},
                      'additionalProperties': False}},
    {'name': 'arm_state',
     'description': 'Where each arm is: joint angles (rad), gripper opening (0 closed .. 1 open), tool position (mm, robot frame). '
                    'The gripper opening does not tell whether an object is held (a jaw around a wide cup reads nearly open): '
                    'use the outcome that run_task returns.',
     'input_schema': {'type': 'object', 'properties': {}, 'additionalProperties': False}},
    {'name': 'run_task',
     'description': 'Have the robot do one physical task described in plain words, such as "pick up the cup" or "put the cup '
                    'back where it was", and wait until it ends. Returns status (done, stopped, failed), the number of moves, '
                    'a message and outcome: outcome.holding.left / .right is what that arm holds now ({object, lifted_mm, '
                    'evidence, verified}), null when the robot measured it empty, or "unknown" when it did not check that arm; '
                    'outcome.placed lists what the task let go of. When an arm you need is "unknown", use look before relying '
                    'on it. mode "preview" plans and shows the motion without moving anything (outcome is null).',
     'input_schema': {'type': 'object', 'required': ['prompt'], 'additionalProperties': False, 'properties': {
         'prompt': {'type': 'string', 'minLength': 1, 'maxLength': 1500},
         'mode': {'type': 'string', 'enum': ['run', 'preview'], 'default': 'run'},
         'arm': {'type': 'string', 'enum': ['left', 'right', 'both'], 'description': 'leave out to let the robot choose'},
         'timeout_s': {'type': 'number', 'default': 300, 'description': 'stop the task if it runs longer than this'}}}},
    {'name': 'stop',
     'description': 'Stop all motion now. Always available, never refused. Use it whenever something looks wrong.',
     'input_schema': {'type': 'object', 'properties': {}, 'additionalProperties': False}},
]


def _task_result(task):
    keep = ('id', 'status', 'moves', 'message', 'error', 'error_info', 'elapsed_s')
    out = {k: task.get(k) for k in keep if task.get(k) is not None}
    out['outcome'] = task.get('outcome')        # kept when null: "nothing to report" is an answer the model can read
    return out


def call_tool(name, arguments=None, robot=None):
    """Run one tool call: a JSON-serialisable result, an image block for `look`, or {"error": {...}}."""
    robot = robot or Robot()
    args = dict(arguments or {})
    try:
        if name == 'robot_status':
            return robot.status()
        if name == 'look':
            camera = args.get('camera', 'top')
            if camera not in LOOK_CAMERAS:      # the schema's enum is advice to the model, not a check: enforce it here
                return {'error': {'code': 'invalid', 'retryable': False,
                                  'message': f'camera must be one of {", ".join(LOOK_CAMERAS)}, not {camera!r}'}}
            jpeg = robot.frame(camera)
            return {'type': 'image', 'media_type': 'image/jpeg', 'data': base64.b64encode(jpeg).decode()}
        if name == 'arm_state':
            state = robot.state()
            return {'arms': {side: {k: arm.get(k) for k in ('ok', 'joints_rad', 'gripper', 'tool_mm')}
                             for side, arm in (state.get('arms') or {}).items()}}
        if name == 'run_task':
            task = robot.run(args['prompt'], mode=args.get('mode', 'run'), arm=args.get('arm'),
                             timeout=float(args.get('timeout_s', 300)))
            return _task_result(task)
        if name == 'stop':
            return robot.stop()
        return {'error': {'code': 'unknown_tool', 'message': f'no tool named {name!r}', 'retryable': False}}
    except TimeoutError as exc:
        return {'error': {'code': 'timeout', 'message': f'{exc}; the task was stopped', 'retryable': True}}
    except KeyError as exc:
        return {'error': {'code': 'invalid', 'message': f'missing argument {exc}', 'retryable': False}}
    except RobotError as exc:
        return {'error': {'code': exc.code, 'message': exc.message, 'retryable': exc.retryable}}


def main(argv):
    if argv[:1] == ['tools']:
        print(json.dumps(TOOLS, indent=1))
        return 0
    if argv[:1] == ['call'] and len(argv) >= 2:
        result = call_tool(argv[1], json.loads(argv[2]) if len(argv) > 2 else {})
        if isinstance(result, dict) and result.get('type') == 'image':
            result = dict(result, data=f"<{len(result['data'])} base64 characters>")
        print(json.dumps(result, indent=1))
        return 1 if isinstance(result, dict) and 'error' in result else 0
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
