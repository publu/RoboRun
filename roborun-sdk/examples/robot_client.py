"""Drive the robot on this computer from the command line, with roborun_sdk.client.Robot (the typed client for the local API).

    roborun python robot_client.py status
    roborun python robot_client.py run "pick up the cup" [--preview] [--stop-after 5]
    roborun python robot_client.py stop
    roborun python robot_client.py frame top top.jpg          # the camera now, faces blurred

In your own code:

    from roborun_sdk.client import Robot, RobotError
    robot = Robot()                                   # http://127.0.0.1:8120, this computer only
    task = robot.run('pick up the cup')               # start it and wait until it ends; mode='preview' moves nothing
    print(task['status'], task['message'])
    robot.stop()                                      # all motion, like the app's Stop

`roborun python` runs this file with Roborun's own Python, where roborun_sdk is installed.
"""
import json
import sys
import time

from roborun_sdk.client import Robot, RobotError, RobotUnavailable


def main(argv, robot=None):
    sys.stdout.reconfigure(line_buffering=True)
    robot = robot or Robot()
    try:
        if argv[:1] == ['status']:
            print(json.dumps(robot.status(), indent=1))
            return 0
        if argv[:1] == ['stop']:
            print(json.dumps(robot.stop()))
            print(json.dumps(robot.stop_status()))
            return 0
        if argv[:1] == ['frame'] and len(argv) >= 3:
            data = robot.frame(argv[1])
            with open(argv[2], 'wb') as out:
                out.write(data)
            print(f'{argv[1]}: {len(data)} bytes -> {argv[2]}')
            return 0
        if argv[:1] == ['run'] and len(argv) >= 2:
            stop_after = float(argv[argv.index('--stop-after') + 1]) if '--stop-after' in argv else None
            task = robot.start(argv[1], mode='preview' if '--preview' in argv else 'run')
            print('started', task['id'], task['status'])
            started, last = time.monotonic(), None
            for view in robot.live(task['id']):
                if (view['status'], view.get('message')) != last:
                    last = (view['status'], view.get('message'))
                    print(f"{view.get('elapsed_s') or 0:6.1f}s {view['status']:8s} {view.get('message') or ''}")
                if stop_after is not None and time.monotonic() - started > stop_after and view['status'] == 'running':
                    print('stopping:', robot.stop_task(task['id'])['status'])
                    stop_after = None
            final = robot.task(task['id'])
            print(json.dumps({k: final.get(k) for k in ('id', 'status', 'moves', 'message', 'error')}))
            return 0 if final['status'] in ('done', 'stopped') else 1
    except RobotUnavailable as exc:
        print(exc.message, file=sys.stderr)
        return 3
    except RobotError as exc:
        print(f'{exc.status} {exc.code}: {exc.message}' + (' (try again)' if exc.retryable else ''), file=sys.stderr)
        return 2
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
