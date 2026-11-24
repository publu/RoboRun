"""Run a DreamLabs policy (Dream-1) on your arms through Roborun, from Python.

    pip install 'roborun-sdk[dreamlabs]'          # DreamLabs' client (dreamlabs-client) and the face blur
    # set up your DreamLabs credentials as DreamLabs documents them (DREAM_API_KEY, or ~/.dreamlabs/credentials)
    roborun up                                    # two YAM arms, cameras top, left and right
    roborun python dreamlabs_policy.py check      # offline: does the model fit this robot? nothing is sent
    roborun python dreamlabs_policy.py shadow     # asks the model, moves nothing, reports what it would do
    roborun python dreamlabs_policy.py run "fold the towel"

DreamLabs' model chooses the motion; Roborun plays it through its own limits (the per-tick clip, the seam and age checks,
the joint and tool speed limits, your rules file), holds the arms when an answer is late, and Stop (the page, the viewer,
`roborun stop`) ends the run at once. The run is a Roborun task: its record, its recording and a dataset episode are not sent
to DreamLabs; like any other task's, they go to your Roborun history (roborun up --no-data-sync keeps them on this computer).
The request timing is kept in log/policy/<id>/ and never leaves this computer.

What is sent to DreamLabs while a run is active: the prompt, both arms' joint angles, grippers and tool poses (computed from
the joint angles), and recent camera frames (face-blurred), with your DreamLabs key. Nothing is sent by `check`.
"""
import sys

from roborun_sdk import policy

MODEL = 'dreamlabs/dream-1-beta'


def main(argv):
    what = argv[0] if argv else 'check'
    if what == 'check':
        print(policy.check('dreamlabs', model=MODEL))
        return 0
    # consent=True: you agree, for this run only, to send what the docstring above lists to DreamLabs. Without it (and
    # without an answer stored by `roborun policy run` in a terminal) the run is refused and nothing is sent.
    if what == 'shadow':
        report = policy.run('dreamlabs', model=MODEL, prompt=' '.join(argv[1:]) or 'fold the towel', shadow=True,
                            seconds=20, consent=True)
        print(report)
        return 0
    result = policy.run('dreamlabs', model=MODEL, prompt=' '.join(argv[1:]) or 'fold the towel', arms='both', seconds=60,
                        consent=True)
    print(result.task_id, result.status, result.stop_reason, result.message)
    return 0 if result.status in ('done', 'stopped') else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
