"""Rules: a keep-out box. The gripper may not enter a box on the table (for example, where a person or a fixture is).

    rules: /path/to/keep_out_box.py        # in robot.yaml; applies to every arm built on StreamedArm

check() runs for every arm command. state['tool_position_m'] is where the gripper is now (metres, the arm's base frame),
from the arm's model; it is None for an arm without one, and this rule then lets commands through. Refusing makes the arm
brake and hold where it is; Roborun ends the task with "your rules file refused this motion" if it lasts. To leave the box,
take the arm out by hand or with Home after editing the rule: a rule that refuses every command keeps the arm still.

    python keep_out_box.py      # calls check() the way an arm does, with two measured gripper positions
"""

BOX_MIN_M = (0.30, -0.10, -0.05)       # x, y, z: the box's near corner, arm base frame
BOX_MAX_M = (0.50, 0.10, 0.30)


def inside(point):
    return all(lo <= v <= hi for v, lo, hi in zip(point, BOX_MIN_M, BOX_MAX_M))


def check(side, positions_rad, gripper, state):
    tool = state.get('tool_position_m')
    if tool is not None and inside(tool):
        return False
    return None


if __name__ == '__main__':
    q = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]
    print('gripper at (0.40, 0.00, 0.10):', check('left', q, 1.0, dict(tool_position_m=[0.40, 0.00, 0.10])))
    print('gripper at (0.20, 0.25, 0.10):', check('left', q, 1.0, dict(tool_position_m=[0.20, 0.25, 0.10])))
