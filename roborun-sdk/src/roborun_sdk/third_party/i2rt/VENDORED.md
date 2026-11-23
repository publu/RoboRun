# i2rt (vendored)

Source: https://github.com/i2rt-robotics/i2rt, commit 234d895 (MIT, see LICENSE).

Contents: the modules `i2rt.robots.get_robot` imports, `robots/config`, and the MJCF models and meshes of the
yam, yam_pro, yam_ultra (v1, v2) and big_yam arms and of the grippers. URDFs, READMEs, tests and everything else are left out.
Imported as `i2rt` (roborun_sdk/arms/yam.py puts this folder's parent on sys.path).

## Edits (each is marked `ROBORUN EDIT n` in the code)

1. `motor_drivers/can_interface.py`: `BUS_OPENER` hook; when set, `open_can_bus` gets every bus from it, including the
   buses of the startup checks. Roborun sets it to `roborun_sdk.arms.canbus.open_bus`.
2. `motor_drivers/motor_check.py`: with `BUS_OPENER` set, the startup checks accept any channel (they used to refuse a
   channel without "can" in its name). The checks themselves are unchanged and always run.
3. `robots/get_robot.py`, `motor_drivers/dm_driver.py`: `get_yam_robot(gravity_hold_from_first_cycle=True)` loads the dynamics model,
   reads every motor's position while it is still disabled (DM status refresh 0xCC, falling back to the reply to
   enable), and sends each motor its gravity torque and idle damping right behind the enable frame (`motor_on_holding`, `can_interface.py` `follow`),
   so the arm is not left unheld while this process waits for a reply or the rest of the chain and `MotorChainRobot` start. Off by default.
4. `motor_drivers/dm_driver.py`, `robots/motor_chain_robot.py`: the chain and server threads are daemon threads, so an
   exiting process cannot hang on them.

Roborun's edits in this folder are also under the MIT License in `LICENSE` here.
