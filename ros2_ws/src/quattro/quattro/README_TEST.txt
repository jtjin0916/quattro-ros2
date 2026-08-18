# Quattro input/state-machine update v2

This version removes the feedback path that could overwrite keyboard state.

Final command flow:

keyboard_teleop.py --------\
keyboard_teleop_gim.py -----+--> /quattro/cmd_raw --> quattro_sm.py --> /quattro/cmd --> quattro_commander
teleop_node.py (joystick) --/

Key rule:
- Input nodes NEVER subscribe to /quattro/cmd.
- /quattro/cmd_raw is operator/raw input only.
- /quattro/cmd is state-machine output only.
- Run one operator input node at a time (keyboard OR joystick).
- keyboard/joystick publish /quattro/cmd_raw continuously at 20 Hz.
- quattro_sm republishes /quattro/cmd at 100 Hz.
- if raw input disappears for 1.0 s, quattro_sm outputs a safe Stop.

Test:
  ros2 run quattro quattro_sm
  ros2 run quattro keyboard_teleop

  ros2 topic info /quattro/cmd_raw -v
  ros2 topic info /quattro/cmd -v
  ros2 topic hz /quattro/cmd_raw
  ros2 topic hz /quattro/cmd

Expected:
  /quattro/cmd_raw:
    publisher = keyboard_teleop OR quattro_teleop
    subscriber = quattro_sm

  /quattro/cmd:
    publisher = quattro_sm
    subscriber = quattro_commander
    keyboard_teleop should NOT appear as a subscriber

  raw ~= 20 Hz
  processed ~= 100 Hz
