from roboctrl.motion_control import initialize, move_to, get_position

initialize()
move_to(10.0, 2.0)
print("Position:", get_position())