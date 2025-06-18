from motion_control import initialize, move_to, get_position

# Initialize the control system
initialize()

# Send a motion command
move_to(10.0, 2.0)  # Move to position 10 at max velocity 2

# Get current position
print("Current position:", get_position())