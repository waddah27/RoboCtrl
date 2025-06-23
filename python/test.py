from roboctrl import trajectory

planner = trajectory.LinearStepPlanner()
planner.plan_trajectory(10.0, 2.0, 0.1)
print(planner.get_next_setpoint())
print(trajectory.__dir__())