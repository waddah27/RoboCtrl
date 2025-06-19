#include "control.h"
#include "trajectory.hh"
#include <iostream>
TrajectoryState trajectory(0.0, 1.0, 0.0, 0.0);

static TrajectoryPlanner planner(trajectory);

void initialize_control_system() {
    std::cout << "TODO -- Control system initialize" << std::endl;
}

void send_motion_command(double position, double velocity, double step) {
    std::cout << "Received command - Position: " << position 
              << ", Velocity: " << velocity << std::endl;
    planner.plan_trajectory(position, velocity, step);
    planner.print_summary();
}

double get_current_position() {
    return planner.get_next_setpoint();
}