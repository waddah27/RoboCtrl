#include "control.h"
#include "trajectory.hh"
#include <iostream>

static TrajectoryPlanner planner;
static double current_position = 0.0;

void initialize_control_system() {
    std::cout << "TODO -- Control system initialize" << std::endl;
}

void send_motion_command(double position, double velocity) {
    std::cout << "Received command - Position: " << position 
              << ", Velocity: " << velocity << std::endl;
    planner.plan_trajectory(position, velocity);
    planner.print_summary();
}

double get_current_position() {
    return current_position++;
}