#include "control.h"
#include "trajectory.hh"
#include <iostream>


TrajectoryState trajectory(X0, XD, vMax, STEP);

static TrajectoryLinearStepPlanner planner(trajectory);
static TrajectoryTrapezoidalVelocityProfilePlanner plannerTrapz(X0, XD, vMax, aMax, STEP);

void initialize_control_system() {
    std::cout << "TODO -- Control system initialize" << std::endl;
}

void send_motion_command(double position, double velocity, double step) {
    std::cout << "Received command - Position: " << position 
              << ", Velocity: " << velocity << std::endl;
    planner.plan_trajectory(position, velocity, step);
    std::cout<<"trapezoidal planner:\n x_t = "<<plannerTrapz.update()<<std::endl;
    planner.print_summary();
}

double get_current_position() {
    return planner.get_next_setpoint();
}
double get_current_pos_trapz(){
    return plannerTrapz.update();
}