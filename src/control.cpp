#include "control.h"
#include "trajectory.hh"
#include <iostream>


TrajectoryState trajectory(X0, XD, vMax, STEP);

static LinearStepPlanner planner(trajectory);
static TrapezoidalPlanner plannerTrapz(X0, XD, vMax, aMax, STEP, false);

void initialize_control_system() {
    std::cout << "TODO -- Control system initialize" << std::endl;
}

void send_motion_command(double position, double velocity, double step) {
    std::cout << "Received command - Position: " << position 
              << ", Velocity: " << velocity << std::endl;
    planner.plan_trajectory(position, velocity, step);
    std::cout<<"plannerTrapz.x_t = "<<plannerTrapz.update()<<std::endl;
    planner.print_summary();
}

double get_current_position() {
    return planner.get_next_setpoint();
}
double get_current_pos_trapz(bool & is_done){
    if (plannerTrapz.is_done()){
    is_done = true;
    std::cout<<"plannerTrapz is done! "<<std::endl;
    }
    return plannerTrapz.update();
    
}