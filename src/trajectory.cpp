#include "trajectory.hh"
#include <cmath>
#include <iostream>

TrajectoryPlanner::TrajectoryPlanner() {
    trajectory = new TrajectoryState(0.0, 0.0, 0.0);
}

TrajectoryPlanner::TrajectoryPlanner(double x_t, double x_d, double v_max){
    trajectory->x_t = x_t;
    trajectory->x_d = x_d;
    trajectory->v_max = v_max;
}

TrajectoryPlanner::~TrajectoryPlanner(){
    std::cout<<"Destroy the TP object\n"<<std::endl;
    delete trajectory;
}

void TrajectoryPlanner::plan_trajectory(double target_pos, double max_vel) {
    trajectory->x_d = target_pos;
    trajectory->v_max = max_vel;
    std::cout << "Trajectory planned to " << target_pos 
              << " with max velocity " << max_vel << std::endl;
}

double TrajectoryPlanner::get_next_setpoint() {
    // Simple linear trajectory for demonstration
    double step = 0.1;
    if (trajectory->x_t < trajectory->x_d - step) {
        trajectory->x_t += step;
    } else if (trajectory->x_t > trajectory->x_d + step) {
        trajectory->x_t -= step;
    }
    return trajectory->x_t;
}

void TrajectoryPlanner::print_summary(){
    std::cout <<"planner.trajectory->x_d = %.2f"<<trajectory->x_d<<std::endl;
    std::cout <<"planner.trajectory->x_t = %.2f"<<trajectory->x_t<<std::endl;
    std::cout <<"planner.trajectory->v_max = %.2f"<<trajectory->v_max<<std::endl;
}