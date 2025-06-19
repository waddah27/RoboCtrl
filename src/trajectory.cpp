#include "trajectory.hh"
#include <cmath>
#include <iostream>

// TrajectoryPlanner::~TrajectoryPlanner(){
//     std::cout<<"Destroy the TP object\n"<<std::endl;
//     // delete trajectory; // only if it was a simple pointer not smart 
// }

// TrajectoryState constructor
TrajectoryState::TrajectoryState(double xt, double xd, double vmax, double step)
    : x_t(xt), x_d(xd), v_max(vmax), step(step) {}

// TrajectoryPlanner constructors
TrajectoryPlanner::TrajectoryPlanner()
    : trajectory(std::make_unique<TrajectoryState>()) {}

TrajectoryPlanner::TrajectoryPlanner(TrajectoryState Traj)
    : trajectory(std::make_unique<TrajectoryState>(Traj.x_t, Traj.x_d, Traj.v_max, Traj.step)) {}

TrajectoryPlanner::~TrajectoryPlanner() = default;

void TrajectoryPlanner::plan_trajectory(double target_pos, double max_vel, double step) {
    trajectory->step = step;
    trajectory->x_d = target_pos;
    trajectory->v_max = max_vel;
    std::cout << "Trajectory planned to " << target_pos 
              << " with max velocity " << max_vel << std::endl;
}

double TrajectoryPlanner::get_next_setpoint() {
    // Simple linear trajectory for demonstration
    if (trajectory->x_t < trajectory->x_d - trajectory->step) {
        trajectory->x_t += trajectory->step;
    } else if (trajectory->x_t > trajectory->x_d + trajectory->step) {
        trajectory->x_t -= trajectory->step;
    }
    return trajectory->x_t;
}


void TrajectoryPlanner::print_summary(){
    std::cout <<"planner.trajectory->x_d = "<<trajectory->x_d<<std::endl;
    std::cout <<"planner.trajectory->x_t = "<<trajectory->x_t<<std::endl;
    std::cout <<"planner.trajectory->v_max = "<<trajectory->v_max<<std::endl;
}