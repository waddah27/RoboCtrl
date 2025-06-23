#include "trajectory.hh"
#include <cmath>
#include <iostream>

// TrajectoryState constructor
TrajectoryState::TrajectoryState(double xt, double xd, double vmax, double step)
    : x_t(xt), x_d(xd), v_max(vmax), step(step) {}

// TrajectoryPlanner constructors
LinearStepPlanner::LinearStepPlanner()
    : trajectory(std::make_unique<TrajectoryState>()) {}

LinearStepPlanner::LinearStepPlanner(TrajectoryState Traj)
    : trajectory(std::make_unique<TrajectoryState>(Traj.x_t, Traj.x_d, Traj.v_max, Traj.step)) {}

LinearStepPlanner::~LinearStepPlanner() = default;

void LinearStepPlanner::plan_trajectory(double target_pos, double max_vel, double step) {
    trajectory->step = step;
    trajectory->x_d = target_pos;
    trajectory->v_max = max_vel;
    std::cout << "Trajectory planned to " << target_pos 
              << " with max velocity " << max_vel << std::endl;
}

double LinearStepPlanner::get_next_setpoint() {
    // Simple linear trajectory for demonstration
    if (trajectory->x_t < trajectory->x_d - trajectory->step) {
        trajectory->x_t += trajectory->step;
    } else if (trajectory->x_t > trajectory->x_d + trajectory->step) {
        trajectory->x_t -= trajectory->step;
    }
    return trajectory->x_t;
}


void LinearStepPlanner::print_summary(){
    std::cout <<"planner.trajectory->x_d = "<<trajectory->x_d<<std::endl;
    std::cout <<"planner.trajectory->x_t = "<<trajectory->x_t<<std::endl;
    std::cout <<"planner.trajectory->v_max = "<<trajectory->v_max<<std::endl;
}


// TrajectoryTrapezoidalVelocityProfilePlanner constructors
TrapezoidalPlanner::TrapezoidalPlanner(){}
TrapezoidalPlanner::TrapezoidalPlanner(double x_0, double x_d, double v_max, double a_max, double dt, bool triangle)
: x_0(x_0), x_d(x_d), v_max(v_max), a_max(a_max), dt(dt), triangle(triangle) {
    dtg = x_d - x_0;
    t_acc = v_max/a_max;
    t_dec = v_max/a_max;
    dtg_acc = 0.5 * a_max * t_acc * t_acc;
    dtg_dec = 0.5 * a_max * t_dec * t_dec;
    double dtg_abs = std::abs(dtg);
    t_cr = triangle ? 0.0 : (dtg_abs - dtg_acc - dtg_dec) / v_max;
    t_total = t_acc + t_dec + t_cr;


}

//TrajectoryTrapezoidalVelocityProfilePlanner destructor 
TrapezoidalPlanner::~TrapezoidalPlanner() = default;

bool TrapezoidalPlanner::is_done() const {
    return done;
}

double TrapezoidalPlanner::update(){
    double dir = dtg > 0 ? 1:-1;
    double t1 = t_acc, t2 = t1+t_cr, t3 = t_total;
    
    if (t<=t_acc){
        x_t = x_0 + dir * 0.5 * a_max * t*t;
        
    }
    else if(t<=t_acc+t_cr){
        // double t1 = t-t_acc;
        x_t = x_0 + dir * (dtg_acc + v_max*(t-t1));
    }
    else if(t<t_total){
        // double t2 = t-(t_acc+t_cr);
        x_t = x_0 + dir *(dtg_acc + v_max*(t2-t1) + 0.5*a_max*(t-t2)*(t-t2));

    }
    else{
        // std::cout<<"DONE!"<<std::endl;
        done = true;
        x_t = x_d;
        
    }

    t+=dt;
    return x_t;
}

