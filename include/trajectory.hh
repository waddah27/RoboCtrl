
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include<iostream>
#include <memory>

struct TrajectoryState
{
    double x_t; // current position
    double x_d; // target position
    double v_max; // maximum velocity
    double step;

    // Initialization of the declared parameters 
    TrajectoryState(double xt = 0.0, double xd = 0.0, double vmax = 0.0, double step = 0.0);
          
};


class TrajectoryPlanner {
public:
    // Constructors
    TrajectoryPlanner();  // Default
    TrajectoryPlanner(struct TrajectoryState trajectory);  // Parameterized
    
    // Destructor
    ~TrajectoryPlanner();

    void plan_trajectory(double target_position, double max_velocity, double step);
    double get_next_setpoint();
    void print_summary();


      
    
private:
    // struct TrajectoryState * trajectory;
    std::unique_ptr<struct TrajectoryState> trajectory;  // Smart pointer instead of raw pointer
};

#endif // TRAJECTORY_H