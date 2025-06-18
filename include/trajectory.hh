
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include<iostream>

struct TrajectoryState
{
    double x_t; // current position
    double x_d; // target position
    double v_max; // maximum velocity

    // Initialization of the declared parameters 
    TrajectoryState(double xt = 0.0, double xd = 0.0, double vmax = 0.0):
        x_t(xt), x_d(xd), v_max(vmax){}  
};


class TrajectoryPlanner {
public:
    // Constructors
    TrajectoryPlanner();  // Default
    TrajectoryPlanner(double x_t, double x_d, double v_max);  // Parameterized
    
    // Destructor
    ~TrajectoryPlanner();

    void plan_trajectory(double target_position, double max_velocity);
    double get_next_setpoint();
    void print_summary();


      
    
private:
    struct TrajectoryState * trajectory;
    
};

#endif // TRAJECTORY_H