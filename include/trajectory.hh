
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include<iostream>
#include <memory>

// some default parameters to insure common configs among all trajectory planners
#define STEP 0.05
#define vMax 10
#define aMax 100
#define XD 1.0
#define X0 0.0
struct TrajectoryState
{
    double x_t; // current position
    double x_d; // target position
    double v_max; // maximum velocity
    double step;

    // Initialization of the declared parameters 
    TrajectoryState(double xt = 0.0, double xd = 0.0, double vmax = 0.0, double step = 0.0);
          
};


/* 
Simplest trajectory planner based on linear step -- no control for velocity, nor acceleration, constant predifined step only
*/
class LinearStepPlanner {
public:
    // Constructors
    LinearStepPlanner();  // Default
    LinearStepPlanner(struct TrajectoryState trajectory);  // Parameterized
    
    // Destructor
    ~LinearStepPlanner();

    void plan_trajectory(double target_position, double max_velocity, double step);
    double get_next_setpoint();
    void print_summary();


      
    
private:
    // struct TrajectoryState * trajectory; // doing raw pointer requires new in constructor and delete in destructor 
    std::unique_ptr<struct TrajectoryState> trajectory;  // Smart pointer instead of raw pointer
};



/*
Trajectory planner based on trapezoidal velocity profile algorithm.
inputs:
    double x_0  : initial pos
    double x_d  : desired pos (target)
    double v_max: max velocity
    double a_max: max acceleration
    double dt   : time step 
    bool triangle: if true, then no cruise phase ... velocity starts decreasing immediately after reaching vmax


                    vmax
             |   |--------------------|   |
             |  /|                    |\  |
             | / |                    | \ | 
    ---------|/  |                    |  \|--------- t 
            t0  t1                    t2  t3
             |___|________t_cr________|___|
            t_acc                       t_dec
*/
class TrapezoidalPlanner{

    public:
    
    TrapezoidalPlanner();
    TrapezoidalPlanner(double x_0,double x_d, double v_max, double a_max, double dt, bool triangle);
    
    ~TrapezoidalPlanner();
    double update();
    bool is_done() const;

    private:
        double x_0=0.0; // initial pos
        double x_d=0.0; // target pos
        double dtg=0.0; // distance to go
        double x_t=0.0; // current pos
        double v_max = 0.0; // max allowed velocity
        double a_max = 0.0; // max acceleration
        double dtg_acc = 0.0; // distance to go (dtg) during acceleration
        double dtg_dec = 0.0; // dtg during deceleration
        double t_acc = 0.0; // acceleration phase time (t1 - t0)
        double t_dec = 0.0; // deceleration phase time (t3 - t2)
        double t_cr = 0.0; // constant velocity phase time(t2 -t1)
        double t_total = 0.0; // total motion time 
        double t =0.0; // current time instance
        double dt = 0.0; // time step 
        bool done = false; // flag indicates complete motion 
        bool triangle = false; // if true, then no const velocity phase, i.e., t_cr = 0.0


};

#endif // TRAJECTORY_H