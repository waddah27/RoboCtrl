#include "control.h"
#include "trajectory.hh"
#include <iostream>


int main() {
    initialize_control_system();
    send_motion_command(XD, vMax, STEP);
    bool done = false;
    while (true) {
        // double pos = get_current_position();
        // double pos_trapz = get_current_pos_trapz(done);
        get_current_pos_trapzND(done);
        // std::cout << "Current position: " << pos << std::endl;
        // std::cout << "Current position trapez: " << pos_trapz << std::endl;
        // if (pos >= XD - STEP) break; // stop criteria for simple linear step planner 
        if (done) break; // stop criteria for trapezoidal velocity profile planner 
    }
    // while (!plannerTrapzND.is_done()) {
    // std::vector<double> pos = planner.update();
    // for (double v : pos) std::cout << v << " ";
    // std::cout << std::endl;
    
    return 0;
}