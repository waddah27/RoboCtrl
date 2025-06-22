#include "control.h"
#include "trajectory.hh"
#include <iostream>


int main() {
    initialize_control_system();
    send_motion_command(XD, vMax, STEP);
    
    while (true) {
        double pos = get_current_position();
        double pos_trapz = get_current_pos_trapz();
        std::cout << "Current position: " << pos << std::endl;
        std::cout << "Current position trapez: " << pos_trapz << std::endl;
        if (pos >= XD - STEP) break;
    }
    
    return 0;
}