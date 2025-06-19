#include "control.h"
#include <iostream>
#define STEP 0.2
double target_pos = 3;
int main() {
    initialize_control_system();
    send_motion_command(3, 2.0, STEP);
    
    while (true) {
        double pos = get_current_position();
        std::cout << "Current position: " << pos << std::endl;
        if (pos >= target_pos - STEP) break;
    }
    
    return 0;
}