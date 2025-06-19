#include "control.h"
#include <iostream>
#define STEP 0.2
int main() {
    initialize_control_system();
    send_motion_command(10.0, 2.0, STEP);
    
    while (true) {
        double pos = get_current_position();
        std::cout << "Current position: " << pos << std::endl;
        if (pos >= 9.9) break;
    }
    
    return 0;
}