#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void initialize_control_system();
void send_motion_command(double position, double velocity);
double get_current_position();

#ifdef __cplusplus
}
#endif

#endif // CONTROL_H