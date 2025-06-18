import time
import threading
from queue import Queue
import math

class CNCAxisController:
    def __init__(self, num_motors=1, steps_per_mm=100, max_speed=500, acceleration=100):
        """
        Initialize the X-axis controller
        
        Parameters:
        - num_motors: 1 or 2 (number of motors driving the axis)
        - steps_per_mm: Steps per millimeter for the motors
        - max_speed: Maximum speed in mm/min
        - acceleration: Acceleration in mm/s²
        """
        self.num_motors = num_motors
        self.steps_per_mm = steps_per_mm
        self.max_speed = max_speed  # mm/min
        self.acceleration = acceleration  # mm/s²
        
        self.current_position = 0.0  # mm
        self.target_position = 0.0  # mm
        self.is_moving = False
        self.emergency_stop = False
        
        # For multi-motor configuration
        self.motor_positions = [0.0, 0.0]  # Individual motor positions
        
        # Command queue for UI commands
        self.command_queue = Queue()
        
        # Start the motion control thread
        self.control_thread = threading.Thread(target=self._motion_control_loop, daemon=True)
        self.control_thread.start()
        
    def move_to_position(self, position_mm):
        """Command to move to absolute position (in mm)"""
        if not self.emergency_stop:
            self.command_queue.put(('MOVE_ABS', position_mm))
    
    def move_relative(self, distance_mm):
        """Command to move relative distance (in mm)"""
        if not self.emergency_stop:
            self.command_queue.put(('MOVE_REL', distance_mm))
    
    def stop(self):
        """Stop motion immediately"""
        self.command_queue.put(('STOP', None))
    
    def emergency_stop(self):
        """Emergency stop - halts all motion"""
        self.emergency_stop = True
        self.stop()
    
    def reset_emergency(self):
        """Reset emergency stop condition"""
        self.emergency_stop = False
    
    def _motion_control_loop(self):
        """Main control loop running in a separate thread"""
        while True:
            if not self.command_queue.empty():
                cmd, value = self.command_queue.get()
                
                if cmd == 'MOVE_ABS':
                    self.target_position = value
                    self._execute_move()
                elif cmd == 'MOVE_REL':
                    self.target_position = self.current_position + value
                    self._execute_move()
                elif cmd == 'STOP':
                    self.is_moving = False
                    # Implement motor stop here
                    print("Motion stopped")
            
            time.sleep(0.001)  # Short sleep to prevent CPU overuse
    
    def _execute_move(self):
        """Execute the move from current to target position"""
        if self.emergency_stop:
            return
            
        distance = self.target_position - self.current_position
        if abs(distance) < 0.001:  # Already at position (within tolerance)
            return
            
        self.is_moving = True
        direction = 1 if distance > 0 else -1
        distance = abs(distance)
        
        # Convert speeds from mm/min to mm/s
        max_speed_mms = self.max_speed / 60.0
        
        # Calculate move parameters (trapezoidal velocity profile)
        # Time to accelerate to max speed
        t_accel = max_speed_mms / self.acceleration
        # Distance covered during acceleration
        d_accel = 0.5 * self.acceleration * t_accel**2
        
        if 2 * d_accel > distance:
            # Triangular profile (won't reach max speed)
            t_accel = math.sqrt(distance / self.acceleration)
            d_accel = 0.5 * self.acceleration * t_accel**2
            max_speed_mms = self.acceleration * t_accel
            t_decel = t_accel
            t_const = 0
        else:
            # Trapezoidal profile
            t_decel = t_accel
            t_const = (distance - 2 * d_accel) / max_speed_mms
        
        total_time = t_accel + t_const + t_decel
        
        # Execute the move
        start_time = time.time()
        elapsed = 0
        
        while elapsed < total_time and not self.emergency_stop:
            if elapsed < t_accel:
                # Acceleration phase
                speed = self.acceleration * elapsed
                distance_covered = 0.5 * self.acceleration * elapsed**2
            elif elapsed < t_accel + t_const:
                # Constant speed phase
                speed = max_speed_mms
                distance_covered = d_accel + max_speed_mms * (elapsed - t_accel)
            else:
                # Deceleration phase
                speed = max_speed_mms - self.acceleration * (elapsed - t_accel - t_const)
                distance_covered = (d_accel + 
                                  max_speed_mms * t_const + 
                                  max_speed_mms * (elapsed - t_accel - t_const) - 
                                  0.5 * self.acceleration * (elapsed - t_accel - t_const)**2)
            
            # Update position
            self.current_position = self.target_position - direction * (distance - distance_covered)
            
            # For dual motor configuration, update each motor position
            if self.num_motors == 2:
                self.motor_positions[0] = self.current_position
                self.motor_positions[1] = self.current_position  # Assuming they move in sync
            
            # Generate step pulses (this would control actual motors in hardware)
            self._generate_step_pulses(direction, speed)
            
            # Control loop timing
            time.sleep(0.001)  # 1ms control loop
            elapsed = time.time() - start_time
        
        self.is_moving = False
        self.current_position = self.target_position  # Ensure final position is exact
    
    def _generate_step_pulses(self, direction, speed):
        """Generate step pulses for the motors (simulated here)"""
        # In a real system, this would interface with motor drivers
        steps_per_second = speed * self.steps_per_mm
        
        if steps_per_second > 0:
            step_delay = 1.0 / steps_per_second
            
            # For dual motor configuration
            for motor in range(self.num_motors):
                # Generate step pulse (simulated)
                # In real hardware: Set step pin high, then low after short delay
                pass
            
            time.sleep(step_delay)
    
    def get_position(self):
        """Get current position (mm)"""
        return self.current_position
    
    def get_status(self):
        """Get system status"""
        return {
            'position': self.current_position,
            'target': self.target_position,
            'moving': self.is_moving,
            'emergency_stop': self.emergency_stop,
            'motors': self.num_motors
        }


# Example UI Interface (simplified)
class CNCUI:
    def __init__(self, controller):
        self.controller = controller
    
    def run(self):
        print("CNC X-Axis Controller")
        print("Commands: [move] [stop] [status] [emergency] [reset] [exit]")
        
        while True:
            cmd = input("> ").strip().lower().split()
            if not cmd:
                continue
                
            if cmd[0] == 'move':
                try:
                    pos = float(cmd[1])
                    self.controller.move_to_position(pos)
                    print(f"Moving to {pos} mm")
                except (IndexError, ValueError):
                    print("Usage: move [position_mm]")
            
            elif cmd[0] == 'stop':
                self.controller.stop()
                print("Stopping motion")
            
            elif cmd[0] == 'status':
                status = self.controller.get_status()
                print(f"Position: {status['position']:.3f} mm")
                print(f"Target: {status['target']:.3f} mm")
                print(f"Moving: {'Yes' if status['moving'] else 'No'}")
                print(f"Motors: {status['motors']}")
                print(f"Emergency Stop: {'Active' if status['emergency_stop'] else 'Inactive'}")
            
            elif cmd[0] == 'emergency':
                self.controller.emergency_stop()
                print("EMERGENCY STOP ACTIVATED")
            
            elif cmd[0] == 'reset':
                self.controller.reset_emergency()
                print("Emergency stop reset")
            
            elif cmd[0] == 'exit':
                break
            
            else:
                print("Invalid command")


# Example usage
if __name__ == "__main__":
    # Create controller with 2 motors (can change to 1 if needed)
    cnc_controller = CNCAxisController(num_motors=2, steps_per_mm=200, max_speed=1000, acceleration=200)
    
    # Create and run UI
    ui = CNCUI(cnc_controller)
    ui.run()