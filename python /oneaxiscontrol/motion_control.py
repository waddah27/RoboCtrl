import ctypes
import os
import sys

# Load the shared library
def load_library():
    # Path to the built library
    lib_path = os.path.join(os.path.dirname(__file__), '..', 'build', 'liboneaxiscontrol.so')
    if sys.platform == 'win32':
        lib_path = os.path.join(os.path.dirname(__file__), '..', 'build', 'Debug', 'oneaxiscontrol.dll')
    elif sys.platform == 'darwin':
        lib_path = os.path.join(os.path.dirname(__file__), '..', 'build', 'liboneaxiscontrol.dylib')
    
    lib = ctypes.CDLL(lib_path)
    
    # Define function prototypes
    lib.initialize_control_system.argtypes = []
    lib.initialize_control_system.restype = None
    
    lib.send_motion_command.argtypes = [ctypes.c_double, ctypes.c_double]
    lib.send_motion_command.restype = None
    
    lib.get_current_position.argtypes = []
    lib.get_current_position.restype = ctypes.c_double
    
    return lib

# Initialize the library
control_lib = load_library()

# Python interface functions
def initialize():
    """Initialize the control system"""
    control_lib.initialize_control_system()

def move_to(position, velocity=1.0):
    """Send a motion command to the control system"""
    control_lib.send_motion_command(ctypes.c_double(position), ctypes.c_double(velocity))

def get_position():
    """Get the current position"""
    return control_lib.get_current_position()