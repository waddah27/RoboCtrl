cmake_minimum_required(VERSION 3.10)
project(OneAxisControl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Python configuration
find_package(Python 3.6 REQUIRED COMPONENTS Interpreter Development)

# Add the library
add_library(oneaxiscontrol SHARED 
    src/control.cpp 
    src/trajectory.cpp
)

# Include directories
target_include_directories(oneaxiscontrol 
    PUBLIC 
    ${PROJECT_SOURCE_DIR}/include
    ${Python_INCLUDE_DIRS}
)

# Link Python
target_link_libraries(oneaxiscontrol ${Python_LIBRARIES})

# For building a standalone executable (optional)
add_executable(oneaxis_main src/main.cpp)
target_link_libraries(oneaxis_main oneaxiscontrol)