cmake_minimum_required(VERSION 3.10)
project(roboctrl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Python configuration
find_package(Python 3.6 REQUIRED COMPONENTS Interpreter Development)

# Add the library
add_library(roboctrl SHARED 
    src/control.cpp 
    src/trajectory.cpp
)

# Include directories
target_include_directories(roboctrl 
    PUBLIC 
    ${PROJECT_SOURCE_DIR}/include
    ${Python_INCLUDE_DIRS}
)

# for binding c++ with Python 
find_package(pybind11 REQUIRED)

pybind11_add_module(trajectory MODULE
    src/trajectory_pybind.cpp
    src/trajectory.cpp
)

target_include_directories(trajectory PRIVATE ${PROJECT_SOURCE_DIR}/include)

set_target_properties(trajectory PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/python/roboctrl
)

# Link Python
target_link_libraries(roboctrl ${Python_LIBRARIES})

# For building a standalone executable (optional)
add_executable(run src/main.cpp)
target_link_libraries(run roboctrl)