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

# Link Python
target_link_libraries(roboctrl ${Python_LIBRARIES})

# For building a standalone executable (optional)
add_executable(run src/main.cpp)
target_link_libraries(run roboctrl)