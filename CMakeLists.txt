cmake_minimum_required(VERSION 3.28)
project(robasic)

set(CMAKE_CXX_STANDARD 17)

# Set the path to the MuJoCo installation directory
set(MUJOCO_HOME "../mujoco_install")

# Add the include directory for MuJoCo
include_directories(${MUJOCO_HOME}/include)

# Add the library directory for MuJoCo
link_directories(${MUJOCO_HOME}/lib)
link_directories(${MUJOCO_HOME}/bin)

add_executable(robasic main.cpp)

target_link_libraries(robasic mujoco glfw)
