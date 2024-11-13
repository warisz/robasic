cmake_minimum_required(VERSION 3.28)
project(robasic)

set(CMAKE_CXX_STANDARD 17)

# Set the path to the MuJoCo installation directory
set(MUJOCO_HOME "../mujoco_install")

set(CMAKE_TOOLCHAIN_FILE "/Users/warisz/Code/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "VCPKG toolchain file")

# Add the include directory for MuJoCo
include_directories(${MUJOCO_HOME}/include)

# Add the library directory for MuJoCo
link_directories(${MUJOCO_HOME}/lib)
link_directories(${MUJOCO_HOME}/bin)

add_executable(robasic main.cpp)

target_link_libraries(
        robasic
        mujoco
        glfw
)
