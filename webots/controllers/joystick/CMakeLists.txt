cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(WEBOTS_HOME $ENV{WEBOTS_HOME})

add_definitions(-I${WEBOTS_HOME}/include/controller/cpp -I${WEBOTS_HOME}/include -Wall -I${WEBOTS_HOME}/include/controller/c -DLINUX)

rosbuild_add_executable(joystick joystick.cpp)
find_library(CPP_CONTROLLER_LIBRARY CppController ${WEBOTS_HOME}/lib)
find_library(C_CONTROLLER_LIBRARY Controller ${WEBOTS_HOME}/lib)

target_link_libraries(joystick ${C_CONTROLLER_LIBRARY} ${CPP_CONTROLLER_LIBRARY})

