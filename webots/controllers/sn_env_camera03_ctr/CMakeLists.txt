cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rosbuild_init()


#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


set(WEBOTS_HOME $ENV{WEBOTS_HOME})
#add_definitions(-I${WEBOTS_HOME}/include/controller/cpp -I${WEBOTS_HOME}/include -Wall -I${WEBOTS_HOME}/include/controller/c -DLINUX)
add_definitions(-I${WEBOTS_HOME}/include/controller/cpp -I${WEBOTS_HOME}/include -Wall -I${WEBOTS_HOME}/include/controller/c -DOSX -DOS=OSX)


rosbuild_add_executable(sn_env_camera03_ctr sn_env_camera03_ctr.cpp)

find_library(CPP_CONTROLLER_LIBRARY CppController ${WEBOTS_HOME}/lib)
find_library(C_CONTROLLER_LIBRARY Controller ${WEBOTS_HOME}/lib)

target_link_libraries(sn_env_camera03_ctr ${C_CONTROLLER_LIBRARY} ${CPP_CONTROLLER_LIBRARY})


#########################################################
include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost COMPONENTS thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# We list find_package-able ROS packages' names here and skip the ones that are not catkinized yet.
#find_package(ROS REQUIRED COMPONENTS catkin roscpp std_msgs roslib)
include_directories(${ROS_INCLUDE_DIRS})

#if (NOT ROS_FOUND)
#  message(FATAL_ERROR "Didn't manage to find ROS components. Did you put e.g. /opt/ros/fuerte (or wherever) in your CMAKE_PREFIX_PATH?")
#endif()

# The following packages are not find_package-able yet, 
# 
# We can't do:
#  find_package(cv_bridge)
#  find_package(image_transport)
#
# We have to manually include them.
include_directories(~/ros/image_common/image_transport/include ~/ros/vision_opencv/cv_bridge/include)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# Now we have to link the TARGET or it won't compile
#target_link_libraries(mjpeg_server ${Boost_LIBRARIES} ${ROS_LIBRARIES} ${OpenCV_LIBS} /opt/ros/fuerte/stacks/image_common/image_transport/lib/libimage_transport.so /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/lib/libcv_bridge.so)

# We have a binary we want to install
install(TARGETS ${PROJECT_NAME} DESTINATION share/${PROJECT_NAME}/bin)

# We want to install our header files
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

# We want to install our launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch FILES_MATCHING PATTERN "*.launch" PATTERN ".svn" EXCLUDE)

# We also want to install our manifest.xml
install(FILES manifest.xml DESTINATION share/${PROJECT_NAME})
