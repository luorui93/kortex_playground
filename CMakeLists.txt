cmake_minimum_required(VERSION 3.0.2)
project(kortex_playground)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    moveit_msgs
    tf
    kdl_parser
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES promp_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(kdl_test 
    test/kdl_test.cpp
)
target_link_libraries(kdl_test
    ${catkin_LIBRARIES}
)

add_executable(kinova_teleop_node
    src/kinova_controller.cpp
    nodes/kinova_teleop_node.cpp
)

target_link_libraries(kinova_teleop_node
    ${catkin_LIBRARIES}
)

add_dependencies(kinova_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
