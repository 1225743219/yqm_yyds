cmake_minimum_required(VERSION 3.10)
project(yqmrcf_chassis_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror -Wno-enum-compare)

set(ROS_DEPENDENCIES
        roscpp
        std_msgs
        yqmrcf_msgs
        yqmrcf_common
        pluginlib
        hardware_interface
        controller_interface
        realtime_tools
        control_toolbox
        effort_controllers
        tf2
        tf2_geometry_msgs

        rm_common
        )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES})
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIR}
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${ROS_DEPENDENCIES}
        )
 
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
        src/chassis_base.cpp
        src/balance_pid.cpp
        src/mecanum.cpp
        src/balance_lqr.cpp
        src/omni.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
