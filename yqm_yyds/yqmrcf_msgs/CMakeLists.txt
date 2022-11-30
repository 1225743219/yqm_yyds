cmake_minimum_required(VERSION 3.10)
project(yqmrcf_msgs)

## Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ROS_DEPENDENCIES
        std_msgs
        message_generation
        geometry_msgs
        actionlib
        actionlib_msgs
        )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES})

add_message_files(
        FILES
        ChassisCmd.msg
        BalanceState.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        actionlib
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS
        ${ROS_DEPENDENCIES}
)
