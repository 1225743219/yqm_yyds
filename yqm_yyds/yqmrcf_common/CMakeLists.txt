cmake_minimum_required(VERSION 3.10)
project(yqmrcf_common)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror -Wno-enum-compare)

set(ROS_DEPENDENCIES
        roscpp
        std_msgs
        yqmrcf_msgs
        )

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES})

# catkin 依赖关系，会传递给使用本包的包
catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        LIBRARIES
        yqmrcf_common
        CATKIN_DEPENDS
        ${ROS_DEPENDENCIES}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

###########
## Build ##
###########

file(GLOB_RECURSE sources "src/*.cpp")

add_library(yqmrcf_common SHARED ${sources})
target_link_libraries(yqmrcf_common ${catkin_LIBRARIES})
add_dependencies(yqmrcf_common yqmrcf_msgs_generate_messages_cpp)
