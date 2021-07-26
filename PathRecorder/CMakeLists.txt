cmake_minimum_required(VERSION 2.8.13)
project(path_recorder)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        roscpp
        rosbag
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS nav_msgs roscpp rosbag
)

add_executable(path_recorder record.cc src/path_recorder.cc)
target_link_libraries(path_recorder ${catkin_LIBRARIES})

