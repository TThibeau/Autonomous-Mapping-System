cmake_minimum_required(VERSION 3.0.2)
project(point_to_pose)


find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(point_to_pose src/point_to_pose.cpp)
target_link_libraries(point_to_pose ${catkin_LIBRARIES})