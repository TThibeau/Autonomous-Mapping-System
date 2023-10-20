cmake_minimum_required(VERSION 3.0.2)
project(obstacle_distance_measurement)

find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        pcl_conversions
        roscpp
        sensor_msgs
        )
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES obstacle_distance_measurement
        #  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_distance_measurement src/obstacle_distance_measurement.cpp)
target_link_libraries(obstacle_distance_measurement ${catkin_LIBRARIES})
