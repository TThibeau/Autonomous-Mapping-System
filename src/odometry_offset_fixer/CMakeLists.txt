cmake_minimum_required(VERSION 3.0.2)
project(odometry_offset_fixer)


find_package(catkin REQUIRED
        pcl_ros
        pcl_conversions
        roscpp
        sensor_msgs
        tf
        )


catkin_package(CATKIN_DEPENDS roscpp tf pcl_ros sensor_msgs)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(odometry_offset_fixer src/odometry_offset_fixer.cpp)
target_link_libraries(odometry_offset_fixer ${catkin_LIBRARIES})
