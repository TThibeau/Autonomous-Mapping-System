cmake_minimum_required(VERSION 2.8.3)
project(init_tf_publisher)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        geometry_msgs
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(init_tf_publisher src/publish_init.cpp)
target_link_libraries(init_tf_publisher ${catkin_LIBRARIES})

