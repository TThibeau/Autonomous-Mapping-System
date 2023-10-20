cmake_minimum_required(VERSION 3.0.2)
project(body_corrected_publisher)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf

        )

catkin_package(
        CATKIN_DEPENDS roscpp tf
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(body_corrected_publisher src/body_corrected_publisher.cpp)
target_link_libraries(body_corrected_publisher ${catkin_LIBRARIES})