cmake_minimum_required(VERSION 2.8.3)
project(afrl_driver)

find_package(PkgConfig REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_filters
    message_generation
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES afrl_driver
    CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp rospy sensor_msgs std_msgs tf visualization_msgs
    DEPENDS system_lib
)




include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(Driver src/driver.cpp)
target_link_libraries(Driver ${catkin_LIBRARIES})

add_executable(Patterns src/patterns.cpp)
target_link_libraries(Patterns ${catkin_LIBRARIES})

