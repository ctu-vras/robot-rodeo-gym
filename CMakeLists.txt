cmake_minimum_required(VERSION 2.8.12)
project(robot_rodeo_gym)

if (POLICY CMP0048) # Disable warning about deprecated command
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

add_compile_options(-pthread) # Allow usage of pthread library

find_package(
        catkin REQUIRED
        COMPONENTS
        roscpp
        rospy
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        rospy
)

include_directories(${catkin_INCLUDE_DIRS})