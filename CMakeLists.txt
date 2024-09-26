cmake_minimum_required(VERSION 2.8.12)
project(robot_rodeo_gym)

if (POLICY CMP0048) # Disable warning about deprecated command
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

add_compile_options(-pthread) # Allow usage of pthread library

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-transport9 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

find_package(
        catkin REQUIRED
        COMPONENTS
        roscpp
        rospy
        rqt_gui
        rqt_gui_cpp
        grid_map_msgs
        grid_map_pcl
        tf2
        tf2_ros
        eigen_conversions
        message_generation
        std_msgs
        geometry_msgs
)

find_package(Qt5Widgets REQUIRED)

add_message_files(
        DIRECTORY msg
        FILES
        EntityPose.msg
)

add_service_files(
        DIRECTORY srv
        FILES
        SetEntityPose.srv
        SetCameraFollow.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        LIBRARIES
        rqt_score
        flipper_control_plugin
        CATKIN_DEPENDS
        roscpp
        rospy
        rqt_gui
        rqt_gui_cpp
        tf2
        tf2_ros
        eigen_conversions
        message_generation
        std_msgs
        geometry_msgs
        message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Qt5Widgets_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})
add_definitions(${Qt5Widgets_DEFINITIONS})

## Ground truth publisher node
add_executable(ground_truth_publisher src/ground_truth_publisher.cpp)
target_link_libraries(ground_truth_publisher ${catkin_LIBRARIES} ignition-transport9::core ignition-msgs6::core)
target_include_directories(ground_truth_publisher PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(ground_truth_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(set_entity_pose src/set_entity_pose.cpp)
target_link_libraries(set_entity_pose ${catkin_LIBRARIES} ignition-transport9::core ignition-msgs6::core)
target_include_directories(set_entity_pose PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(set_entity_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(crop_pointcloud src/crop_point_cloud.cpp)
target_link_libraries(crop_pointcloud ${catkin_LIBRARIES})
add_dependencies(crop_pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(set_camera_follow src/set_camera_follow.cpp)
target_link_libraries(set_camera_follow ${catkin_LIBRARIES} ignition-transport9::core ignition-msgs6::core)
target_include_directories(set_camera_follow PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(set_camera_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Marv unique ##
add_executable(marv_compute_distance_from_ground robots/marv/src/distance_from_ground.cpp)
target_link_libraries(marv_compute_distance_from_ground ${catkin_LIBRARIES})
add_dependencies(marv_compute_distance_from_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(marv_reset_robot robots/marv/src/reset_robot.cpp)
target_link_libraries(marv_reset_robot ${catkin_LIBRARIES})
add_dependencies(marv_reset_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Flipper control plugin for gazebo
add_library(flipper_control_plugin robots/marv/src/flipper_control_plugin.cpp)
target_link_libraries(flipper_control_plugin ignition-gazebo6::core ignition-common4::ignition-common4)

set(rqt_score_HDRS include/robot_rodeo_gym/rqt_score.h)
qt5_wrap_cpp(rqt_score_MOCS ${rqt_score_HDRS})

set(rqt_score_UIS robots/marv/ui/rqt_score.ui)
qt5_wrap_ui(rqt_score_UIS_H ${rqt_score_UIS})

add_library(rqt_score ${rqt_score_MOCS} ${rqt_score_UIS_H} robots/marv/src/rqt_score.cpp)
target_link_libraries(rqt_score ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})
add_dependencies(rqt_score ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rqt_score
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS robots/marv/scripts/marv_rqt_score
        arenas/long_arena/src/long_obstacle_id_publisher.py
        arenas/short_arena/src/short_obstacle_id_publisher.py
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS robots/marv/scripts/marv_rqt_score
        arenas/long_arena/src/long_obstacle_id_publisher.py
        arenas/short_arena/src/short_obstacle_id_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
        arenas/long_arena/src/long_obstacle_id_publisher.py
        arenas/short_arena/src/short_obstacle_id_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)