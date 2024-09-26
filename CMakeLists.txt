cmake_minimum_required(VERSION 2.8.12)
project(robot_rodeo_gym)

if (POLICY CMP0048) # Disable warning about deprecated command
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

add_compile_options(-pthread) # Allow usage of pthread library

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

find_package(
        catkin REQUIRED
        COMPONENTS
        roscpp
        rospy
        rqt_gui
        rqt_gui_cpp
)

find_package(Qt5Widgets REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        rospy
        rqt_gui
        rqt_gui_cpp
)


include_directories(include ${catkin_INCLUDE_DIRS} ${Qt5Widgets_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})
add_definitions(${Qt5Widgets_DEFINITIONS})

set(rqt_score_HDRS include/robot_rodeo_gym/rqt_score.h)
qt5_wrap_cpp(rqt_score_MOCS ${rqt_score_HDRS})

set(rqt_score_UIS ui/rqt_score.ui)
qt5_wrap_ui(rqt_score_UIS_H ${rqt_score_UIS})

add_library(rqt_score ${rqt_score_MOCS} ${rqt_score_UIS_H} src/rqt_score.cpp)
target_link_libraries(rqt_score ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})
add_dependencies(rqt_score ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)