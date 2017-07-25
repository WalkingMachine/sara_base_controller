cmake_minimum_required(VERSION 2.8.3)
project(wm_mecanum_base_controller)

find_package(catkin REQUIRED COMPONENTS
        controller_interface
        nav_msgs
        realtime_tools
        tf
        urdf
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/controller.cpp
        src/speed_limiter.cpp
        include/wm_mecanum_base_controller/controller.h
        include/wm_mecanum_base_controller/speed_limiter.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-std=c++11)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES wm_mecanum_base_controller_pugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})