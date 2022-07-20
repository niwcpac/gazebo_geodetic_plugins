cmake_minimum_required(VERSION 3.0.2)
project(gazebo_geodetic_plugins)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++17)
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  nav_msgs
  roscpp
  uuv_gazebo_plugins
  uuv_world_ros_plugins_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(gazebo REQUIRED)

set(CMAKE_AUTOMOC ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    geodetic_grid
    vehicle_position_widget
  CATKIN_DEPENDS
    gazebo_ros
    nav_msgs
    roscpp
    uuv_gazebo_plugins
    uuv_world_ros_plugins_msgs
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5Core_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                  )

link_directories(${catkin_LIBRARY_DIRS}
                  ${Boost_LIBRARY_DIR}
                  ${Qt5Widgets_LIBRARY_DIRS}
                  ${Qt5Core_LIBRARY_DIRS}
                  ${GAZEBO_LIBRARY_DIRS}
                )

add_compile_options("${GAZEBO_CXX_FLAGS}")

add_library(geodetic_grid SHARED
  include/gazebo_geodetic_plugins/geodetic_grid.h
  src/geodetic_grid.cc
)

add_dependencies(geodetic_grid
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(geodetic_grid
    ${catkin_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

add_library(vehicle_position_widget SHARED
  include/gazebo_geodetic_plugins/vehicle_position_widget.h
  src/vehicle_position_widget.cc
)

add_dependencies(vehicle_position_widget
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(vehicle_position_widget
    ${catkin_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

install(TARGETS
  geodetic_grid
  vehicle_position_widget
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN ".h"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY launch worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)
