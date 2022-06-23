cmake_minimum_required(VERSION 2.8.3)
project(drive_base_tutorial)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs tf nav_msgs std_msgs roscpp)

catkin_package(
  LIBRARIES drive_base_tutorial
  CATKIN_DEPENDS std_msgs roscpp geometry_msgs tf nav_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(drive_base src/simulation.cpp)
add_dependencies(drive_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_base ${catkin_LIBRARIES})

add_executable(robot_driver src/drive_base.cpp)
add_dependencies(robot_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_driver ${catkin_LIBRARIES})
