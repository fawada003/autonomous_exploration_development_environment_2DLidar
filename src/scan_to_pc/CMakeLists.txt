cmake_minimum_required(VERSION 3.0.2)
project(scan_to_pc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  laser_geometry
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_to_pc src/scan_to_pc.cpp)
target_link_libraries(scan_to_pc ${catkin_LIBRARIES})