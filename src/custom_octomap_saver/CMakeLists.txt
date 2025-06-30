cmake_minimum_required(VERSION 3.0.2)
project(custom_octomap_saver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  cv_bridge
  tf
  cv_bridge
  octomap_server  

)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs tf cv_bridge octomap_server  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(custom_octomap_saver src/custom_octomap_saver.cpp)
target_link_libraries(custom_octomap_saver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


