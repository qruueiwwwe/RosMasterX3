cmake_minimum_required(VERSION 3.0.2)
project(driver_node_ros1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 