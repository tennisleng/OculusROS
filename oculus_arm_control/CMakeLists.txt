cmake_minimum_required(VERSION 3.0.2)
project(oculus_arm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Main executable nodes
add_executable(oculus_interface_node src/oculus_interface_node.cpp)
target_link_libraries(oculus_interface_node ${catkin_LIBRARIES})

add_executable(arm_controller_node src/arm_controller_node.cpp)
target_link_libraries(arm_controller_node ${catkin_LIBRARIES})

# Network latency monitoring
add_executable(network_latency_node 
  src/network_latency_node.cpp
  src/utils/network_latency.cpp
)
target_link_libraries(network_latency_node ${catkin_LIBRARIES})

# Install targets
install(TARGETS 
  oculus_interface_node
  arm_controller_node
  network_latency_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install web interface
install(DIRECTORY web/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/web
)

# Create logs directory
install(CODE "
  file(MAKE_DIRECTORY \${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/logs)
")

# Make Python scripts executable
catkin_install_python(
  PROGRAMS 
    scripts/latency_responder.py
    scripts/web_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 