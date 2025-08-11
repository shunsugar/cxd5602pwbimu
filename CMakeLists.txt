cmake_minimum_required(VERSION 3.8)
project(cxd5602pwbimu)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# Add include directories
include_directories(
  include
)

# install
install(DIRECTORY config include launch src
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(cxd5602pwbimu_node src/cxd5602pwbimu_node.cpp)
ament_target_dependencies(cxd5602pwbimu_node rclcpp sensor_msgs serial)

# Install cpp executable
install(TARGETS cxd5602pwbimu_node
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(wheel_spimu_odom src/wheel_spimu_odom.cpp)
ament_target_dependencies(wheel_spimu_odom
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install cpp executable
install(TARGETS wheel_spimu_odom
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(gyrobias_estimation src/gyrobias_estimation.cpp)
ament_target_dependencies(gyrobias_estimation rclcpp sensor_msgs)

# Install cpp executable
install(TARGETS gyrobias_estimation
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(gyrobias_corrector src/gyrobias_corrector.cpp)
ament_target_dependencies(gyrobias_corrector rclcpp sensor_msgs)

# Install cpp executable
install(TARGETS gyrobias_corrector
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(initial_yaw_corrector src/initial_yaw_corrector.cpp)
ament_target_dependencies(initial_yaw_corrector rclcpp sensor_msgs tf2_geometry_msgs)

# Install cpp executable
install(TARGETS initial_yaw_corrector
  DESTINATION lib/${PROJECT_NAME})

ament_package()
