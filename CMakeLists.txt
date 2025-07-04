cmake_minimum_required(VERSION 3.8)
project(cxd5602pwbimu)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

# Add include directories
include_directories(
  include
)

# install
install(DIRECTORY include launch src
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(spresense_imu_node src/spresense_imu_node.cpp)
ament_target_dependencies(spresense_imu_node rclcpp sensor_msgs serial)

# Install cpp executable
install(TARGETS spresense_imu_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
