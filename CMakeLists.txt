cmake_minimum_required(VERSION 3.8)
project(cxd5602pwbimu)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

# Add include directories
include_directories(
  include
)

# install
install(DIRECTORY include src
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(imu_node1 src/imu_node1.cpp)
ament_target_dependencies(imu_node1 rclcpp sensor_msgs serial)

# Install cpp executable
install(TARGETS imu_node1
  DESTINATION lib/${PROJECT_NAME})

ament_package()
