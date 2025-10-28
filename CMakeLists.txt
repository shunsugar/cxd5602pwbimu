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
find_package(Eigen3 REQUIRED)

# Add include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# install
install(DIRECTORY config include launch src
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create and install cpp executable
add_executable(cxd5602pwbimu_node src/driver/cxd5602pwbimu_node.cpp)
ament_target_dependencies(cxd5602pwbimu_node rclcpp sensor_msgs serial)

install(TARGETS cxd5602pwbimu_node
  DESTINATION lib/${PROJECT_NAME})

# Create and install cpp executable
add_executable(wheel_spimu_odom src/processor/wheel_spimu_odom.cpp)
ament_target_dependencies(wheel_spimu_odom
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS wheel_spimu_odom
  DESTINATION lib/${PROJECT_NAME})

# Create and install cpp executable
add_executable(gyrobias_estimation src/processor/gyrobias_estimation.cpp)
ament_target_dependencies(gyrobias_estimation rclcpp sensor_msgs)

install(TARGETS gyrobias_estimation
  DESTINATION lib/${PROJECT_NAME})

# Create and install cpp executable
add_executable(gyrobias_corrector src/processor/gyrobias_corrector.cpp)
ament_target_dependencies(gyrobias_corrector rclcpp sensor_msgs)

install(TARGETS gyrobias_corrector
  DESTINATION lib/${PROJECT_NAME})

# Create and install cpp executable
add_executable(initial_yaw_corrector src/processor/initial_yaw_corrector.cpp)
ament_target_dependencies(initial_yaw_corrector rclcpp sensor_msgs tf2_geometry_msgs)

install(TARGETS initial_yaw_corrector
  DESTINATION lib/${PROJECT_NAME})

# Create and install cpp executable
add_executable(yaw_kalman_node src/processor/yaw_kalman_node.cpp)
ament_target_dependencies(yaw_kalman_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)

install(TARGETS yaw_kalman_node
  DESTINATION lib/${PROJECT_NAME})

# Create and install cpp executable
add_executable(ekf_myself src/processor/ekf_myself.cpp)
ament_target_dependencies(ekf_myself
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_include_directories(ekf_myself PRIVATE ${EIGEN3_INCLUDE_DIR})

# Optional: compiler flags
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(ekf_myself PRIVATE -O3 -march=native -Wall -Wextra)
endif()

install(TARGETS ekf_myself
  DESTINATION lib/${PROJECT_NAME})

# Create and install cpp executable
add_executable(quat_attitude_estimator src/processor/quat_attitude_estimator.cpp)
ament_target_dependencies(quat_attitude_estimator
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS quat_attitude_estimator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
