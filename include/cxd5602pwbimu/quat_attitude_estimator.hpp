/*
  Copyright 2025 shunsugar <shin.ioaoi@gmail.com>

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#ifndef QUAT_ATTITUDE_ESTIMATOR_HPP_
#define QUAT_ATTITUDE_ESTIMATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <array>
#include <string>

namespace quat_attitude_estimator
{

class QuatAttitudeEstimatorNode : public rclcpp::Node
{
public:
  explicit QuatAttitudeEstimatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~QuatAttitudeEstimatorNode() = default;

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publishWithOrientation(const sensor_msgs::msg::Imu::SharedPtr msg);
  static std::array<double,4> quaternionMultiply(const std::array<double,4>& a, const std::array<double,4>& b);
  static void normalizeQuaternion(std::array<double,4>& q);
  static void quaternionToRPY(const std::array<double,4>& q, double & roll, double & pitch, double & yaw);
  static std::array<double,4> rpyToQuaternion(double roll, double pitch, double yaw);

  std::array<double,4> q_;              // [w, x, y, z] (scalar-first)
  bool have_prev_stamp_;
  rclcpp::Time prev_stamp_;

  bool correct_initial_yaw_;
  bool yaw_initialized_;
  double initial_yaw_;

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

} // namespace quat_attitude_estimator

#endif // QUAT_ATTITUDE_ESTIMATOR_HPP_

