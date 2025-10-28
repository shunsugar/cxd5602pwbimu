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

#include "cxd5602pwbimu/quat_attitude_estimator.hpp"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

namespace quat_attitude_estimator
{

QuatAttitudeEstimatorNode::QuatAttitudeEstimatorNode(const rclcpp::NodeOptions & options)
: Node("quat_attitude_estimator", options),
  q_{1.0, 0.0, 0.0, 0.0},
  have_prev_stamp_(false),
  correct_initial_yaw_(true),
  yaw_initialized_(false),
  initial_yaw_(0.0)
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/imu/spresense");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/imu/spresense/inityaw");
  correct_initial_yaw_ = this->declare_parameter<bool>("correct_initial_yaw", true);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    input_topic_, 10, std::bind(&QuatAttitudeEstimatorNode::imuCallback, this, std::placeholders::_1));
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Initialized quat_attitude_estimator_node.");
  RCLCPP_INFO(this->get_logger(), "correct_initial_yaw: %s", correct_initial_yaw_ ? "true" : "false");
}

/*
 * Quaternion multiplication (Hamilton product)
 * q = [q0, q1, q2, q3] where q0 = scalar (w)
 */
std::array<double,4> QuatAttitudeEstimatorNode::quaternionMultiply(const std::array<double,4>& a, const std::array<double,4>& b)
{
  double a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
  double b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];

  return {
    a0*b0 - a1*b1 - a2*b2 - a3*b3,
    a0*b1 + a1*b0 + a2*b3 - a3*b2,
    a0*b2 - a1*b3 + a2*b0 + a3*b1,
    a0*b3 + a1*b2 - a2*b1 + a3*b0
  };
}

void QuatAttitudeEstimatorNode::normalizeQuaternion(std::array<double,4>& q)
{
  double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n > 1e-12)
  {
    q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
  }
  else
  {
    q = {1.0, 0.0, 0.0, 0.0};
  }
}

void QuatAttitudeEstimatorNode::quaternionToRPY(const std::array<double,4>& q, double & roll, double & pitch, double & yaw)
{
  tf2::Quaternion tq(q[1], q[2], q[3], q[0]); // tf2: (x,y,z,w)
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
}

std::array<double,4> QuatAttitudeEstimatorNode::rpyToQuaternion(double roll, double pitch, double yaw)
{
  tf2::Quaternion tq;
  tq.setRPY(roll, pitch, yaw);
  tq.normalize();
  return { tq.getW(), tq.getX(), tq.getY(), tq.getZ() };
}

void QuatAttitudeEstimatorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  rclcpp::Time stamp = msg->header.stamp;
  double dt = 0.0;

  if (!have_prev_stamp_)
  {
    have_prev_stamp_ = true;
    prev_stamp_ = stamp;
    publishWithOrientation(msg);
    return;
  }
  else
  {
    rclcpp::Duration d = stamp - prev_stamp_;
    dt = d.seconds();
    prev_stamp_ = stamp;
    if (dt <= 0.0 || dt > 1.0)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "unreasonable dt: %.6f, skipping update", dt);
      publishWithOrientation(msg);
      return;
    }
  }

  double wx = msg->angular_velocity.x;
  double wy = msg->angular_velocity.y;
  double wz = msg->angular_velocity.z;

  double rx = wx * dt;
  double ry = wy * dt;
  double rz = wz * dt;
  double rnorm = std::sqrt(rx*rx + ry*ry + rz*rz);

  std::array<double,4> dq;
  if (rnorm > 1e-12)
  {
    double ux = rx / rnorm;
    double uy = ry / rnorm;
    double uz = rz / rnorm;
    double half = 0.5 * rnorm;
    double s = std::sin(half);
    dq[0] = std::cos(half);
    dq[1] = ux * s;
    dq[2] = uy * s;
    dq[3] = uz * s;
  }
  else
  {
    dq[0] = 1.0;
    dq[1] = 0.5 * rx;
    dq[2] = 0.5 * ry;
    dq[3] = 0.5 * rz;
    normalizeQuaternion(dq);
  }

  q_ = quaternionMultiply(q_, dq);
  normalizeQuaternion(q_);

  if (correct_initial_yaw_ && !yaw_initialized_)
  {
    double r, p, y;
    quaternionToRPY(q_, r, p, y);
    initial_yaw_ = y;
    yaw_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial yaw captured: %.6f rad", initial_yaw_);
  }
  publishWithOrientation(msg);
}

void QuatAttitudeEstimatorNode::publishWithOrientation(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  sensor_msgs::msg::Imu imu_msg = *msg;
  std::array<double,4> q_out = q_;
  if (correct_initial_yaw_ && yaw_initialized_) {
    double r, p, y;
    quaternionToRPY(q_out, r, p, y);
    double corrected_y = y - initial_yaw_;
    q_out = rpyToQuaternion(r, p, corrected_y);
  }

  imu_msg.orientation.x = q_out[1];
  imu_msg.orientation.y = q_out[2];
  imu_msg.orientation.z = q_out[3];
  imu_msg.orientation.w = q_out[0];
  
  imu_pub_->publish(imu_msg);
}

} // namespace quat_attitude_estimator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quat_attitude_estimator::QuatAttitudeEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}

