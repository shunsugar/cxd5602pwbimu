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

#include "cxd5602pwbimu/cxd5602pwbimu_node.hpp"
#include <array>
using namespace std::chrono_literals;

SpresenseImuNode::SpresenseImuNode() : rclcpp::Node("spresense_imu_node")
{
  declare_parameter("port",      "/dev/sensors/spresense_imu");
  declare_parameter("frame_id",  "imu_link");
  declare_parameter("imu_topic", "/imu/spresense/data_raw");
  declare_parameter("time_out",  10.0);
  declare_parameter("baudrate",  460800);

  declare_parameter("gyro_bias_correction", true);
  declare_parameter("gyro_bias_samples",    4800);

  get_parameter("port",      port_);
  get_parameter("frame_id",  frame_id_);
  get_parameter("imu_topic", imu_topic_);
  get_parameter("time_out",  time_out_);
  get_parameter("baudrate",  baudrate_);

  get_parameter("gyro_bias_correction", gyro_bias_correction_);
  get_parameter("gyro_bias_samples",    sample_count_);

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  timer_   = create_wall_timer(1ms, std::bind(&SpresenseImuNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Port:%s baud:%d", port_.c_str(), baudrate_);
  if (gyro_bias_correction_) {
    RCLCPP_INFO(this->get_logger(), "Gyro bias correction enabled (samples:%d)", sample_count_);
  }
  openSerial(port_, baudrate_, time_out_);
}

SpresenseImuNode::~SpresenseImuNode()
{
  if (imu_serial_.isOpen()) imu_serial_.close();
}

void SpresenseImuNode::openSerial(const std::string& port, const int& baud, const float&)
{
  serial::Timeout to = serial::Timeout::simpleTimeout(10);
  while (rclcpp::ok())
  {
    try {
      imu_serial_.setPort(port); imu_serial_.setBaudrate(baud); imu_serial_.setTimeout(to);
      imu_serial_.open();
      RCLCPP_INFO(this->get_logger(), "\033[32mSerial port opened successfully!\033[0m");
      break;
    } catch (serial::IOException&) {
      RCLCPP_WARN(get_logger(), "Serial open failed… retry in 5s"); rclcpp::sleep_for(5s);
    }
  }
}

void SpresenseImuNode::controlLoop()
{
  if (!imu_serial_.isOpen()) return;

  size_t avail = imu_serial_.available();
  if (!avail) return;

  std::vector<uint8_t> tmp(avail);
  imu_serial_.read(tmp, avail);
  for (uint8_t b : tmp) processByte(b);
}

void SpresenseImuNode::processByte(uint8_t byte)
{
  buf_.push_back(byte);

  if (buf_.size() == 1 && buf_[0] != HEADER_BYTE) { buf_.clear(); return; }
  if (buf_.size() < PKT_SIZE) return;
  if (buf_.size() > PKT_SIZE) { buf_.erase(buf_.begin(), buf_.end() - PKT_SIZE); }

  if (xor_checksum(buf_.data(), CHECK_IDX) != buf_[CHECK_IDX]) {
    RCLCPP_DEBUG(get_logger(), "CRC fail");
    buf_.clear(); return;
  }

  static_assert(PAYLOAD_SIZE == 28);
  struct Payload {
    uint32_t ts; float gx, gy, gz, ax, ay, az;
  } __attribute__((packed));
  Payload p;
  memcpy(&p, &buf_[1], sizeof(p));

  float acc[3] = { p.ax, p.ay, p.az };
  float gyr[3] = { p.gx, p.gy, p.gz };

  /* --- Gyro bias correction processing --- */
  if (gyro_bias_correction_ && !bias_computed_) {
    gyro_sum_[0] += gyr[0];
    gyro_sum_[1] += gyr[1];
    gyro_sum_[2] += gyr[2];

    min_gyro_[0] = std::min(min_gyro_[0], (double)gyr[0]);
    min_gyro_[1] = std::min(min_gyro_[1], (double)gyr[1]);
    min_gyro_[2] = std::min(min_gyro_[2], (double)gyr[2]);

    max_gyro_[0] = std::max(max_gyro_[0], (double)gyr[0]);
    max_gyro_[1] = std::max(max_gyro_[1], (double)gyr[1]);
    max_gyro_[2] = std::max(max_gyro_[2], (double)gyr[2]);

    samples_received_++;
    if (samples_received_ % 1000 == 0) {
      RCLCPP_INFO(this->get_logger(), "Bias collection progress: %d / %d",
                  samples_received_, sample_count_);
    }

    if (samples_received_ >= sample_count_) {
      gyro_bias_[0] = gyro_sum_[0] / sample_count_;
      gyro_bias_[1] = gyro_sum_[1] / sample_count_;
      gyro_bias_[2] = gyro_sum_[2] / sample_count_;
      bias_computed_ = true;

      RCLCPP_INFO(this->get_logger(), "=== Gyro Bias Estimation Complete ===");
      RCLCPP_INFO(this->get_logger(), "gx: avg=%+.5f, min=%+.5f, max=%+.5f",
                  gyro_bias_[0], min_gyro_[0], max_gyro_[0]);
      RCLCPP_INFO(this->get_logger(), "gy: avg=%+.5f, min=%+.5f, max=%+.5f",
                  gyro_bias_[1], min_gyro_[1], max_gyro_[1]);
      RCLCPP_INFO(this->get_logger(), "gz: avg=%+.5f, min=%+.5f, max=%+.5f",
                  gyro_bias_[2], min_gyro_[2], max_gyro_[2]);
    }

    buf_.clear();
    return;
  }

  if (bias_computed_) {
    gyr[0] -= gyro_bias_[0];
    gyr[1] -= gyro_bias_[1];
    gyr[2] -= gyro_bias_[2];
  }

  publishImu(acc, gyr);
  buf_.clear();
}

void SpresenseImuNode::publishImu(const float acc[3], const float gyr[3])
{
  auto& msg = imu_msg_;
  msg.header.stamp = now(); msg.header.frame_id = frame_id_;

  msg.linear_acceleration.x = acc[0];
  msg.linear_acceleration.y = acc[1];
  msg.linear_acceleration.z = acc[2];

  msg.angular_velocity.x = gyr[0];
  msg.angular_velocity.y = gyr[1];
  msg.angular_velocity.z = gyr[2];

  msg.orientation_covariance[0] = -1;

  imu_pub_->publish(msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpresenseImuNode>());
  rclcpp::shutdown();
  return 0;
}
