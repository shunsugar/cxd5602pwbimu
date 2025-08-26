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

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>

class SpresenseImuNode : public rclcpp::Node
{
public:
  SpresenseImuNode();
  ~SpresenseImuNode();

private:
  std::string port_, frame_id_, imu_topic_;
  float  time_out_;
  int    baudrate_;

  static constexpr uint8_t HEADER_BYTE  = 0x55;
  static constexpr size_t  PKT_SIZE     = 30;          // 0x55 + 32 + CRC
  static constexpr size_t  PAYLOAD_SIZE = 28;          // imu_raw32_t
  static constexpr size_t  CHECK_IDX    = 29;          // CRC
  static constexpr double  DEG2RAD      = M_PI / 180.0;

  serial::Serial                       imu_serial_;
  rclcpp::TimerBase::SharedPtr         timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  sensor_msgs::msg::Imu                imu_msg_;

  std::vector<uint8_t> buf_;

  void openSerial(const std::string&, const int&, const float&);
  void controlLoop();
  void processByte(uint8_t byte);
  void publishImu(const float acc[3], const float gyr[3]);

  static uint8_t xor_checksum(const uint8_t* data, size_t len)
  {
    uint8_t c = 0; while (len--) c ^= *data++; return c;
  }

  bool gyro_bias_correction_;
  bool bias_computed_;
  int  sample_count_;
  int  samples_received_;
  std::array<double, 3> gyro_sum_{0.0, 0.0, 0.0};
  std::array<double, 3> gyro_bias_{0.0, 0.0, 0.0};
  std::array<double, 3> min_gyro_{ {1e9, 1e9, 1e9} };
  std::array<double, 3> max_gyro_{ {-1e9, -1e9, -1e9} };
};
