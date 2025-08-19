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

/* ---------- ctor / dtor -------------------------------------------------- */
SpresenseImuNode::SpresenseImuNode() : rclcpp::Node("spresense_imu_node")
{
  declare_parameter("port",      "/dev/sensors/spresense_imu");
  declare_parameter("frame_id",  "imu_link");
  declare_parameter("imu_topic", "/imu/spresense");
  declare_parameter("time_out",  10.0);
  declare_parameter("baudrate",  921600);

  get_parameter("port",      port_);
  get_parameter("frame_id",  frame_id_);
  get_parameter("imu_topic", imu_topic_);
  get_parameter("time_out",  time_out_);
  get_parameter("baudrate",  baudrate_);

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  timer_   = create_wall_timer(1ms, std::bind(&SpresenseImuNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "Init port=%s baud=%d", port_.c_str(), baudrate_);
  openSerial(port_, baudrate_, time_out_);
}

SpresenseImuNode::~SpresenseImuNode()
{
  if (imu_serial_.isOpen()) imu_serial_.close();
}

/* ---------- シリアルオープン ------------------------------------------- */
void SpresenseImuNode::openSerial(const std::string& port, const int& baud, const float&)
{
  serial::Timeout to = serial::Timeout::simpleTimeout(10);
  while (rclcpp::ok())
  {
    try {
      imu_serial_.setPort(port); imu_serial_.setBaudrate(baud); imu_serial_.setTimeout(to);
      imu_serial_.open();  RCLCPP_INFO(get_logger(), "Serial opened.");
      break;
    } catch (serial::IOException&) {
      RCLCPP_WARN(get_logger(), "Serial open failed… retry in 5s"); rclcpp::sleep_for(5s);
    }
  }
}

/* ---------- タイマループ ------------------------------------------------- */
void SpresenseImuNode::controlLoop()
{
  if (!imu_serial_.isOpen()) return;

  size_t avail = imu_serial_.available();
  if (!avail) return;

  std::vector<uint8_t> tmp(avail);
  imu_serial_.read(tmp, avail);
  for (uint8_t b : tmp) processByte(b);
}

/* ---------- 1バイトずつパース ------------------------------------------ */
void SpresenseImuNode::processByte(uint8_t byte)
{
  buf_.push_back(byte);

  /* 先頭同期 */
  if (buf_.size() == 1 && buf_[0] != HEADER_BYTE) { buf_.clear(); return; }

  /* サイズ不足 */
  if (buf_.size() < PKT_SIZE) return;

  /* 34byte 以上溜まっている場合の冗長分は先頭を詰める */
  if (buf_.size() > PKT_SIZE) { buf_.erase(buf_.begin(), buf_.end() - PKT_SIZE); }

  /* チェックサム検証 */
  if (xor_checksum(buf_.data(), CHECK_IDX) != buf_[CHECK_IDX]) {
    RCLCPP_DEBUG(get_logger(), "CRC fail");
    buf_.clear(); return;
  }

  /* ---------- データ展開 ---------- */
  /* buf_[1] 〜 buf_[32] が imu_raw32_t */
  /* フォーマット: <Ifffffff (ts,temp,gx,gy,gz,ax,ay,az) */
  static_assert(PAYLOAD_SIZE == 28);
  struct Payload {
    uint32_t ts; float gx, gy, gz, ax, ay, az;
  } __attribute__((packed));
  Payload p;
  memcpy(&p, &buf_[1], sizeof(p));

  float acc[3] = { p.ax, p.ay, p.az };
  float gyr[3] = { p.gx, p.gy, p.gz };            // 既に rad/s ならそのまま

  publishImu(acc, gyr);
  buf_.clear();
}

/* ---------- Publish ----------------------------------------------------- */
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

  /* orientation 不明なら無効化 */
  msg.orientation_covariance[0] = -1;

  imu_pub_->publish(msg);
}

/* ---------- main -------------------------------------------------------- */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpresenseImuNode>());
  rclcpp::shutdown();
  return 0;
}
