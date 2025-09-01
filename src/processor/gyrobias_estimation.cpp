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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <string>
#include <limits>

class GyroBiasEstimator : public rclcpp::Node
{
public:
  GyroBiasEstimator() : Node("gyro_bias_estimator")
  {
    declare_parameter("imu_topic", "/imu/spresense");
    declare_parameter("sample_count", 1920);

    get_parameter("imu_topic", imu_topic_);
    get_parameter("sample_count", sample_count_);

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&GyroBiasEstimator::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Waiting for %d IMU samples on topic: %s",
                sample_count_, imu_topic_.c_str());
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (samples_received_ >= sample_count_) return;

    double gx = msg->angular_velocity.x;
    double gy = msg->angular_velocity.y;
    double gz = msg->angular_velocity.z;

    gyro_sum_[0] += gx;
    gyro_sum_[1] += gy;
    gyro_sum_[2] += gz;

    min_gyro_[0] = std::min(min_gyro_[0], gx);
    min_gyro_[1] = std::min(min_gyro_[1], gy);
    min_gyro_[2] = std::min(min_gyro_[2], gz);
    max_gyro_[0] = std::max(max_gyro_[0], gx);
    max_gyro_[1] = std::max(max_gyro_[1], gy);
    max_gyro_[2] = std::max(max_gyro_[2], gz);

    samples_received_++;

    if (samples_received_ % 60 == 0) {
      RCLCPP_INFO(get_logger(), "Progress: %d / %d", samples_received_, sample_count_);
    }

    if (samples_received_ == sample_count_) {
      std::array<double, 3> bias{
        gyro_sum_[0] / sample_count_,
        gyro_sum_[1] / sample_count_,
        gyro_sum_[2] / sample_count_
      };

      RCLCPP_INFO(get_logger(), "=== Gyro Bias Estimation Complete ===");
      RCLCPP_INFO(get_logger(), "gx: avg=%+.5f, min=%+.5f, max=%+.5f", bias[0], min_gyro_[0], max_gyro_[0]);
      RCLCPP_INFO(get_logger(), "gy: avg=%+.5f, min=%+.5f, max=%+.5f", bias[1], min_gyro_[1], max_gyro_[1]);
      RCLCPP_INFO(get_logger(), "gz: avg=%+.5f, min=%+.5f, max=%+.5f", bias[2], min_gyro_[2], max_gyro_[2]);

      rclcpp::shutdown();  // 自動終了
    }
  }

  std::string imu_topic_;
  int sample_count_;
  int samples_received_ = 0;

  std::array<double, 3> gyro_sum_{};
  std::array<double, 3> min_gyro_{
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max()
  };
  std::array<double, 3> max_gyro_{
    std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest()
  };

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GyroBiasEstimator>());
  rclcpp::shutdown();
  return 0;
}

