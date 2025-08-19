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

class ImuBiasCorrectorNode : public rclcpp::Node
{
public:
  ImuBiasCorrectorNode() : Node("gyrobias_corrector_node")
  {
    // パラメータ宣言
    declare_parameter("input_topic", "/imu/spresense");
    declare_parameter("output_topic", "/imu/spresense/gyrobias");
    declare_parameter("gyro_bias_x",  0.00005);
    declare_parameter("gyro_bias_y", -0.00026);
    declare_parameter("gyro_bias_z", -0.00029);

    // パラメータ取得
    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("gyro_bias_x", bias_x_);
    get_parameter("gyro_bias_y", bias_y_);
    get_parameter("gyro_bias_z", bias_z_);

    // 購読とパブリッシュ
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, 10,
      std::bind(&ImuBiasCorrectorNode::callback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "IMU Bias Corrector Node started");
    RCLCPP_INFO(get_logger(), "Bias: gx=%.5f, gy=%.5f, gz=%.5f", bias_x_, bias_y_, bias_z_);
  }

private:
  void callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto corrected_msg = *msg;

    corrected_msg.angular_velocity.x = msg->angular_velocity.x - bias_x_;
    corrected_msg.angular_velocity.y = msg->angular_velocity.y - bias_y_;
    corrected_msg.angular_velocity.z = msg->angular_velocity.z - bias_z_;

    pub_->publish(corrected_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  double bias_x_, bias_y_, bias_z_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuBiasCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}

