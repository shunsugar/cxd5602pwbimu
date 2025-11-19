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
#include <nav_msgs/msg/odometry.hpp>

class OdomXyCorrectorNode : public rclcpp::Node
{
public:
  OdomXyCorrectorNode() : Node("odom_initial_xy_corrector")
  {
    declare_parameter("input_topic", "/odom/UM982");
    declare_parameter("output_topic", "/odom/UM982/initxy");

    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, 10,
      std::bind(&OdomXyCorrectorNode::callback, this, std::placeholders::_1));

    pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "Odometry XY Corrector Node started");
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!initialized_) {
      initial_pos_x_ = msg->pose.pose.position.x;
      initial_pos_y_ = msg->pose.pose.position.y;
      initialized_ = true;

      RCLCPP_INFO(get_logger(), "Initial XY captured: x=%.4f y=%.4f", initial_pos_x_, initial_pos_y_);
    }

    nav_msgs::msg::Odometry out = *msg;

    double dx = msg->pose.pose.position.x - initial_pos_x_;
    double dy = msg->pose.pose.position.y - initial_pos_y_;

    out.pose.pose.position.x = dx;
    out.pose.pose.position.y = dy;

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  bool initialized_ = false;
  double initial_pos_x_ = 0.0;
  double initial_pos_y_ = 0.0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomXyCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}

