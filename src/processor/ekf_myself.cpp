#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Dense>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ExtendedKalmanFilter : public rclcpp::Node
{
public:
  ExtendedKalmanFilter()
  : Node("sensor_fusion")
  {
    // parameters
    this->declare_parameter<bool>("ekf_publish_TF", false);
    this->get_parameter("ekf_publish_TF", ekf_publish_TF_);

    sub_a_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/wheel_spimu", 10,
      std::bind(&ExtendedKalmanFilter::sensor_a_callback, this, _1));

    sub_b_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/UM982", 10,
      std::bind(&ExtendedKalmanFilter::sensor_b_callback, this, _1));

    fused_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/fusion/odom", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(100ms, std::bind(&ExtendedKalmanFilter::publish_fused_value, this));

    RCLCPP_INFO(this->get_logger(), "Start ekf_myself node");
  }

private:
  // ROS handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_a_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_b_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // internal state (mirrors python version)
  double GTheta_ = 0.0;
  double GTheta0_ = 0.0;
  double GPSthetayaw0_ = 0.0;
  double DGPStheta_ = 0.0;
  Eigen::Vector2d w_;          // process noise vector
  Eigen::Matrix2d Q_;
  Eigen::Matrix<double,2,4> H_;
  Eigen::Matrix<double,4,4> P_;
  Eigen::Vector4d XX_;

  double prev_time_ = -1.0;
  Eigen::Vector2d prev_pos_;   
  double Speed_ = 0.0;
  double SmpTime_ = 0.1;
  Eigen::Vector2d GpsXY_;
  bool have_gps_ = false;
  int GPS_count_ = 0;
  double GOffset_ = 0.0;
  double offsetyaw_ = 0.0;
  double combineyaw_ = 0.0;
  double robot_yaw_ = 0.0;
  double robot_orientationz_ = 0.0;
  double robot_orientationw_ = 1.0;
  int Number_of_satellites_ = 0;

  // R values
  double R1_ = 0.0, R2_ = 0.0, R3_ = 0.0, R4_ = 0.0;

  bool ekf_publish_TF_ = false;

  nav_msgs::msg::Odometry fused_msg_;

  // helpers
  static double orientation_to_yaw(double z, double w)
  {
    // yaw = atan2(2*(w*z), 1 - 2*z^2)
    return std::atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z));
  }

  static std::pair<double,double> yaw_to_orientation(double yaw)
  {
    double oz = std::sin(yaw / 2.0);
    double ow = std::cos(yaw / 2.0);
    return {oz, ow};
  }

  void sensor_a_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double current_time = this->now().seconds();
    if (prev_time_ >= 0.0) {
      SmpTime_ = current_time - prev_time_;
      if (SmpTime_ <= 0.0) SmpTime_ = 0.1;
    } else {
      SmpTime_ = 0.1;
    }
    prev_time_ = current_time;

    Eigen::Vector2d current_pos(msg->pose.pose.position.x, msg->pose.pose.position.y);
    if (prev_pos_.x() != 0.0 || prev_pos_.y() != 0.0) {
      double dist = (current_pos - prev_pos_).norm();
      Speed_ = dist / SmpTime_;
    } else {
      Speed_ = 0.0;
    }
    prev_pos_ = current_pos;

    GTheta_ = orientation_to_yaw(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  }

  void sensor_b_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    GpsXY_.x() = msg->pose.pose.position.x;
    GpsXY_.y() = msg->pose.pose.position.y;
    have_gps_ = true;

    double GPStheta = orientation_to_yaw(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    DGPStheta_ = GPStheta - GPSthetayaw0_;
    GPSthetayaw0_ = GPStheta;

    // original python uses data.pose.covariance[0] for number of satellites
    // we mirror that behavior but user should ensure publisher populates that field
    Number_of_satellites_ = static_cast<int>(msg->pose.covariance[0]);
  }

  Eigen::Vector2d KalfXY(double Speed, double SmpTime, double GTheta, double R1, double R2)
  {
    if (H_.isZero(0)) {
      initialize(GTheta, SmpTime);
    }

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = R1; R(1,1) = R2;

    double DTheta = GTheta - GTheta0_;
    GTheta0_ = GTheta;

    Eigen::Matrix4d F;
    F << 1, 0, Speed * SmpTime * std::cos(DTheta), -Speed * SmpTime * std::sin(DTheta),
         0, 1, Speed * SmpTime * std::sin(DTheta),  Speed * SmpTime * std::cos(DTheta),
         0, 0, std::cos(DTheta), -std::sin(DTheta),
         0, 0, std::sin(DTheta),  std::cos(DTheta);

    Eigen::Matrix<double,4,2> G;
    G << std::cos(GTheta), -Speed * SmpTime * std::sin(GTheta),
         std::sin(GTheta),  Speed * SmpTime * std::cos(GTheta),
         0, -std::sin(GTheta),
         0,  std::cos(GTheta);

    XX_ = F * XX_ + G * w_;

    return Eigen::Vector2d(XX_.x(), XX_.y());
  }

  Eigen::Vector2d KalfGPSXY(double Speed, double SmpTime, double GTheta, const Eigen::Vector2d &GpsXY, double R1, double R2)
  {
    if (H_.isZero(0)) {
      initializeGPS(GpsXY, GTheta, SmpTime);
    }

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = R1; R(1,1) = R2;

    double DTheta = GTheta - GTheta0_;
    GTheta0_ = GTheta;

    Eigen::Matrix4d F;
    F << 1, 0, Speed * SmpTime * std::cos(DTheta), -Speed * SmpTime * std::sin(DTheta),
         0, 1, Speed * SmpTime * std::sin(DTheta),  Speed * SmpTime * std::cos(DTheta),
         0, 0, std::cos(DTheta), -std::sin(DTheta),
         0, 0, std::sin(DTheta),  std::cos(DTheta);

    Eigen::Matrix<double,4,2> G;
    G << std::cos(GTheta), -Speed * SmpTime * std::sin(GTheta),
         std::sin(GTheta),  Speed * SmpTime * std::cos(GTheta),
         0, -std::sin(GTheta),
         0,  std::cos(GTheta);

    Eigen::Vector2d Y(GpsXY.x(), GpsXY.y());

    XX_ = F * XX_;
    P_ = F * P_ * F.transpose() + G * Q_ * G.transpose();

    Eigen::Matrix<double,2,2> S = H_ * P_ * H_.transpose() + R;
    Eigen::Matrix<double,4,2> K = P_ * H_.transpose() * S.inverse();

    XX_ = XX_ + K * (Y - H_ * XX_);
    P_ = P_ - K * H_ * P_;

    return Eigen::Vector2d(XX_.x(), XX_.y());
  }

  double combine_yaw(double Dtheta, double theta1, double theta2, double w1, double w2)
  {
    const double deg5 = 5.0 * M_PI / 180.0;
    double theta_sum = theta1;
    if (std::abs(Dtheta) < deg5) {
      double t1 = theta1 < 0 ? theta1 + 2.0 * M_PI : theta1;
      double t2 = theta2 < 0 ? theta2 + 2.0 * M_PI : theta2;
      double x1 = w1 * std::cos(t1), y1 = w1 * std::sin(t1);
      double x2 = w2 * std::cos(t2), y2 = w2 * std::sin(t2);
      double x_sum = (x1 + x2) / (w1 + w2);
      double y_sum = (y1 + y2) / (w1 + w2);
      theta_sum = std::atan2(y_sum, x_sum);
      if (theta_sum > M_PI) theta_sum -= 2.0 * M_PI;
      if (theta_sum < -M_PI) theta_sum += 2.0 * M_PI;
    }
    return theta_sum;
  }

  double calculate_offset(double combyaw, double GTheta, double GPStheta)
  {
    double abs_GTheta = std::abs(GTheta);
    double abs_GPStheta = std::abs(GPStheta);
    double abs_combyaw = std::abs(combyaw);
    double pi = M_PI;
    double deference = abs_GTheta + abs_GPStheta;
    double GOffset = 0.0;

    if (GTheta > 0 && GPStheta < 0 && combyaw > 0) {
      GOffset = -(GTheta - combyaw);
    } else if (GTheta > 0 && GPStheta < 0 && combyaw < 0) {
      GOffset = -(GTheta + abs_combyaw);
    } else if (GTheta > 0 && GPStheta > 0 && combyaw > 0 && GTheta > GPStheta) {
      GOffset = -(abs_combyaw - abs_GTheta);
    } else if (GTheta < 0 && GPStheta < 0 && combyaw < 0 && GTheta > GPStheta) {
      GOffset = -(GTheta + abs_combyaw);
    } else if (GTheta < 0 && GPStheta > 0 && combyaw > 0) {
      GOffset = abs_GTheta + combyaw;
    } else if (GTheta < 0 && GPStheta > 0 && combyaw < 0) {
      GOffset = abs_GTheta - abs_combyaw;
    } else if (GTheta > 0 && GPStheta > 0 && combyaw > 0 && GTheta < GPStheta) {
      GOffset = combyaw - GTheta;
    } else if (GTheta < 0 && GPStheta < 0 && combyaw < 0 && GTheta < GPStheta) {
      GOffset = abs_GTheta - abs_combyaw;
    } else if (GTheta > 0 && GPStheta < 0 && combyaw > 0 && deference > pi) {
      GOffset = combyaw - GTheta;
    } else if (GTheta > 0 && GPStheta < 0 && combyaw < 0 && deference > pi) {
      GOffset = pi - GTheta + pi - abs_combyaw;
    } else if (GTheta < 0 && GPStheta > 0 && combyaw > 0 && deference > pi) {
      GOffset = -((pi - combyaw) + (pi - abs_GTheta));
    } else if (GTheta < 0 && GPStheta > 0 && combyaw < 0 && deference > pi) {
      GOffset = -(abs_combyaw - abs_GTheta);
    }

    if (std::abs(GOffset) > 5.0 * pi / 180.0) {
      RCLCPP_WARN(this->get_logger(), "GOffset warning");
      GOffset = 0.0;
    }

    return GOffset;
  }

  void determination_of_R()
  {
    if (0 <= Number_of_satellites_ && Number_of_satellites_ < 4) {
      R1_ = 0.17 * 0.17; R2_ = 0.17 * 0.17; R3_ = 9; R4_ = 1;
    } else if (4 <= Number_of_satellites_ && Number_of_satellites_ < 8) {
      R1_ = 0.08 * 0.08; R2_ = 0.08 * 0.08; R3_ = 4; R4_ = 6;
    } else if (Number_of_satellites_ >= 8) {
      R1_ = 0.05 * 0.05; R2_ = 0.05 * 0.05; R3_ = 2; R4_ = 8;
    }
  }

  void initialize(double GTheta, double SmpTime)
  {
    GTheta0_ = GTheta;
    XX_.setZero();
    XX_(2) = std::cos(GTheta);
    XX_(3) = std::sin(GTheta);

    w_.x() = std::pow(1.379e-3, 2);
    w_.y() = std::pow(0.03 * M_PI / 180.0 * SmpTime, 2);

    H_.setZero();
    H_(0,0) = 1; H_(1,1) = 1;

    Q_.setZero();
    Q_(0,0) = std::pow(1.379e-3,2);
    Q_(1,1) = std::pow(0.03 * M_PI / 180.0 * SmpTime,2);

    Eigen::Matrix<double,4,2> G0;
    G0.setZero();
    G0(0,0) = 1; G0(3,1) = 1;
    P_ = G0 * Q_ * G0.transpose();
  }

  void initializeGPS(const Eigen::Vector2d &GpsXY, double GTheta, double SmpTime)
  {
    GTheta0_ = GTheta;
    XX_.setZero();
    XX_(0) = GpsXY.x(); XX_(1) = GpsXY.y();
    XX_(2) = std::cos(GTheta); XX_(3) = std::sin(GTheta);

    w_.x() = std::pow(1.379e-3, 2);
    w_.y() = std::pow(0.03 * M_PI / 180.0 * SmpTime, 2);

    H_.setZero(); H_(0,0) = 1; H_(1,1) = 1;

    Q_.setZero();
    Q_(0,0) = std::pow(1.379e-3,2);
    Q_(1,1) = std::pow(0.03 * M_PI / 180.0 * SmpTime,2);

    Eigen::Matrix<double,4,2> G0; G0.setZero(); G0(0,0)=1; G0(3,1)=1;
    P_ = G0 * Q_ * G0.transpose();
  }

  void publish_fused_value()
  {
    if (std::isfinite(Speed_) && std::isfinite(SmpTime_) && std::isfinite(GTheta_)) {
      determination_of_R();
      if (have_gps_) {
        Eigen::Vector2d fused_value = KalfGPSXY(Speed_, SmpTime_, GTheta_, GpsXY_, R1_, R2_);
        GPS_count_++;
        if (GPS_count_ % 20 == 0) {
          // need GPStheta stored from sensor_b_callback; we used GPSthetayaw0_ as previous, so compute current GPStheta
          double GPStheta = GPSthetayaw0_;
          combineyaw_ = combine_yaw(DGPStheta_, GTheta_, GPStheta, R3_, R4_);
          offsetyaw_ = calculate_offset(combineyaw_, GTheta_, GPStheta);
        }

        robot_yaw_ = GTheta_ + offsetyaw_;
        if (robot_yaw_ < -M_PI) robot_yaw_ += 2.0 * M_PI;
        if (robot_yaw_ >  M_PI) robot_yaw_ -= 2.0 * M_PI;

        auto orient = yaw_to_orientation(robot_yaw_);
        robot_orientationz_ = orient.first; robot_orientationw_ = orient.second;

        fused_msg_.pose.pose.position.x = static_cast<double>(fused_value.x());
        fused_msg_.pose.pose.position.y = static_cast<double>(fused_value.y());
        fused_msg_.pose.pose.orientation.z = robot_orientationz_;
        fused_msg_.pose.pose.orientation.w = robot_orientationw_;
      } else {
        Eigen::Vector2d fused_value = KalfXY(Speed_, SmpTime_, GTheta_, R1_, R2_);
        robot_yaw_ = GTheta_ + offsetyaw_;
        if (robot_yaw_ < -M_PI) robot_yaw_ += 2.0 * M_PI;
        if (robot_yaw_ >  M_PI) robot_yaw_ -= 2.0 * M_PI;
        auto orient = yaw_to_orientation(robot_yaw_);
        robot_orientationz_ = orient.first; robot_orientationw_ = orient.second;
        fused_msg_.pose.pose.position.x = fused_value.x();
        fused_msg_.pose.pose.position.y = fused_value.y();
        fused_msg_.pose.pose.orientation.z = robot_orientationz_;
        fused_msg_.pose.pose.orientation.w = robot_orientationw_;
      }

      fused_msg_.header.stamp = this->now();
      fused_msg_.header.frame_id = "odom";
      fused_pub_->publish(fused_msg_);

      Speed_ = std::numeric_limits<double>::quiet_NaN();
      SmpTime_ = std::numeric_limits<double>::quiet_NaN();
      GTheta_ = std::numeric_limits<double>::quiet_NaN();

      if (ekf_publish_TF_) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = fused_msg_.pose.pose.position.x;
        t.transform.translation.y = fused_msg_.pose.pose.position.y;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = robot_orientationz_;
        t.transform.rotation.w = robot_orientationw_;
        tf_broadcaster_->sendTransform(t);
      }

      // reset GPS flag but keep last GPS coords
      have_gps_ = false;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtendedKalmanFilter>());
  rclcpp::shutdown();
  return 0;
}

