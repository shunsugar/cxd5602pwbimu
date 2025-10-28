/*
ROS2 package: yaw_kalman
Single-file node: src/yaw_kalman_node.cpp
Build (CMakeLists.txt and package.xml snippets provided below this file).

Features:
 - Subscribes to IMU (sensor_msgs/msg/Imu) -> uses angular_velocity.z for predict
 - Subscribes to GNSS odometry (nav_msgs/msg/Odometry) -> uses pose.orientation to get GNSS yaw and pose covariance to get yaw variance (if available)
 - Outputs fused odometry on topic "fused_odom" (nav_msgs/msg/Odometry) with yaw fused into pose.orientation
 - Implements 2-state Kalman filter: x = [yaw, gyro_bias]
 - Parameters for Q, initial P, R (default), topic names

Notes:
 - Assumes IMU.angular_velocity is in rad/s. If your IMU publishes deg/s, convert to rad/s before calling predict (parameter available).
 - If GNSS odom does not provide pose covariance or yaw variance, node will use the configured parameter "gnss_yaw_variance".
 - GNSS speed-based gating is not implemented here (you requested first runnable program without adaptive R). You can extend later.

CMakeLists and package.xml examples are at the bottom of this file (as comments).
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <cmath>

using std::placeholders::_1;

static double wrapAngle(double a){
  return std::atan2(std::sin(a), std::cos(a));
}

class YawKalmanNode : public rclcpp::Node {
public:
  YawKalmanNode()
  : Node("yaw_kalman_node")
  {
    // parameters
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu");
    gnss_odom_topic_ = this->declare_parameter<std::string>("gnss_odom_topic", "/gnss_odom");
    fused_odom_topic_ = this->declare_parameter<std::string>("fused_odom_topic", "/fused_odom");
    imu_in_deg_ = this->declare_parameter<bool>("imu_in_deg", false);

    // process noise Q diag
    double q_psi = this->declare_parameter<double>("q_psi", 1e-4);
    double q_bias = this->declare_parameter<double>("q_bias", 1e-6);

    // initial covariance P
    double p_psi = this->declare_parameter<double>("init_p_psi", 0.5);
    double p_bias = this->declare_parameter<double>("init_p_bias", 1e-3);

    // default GNSS yaw variance (used if odom covariance not available)
    gnss_yaw_variance_ = this->declare_parameter<double>("gnss_yaw_variance", 0.05 * 0.05); // (rad)^2 ~ 3 deg

    // set up KF matrices
    x_.setZero(); // [yaw, bias]
    P_.setZero();
    P_(0,0) = p_psi;
    P_(1,1) = p_bias;
    Q_.setZero();
    Q_(0,0) = q_psi;
    Q_(1,1) = q_bias;

    // publisher
    fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(fused_odom_topic_, 10);

    // subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 50, std::bind(&YawKalmanNode::imuCallback, this, _1));

    gnss_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      gnss_odom_topic_, 10, std::bind(&YawKalmanNode::gnssOdomCallback, this, _1));

    R_ = gnss_yaw_variance_;

    RCLCPP_INFO(this->get_logger(), "Yaw Kalman node started. IMU topic: %s, GNSS odom topic: %s", imu_topic_.c_str(), gnss_odom_topic_.c_str());
  }

private:
  // KF state
  Eigen::Vector2d x_; // [yaw, bias]
  Eigen::Matrix2d P_;
  Eigen::Matrix2d Q_;
  double R_; // observation variance (scalar)

  // topics
  std::string imu_topic_, gnss_odom_topic_, fused_odom_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;

  // params
  bool imu_in_deg_;
  double gnss_yaw_variance_;

  // time
  rclcpp::Time last_imu_time_;
  bool got_first_imu_ = false;

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    // use angular_velocity.z as omega_meas
    double omega = msg->angular_velocity.z;
    if(imu_in_deg_) omega = omega * M_PI / 180.0;

    rclcpp::Time now = msg->header.stamp;
    if(!got_first_imu_){
      last_imu_time_ = now;
      got_first_imu_ = true;
      return;
    }

    double dt = (now - last_imu_time_).seconds();
    last_imu_time_ = now;
    if(dt <= 0) return;

    predict(omega, dt);

    // publish fused odometry using current state (pose x,y left unchanged; only yaw fused)
    publishFusedOdom(msg->header.stamp);
  }

  void gnssOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    // extract yaw from gnss pose orientation
    geometry_msgs::msg::Quaternion q = msg->pose.pose.orientation;
    tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);

    // try to extract yaw variance from covariance matrix (if provided)
    double yaw_var = std::numeric_limits<double>::quiet_NaN();
    // pose covariance is 6x6 row-major: index for yaw (rot z) is 35 (0-based)
    if(msg->pose.covariance.size() == 36){
      double cov_yaw = msg->pose.covariance[35];
      if(std::isfinite(cov_yaw) && cov_yaw > 0.0){
        yaw_var = cov_yaw;
      }
    }

    updateWithGnssYaw(yaw, yaw_var);

    // publish fused odom with GNSS position but fused yaw from state
    publishFusedOdom(msg->header.stamp, msg);
  }

  void predict(double omega_meas, double dt){
    // state prediction
    double psi = x_(0);
    double b = x_(1);

    double psi_pred = psi + (omega_meas - b) * dt;
    double b_pred = b; // bias modeled as random walk (no change)

    // Jacobian F
    Eigen::Matrix2d F;
    F << 1.0, -dt,
         0.0, 1.0;

    x_(0) = wrapAngle(psi_pred);
    x_(1) = b_pred;

    P_ = F * P_ * F.transpose() + Q_;
  }

  void updateWithGnssYaw(double gnss_yaw, double gnss_yaw_var){
    double R_use = R_;
    if(std::isfinite(gnss_yaw_var) && gnss_yaw_var > 0.0) R_use = gnss_yaw_var;

    // innovation y = wrap( z - psi_pred )
    double y = wrapAngle(gnss_yaw - x_(0));

    // H = [1 0]
    double S = P_(0,0) + R_use;
    if(S <= 0) S = 1e-9;

    Eigen::Vector2d K;
    K(0) = P_(0,0) / S;
    K(1) = P_(1,0) / S;

    x_ = x_ + K * y;
    x_(0) = wrapAngle(x_(0));

    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    Eigen::RowVector2d H; H << 1.0, 0.0;
    P_ = (I - K * H) * P_;
  }

  void publishFusedOdom(const rclcpp::Time &stamp, const nav_msgs::msg::Odometry::SharedPtr gnss_msg = nullptr){
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = (gnss_msg) ? gnss_msg->header.frame_id : std::string("map");
    out.child_frame_id = "base_link";

    // Position: if GNSS provided, use it; otherwise set zero
    if(gnss_msg){
      out.pose = gnss_msg->pose;
    } else {
      // no GNSS: keep zero position with large covariance in position
      out.pose.pose.position.x = 0.0;
      out.pose.pose.position.y = 0.0;
      out.pose.pose.position.z = 0.0;
      // leave covariance as default zeros
    }

    // set orientation from fused yaw and keep roll/pitch from GNSS if available
    double fused_yaw = x_(0);
    double roll = 0.0, pitch = 0.0;
    double yaw_tmp = 0.0;
    if(gnss_msg){
      geometry_msgs::msg::Quaternion q = gnss_msg->pose.pose.orientation;
      tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
      tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw_tmp);
    }

    tf2::Quaternion q_out;
    q_out.setRPY(roll, pitch, fused_yaw);
    q_out.normalize();
    out.pose.pose.orientation = tf2::toMsg(q_out);

    // copy covariance but adjust orientation covariance for fused yaw (set to P_(0,0) for yaw)
    if(gnss_msg && gnss_msg->pose.covariance.size() == 36){
      out.pose.covariance = gnss_msg->pose.covariance;
      // set yaw variance (index 35)
      out.pose.covariance[35] = P_(0,0);
      // set covariance cross-terms involving yaw conservatively to zero
      for(int i=0;i<36;i++){
        if(i%6 == 5 || i/6 == 5) {
          // keep as is except diagonal
        }
      }
    } else {
      // default small covariance
      out.pose.covariance.fill(0.0);
      out.pose.covariance[0] = 1e6; // big uncertainty in x
      out.pose.covariance[7] = 1e6; // big uncertainty in y
      out.pose.covariance[35] = P_(0,0); // yaw var
    }

    // child_frame velocity: leave empty or copy from GNSS
    if(gnss_msg){
      out.twist = gnss_msg->twist;
    }

    fused_odom_pub_->publish(out);
  }
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YawKalmanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/*
CMakeLists.txt (minimum) - put in package root

cmake_minimum_required(VERSION 3.5)
project(yaw_kalman)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(yaw_kalman_node src/yaw_kalman_node.cpp)
ament_target_dependencies(yaw_kalman_node rclcpp sensor_msgs nav_msgs tf2 tf2_geometry_msgs Eigen3)

install(TARGETS
  yaw_kalman_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

package.xml (minimum)

<?xml version="1.0"?>
<package format="2">
  <name>yaw_kalman</name>
  <version>0.0.1</version>
  <description>Yaw Kalman filter node</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>eigen3_cmake_module</depend>
</package>
*/

