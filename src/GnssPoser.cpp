// GnssPoser.cpp
// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 修正: 非推奨ヘッダーを置き換え
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gnss2map {

class GnssPoser : public rclcpp::Node {
public:
  GnssPoser()
  : Node("gnss_poser")
  {
    // Declare and get parameters
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("pub_rate", 10.0);
    declare_parameter<double>("calibration_yaw_offset", 0.0);
    get_parameter("frame_id", frame_id_);
    get_parameter("pub_rate", pub_rate_);
    get_parameter("calibration_yaw_offset", calibration_offset_);
    first_receive_ = false;
    initPubSub();
  }

  // 修正: loop()をpublicに移動
  void loop() {
    if (!first_receive_) return;
    // Prepare header
    robot_pose_.header.frame_id = frame_id_;
    robot_pose_.header.stamp = now();
    // Copy 2D position & covariance
    robot_pose_.pose.pose.position = incoming2d_.pose.pose.position;
    robot_pose_.pose.covariance.fill(0.0);
    robot_pose_.pose.covariance[0] = incoming2d_.pose.covariance[0];
    robot_pose_.pose.covariance[7] = incoming2d_.pose.covariance[7];
    // Convert EUN to ENU
    const auto &qin = incoming3d_.pose.pose.orientation;
    tf2::Quaternion qEun(qin.x, qin.y, qin.z, qin.w);
    tf2::Quaternion qEunToEnu; qEunToEnu.setRPY(M_PI/2.0, 0.0, 0.0);
    tf2::Quaternion qEnu = qEunToEnu * qEun;
    // Extract yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(qEnu).getRPY(roll, pitch, yaw);
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    // Apply calibration offset
    yaw += calibration_offset_;
    // Rebuild 2D quaternion
    tf2::Quaternion q2d; q2d.setRPY(0.0, 0.0, yaw);
    robot_pose_.pose.pose.orientation.x = q2d.x();
    robot_pose_.pose.pose.orientation.y = q2d.y();
    robot_pose_.pose.pose.orientation.z = q2d.z();
    robot_pose_.pose.pose.orientation.w = q2d.w();
    // Publish
    pub_robot_pose_->publish(robot_pose_);
  }

protected:
  void pose2dCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {
    incoming2d_ = *msg;
    first_receive_ = true;
  }

  void vpsPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {
    incoming3d_ = *msg;
  }

protected: // 修正: robot_pose_をprotectedに変更
  geometry_msgs::msg::PoseWithCovarianceStamped robot_pose_;

private:
  // Setup subscriptions & publishers
  void initPubSub() {
    sub_pose2d_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "vps/pose_2d", 10,
      std::bind(&GnssPoser::pose2dCallback, this, std::placeholders::_1)
    );
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "vps/pose", 10,
      std::bind(&GnssPoser::vpsPoseCallback, this, std::placeholders::_1)
    );
    pub_robot_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "robot_pose", 10
    );
  }

  // Parameters & state
  std::string frame_id_;
  double pub_rate_, calibration_offset_;
  bool first_receive_;
  geometry_msgs::msg::PoseWithCovarianceStamped incoming2d_, incoming3d_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose2d_, sub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_robot_pose_;
};

} // namespace gnss2map

#ifndef UNIT_TEST
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gnss2map::GnssPoser>();
  rclcpp::Rate rate(node->get_parameter("pub_rate").as_double());
  while (rclcpp::ok()) {
    node->loop(); // 修正: loop()がpublicになったためアクセス可能
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
#endif