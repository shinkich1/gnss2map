#define UNIT_TEST
#include "../src/GnssPoser.cpp"
#include <gtest/gtest.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class TestableGnssPoser : public gnss2map::GnssPoser {
public:
  TestableGnssPoser() : GnssPoser() {}
  using gnss2map::GnssPoser::pose2dCallback;
  using gnss2map::GnssPoser::vpsPoseCallback;
  using gnss2map::GnssPoser::loop;
  using gnss2map::GnssPoser::robot_pose_;
};

TEST(GnssPoserTest, CalibrationOffset45deg) {
  rclcpp::init(0, nullptr); // ROS 2の初期化

  auto node = std::make_shared<TestableGnssPoser>();
  node->set_parameter(rclcpp::Parameter("calibration_yaw_offset", M_PI/4.0));

  geometry_msgs::msg::PoseWithCovarianceStamped p2d;
  p2d.pose.pose.position.x = 0.0;
  p2d.pose.pose.position.y = 0.0;
  p2d.pose.pose.position.z = 0.0;
  for (auto &c : p2d.pose.covariance) c = 0.0;
  p2d.pose.covariance[35] = 0.1;

  geometry_msgs::msg::PoseWithCovarianceStamped p3d;
  tf2::Quaternion q_identity;
  q_identity.setRPY(0.0, 0.0, 0.0);
  p3d.pose.pose.orientation = tf2::toMsg(q_identity);

  node->pose2dCallback(std::make_shared<decltype(p2d)>(p2d));
  node->vpsPoseCallback(std::make_shared<decltype(p3d)>(p3d));
  node->loop();

  auto out_q = node->robot_pose_.pose.pose.orientation;
  double expected_z = std::sin((M_PI/4.0)/2.0);
  double expected_w = std::cos((M_PI/4.0)/2.0);

  EXPECT_NEAR(out_q.x, 0.0, 1e-6);
  EXPECT_NEAR(out_q.y, 0.0, 1e-6);
  EXPECT_NEAR(out_q.z, expected_z, 1e-6);
  EXPECT_NEAR(out_q.w, expected_w, 1e-6);

  rclcpp::shutdown(); // ROS 2の終了
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}