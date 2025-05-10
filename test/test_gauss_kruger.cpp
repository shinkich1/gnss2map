#define UNIT_TEST
#include "../src/GaussKruger.cpp"
#include <gtest/gtest.h>
#include <cmath>

class TestableGaussKruger : public gnss2map::GaussKruger {
public:
  TestableGaussKruger() : GaussKruger() {}
  using gnss2map::GaussKruger::gaussKruger;
};

TEST(GaussKrugerTest, ProjectionTest) {
  rclcpp::init(0, nullptr); // ROS 2の初期化

  auto node = std::make_shared<TestableGaussKruger>();

  double x, y;
  node->gaussKruger(35.0 * M_PI / 180.0, 135.0 * M_PI / 180.0, x, y);

  EXPECT_NEAR(x, 3885744.0, 1e-1); // 期待値は仮の値
  EXPECT_NEAR(y, 442860.0, 1e-1);  // 期待値は仮の値

  rclcpp::shutdown(); // ROS 2の終了
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}