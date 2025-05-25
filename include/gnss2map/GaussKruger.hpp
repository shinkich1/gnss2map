// SPDX-FileCopyrightText: 2023 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#ifndef GNSS2MAP__GAUSSKRUGER_HPP_
#define GNSS2MAP__GAUSSKRUGER_HPP_

#define NO_FIX -1

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace gnss2map
{
class GaussKruger : public rclcpp::Node
{
    public:
        GaussKruger();
        ~GaussKruger();

    private:
        // メンバー変数
        std::vector<double> p0_, gnss0_, p1_, gnss1_;
        double a_, F_;
        double m0_;
        double alpha_[5], A_[6];
        double A_bar_, S_bar_phi0_;
        double kt_;
        double ignore_th_cov_;
        double offset_z_;
        double rad_theta_offset_;
        double vel_to_dir_;
        bool calc_direction_;
        double pre_x_, pre_y_;

        Eigen::Matrix2d K_;
        Eigen::Rotation2Dd R_;

        // VPSトピックの購読用
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_vps_fix_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_vps_pose_;

        // 出力トピックのパブリッシャー
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gnss_pose_;

        // 現在の位置と方位
        std::array<double, 3> current_position_;
        double current_orientation_;
        std::array<double, 36> current_covariance_;

        // メンバー関数
        void initPubSub();
        void setParam();
        void getParam();
        void initVariable();
        void gaussKruger(double rad_phi, double rad_lambda, double &x, double &y);
        void pubGnssPose(double x, double y, double z, double t, double dev_x, double dev_y, double dev_z);
        double calcDirection(double cur_x, double cur_y);
        void cbVpsFix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg); // VPS Fixのコールバック
        void cbVpsPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg); // VPS Poseのコールバック
        void cbGnss(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg); // GNSS Fixのコールバック
};
}

#endif // GNSS2MAP__GAUSSKRUGER_HPP_
