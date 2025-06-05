// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include "gnss2map/GnssPoser.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath> // M_PI を使用するために必要

namespace gnss2map
{

    GnssPoser::GnssPoser() : Node("gnss_poser")
    {
        declare_params();
        init_pub_sub();
    }

    GnssPoser::~GnssPoser(){}

    void GnssPoser::declare_params(void)
    {
        declare_parameter("frame_id", "map");
        get_parameter("frame_id", frame_id_);
        declare_parameter("pub_rate", 1.0);
        get_parameter("pub_rate", pub_rate_);
        ekf_first_receive_ = false;
    }

    void GnssPoser::init_pub_sub(void)
    {
        pub_gnss_ekf_pose_with_covariance_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gnss_ekf_pose_with_covariance", 2);
        pub_debug_orientation_ = create_publisher<geometry_msgs::msg::Vector3>("debug_orientation", 2); // デバッグ用トピック
        sub_gnss_pose_with_covariance_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "gnss_pose_with_covariance", 2, std::bind(&GnssPoser::gnss_pose_callback, this, std::placeholders::_1)
        );
        // ekf_pose_with_covariance関連は使わないのでコメントアウト
        // sub_ekf_pose_with_covariance_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //     "ekf_pose_with_covariance", 2, std::bind(&GnssPoser::ekf_pose_callback, this, std::placeholders::_1)
        // );
    }

    void GnssPoser::gnss_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
    {
        gnss_pose_with_covariance_ = *msg;
        if(!ekf_first_receive_) ekf_first_receive_ = true; // ここでフラグを立てる
    }

    // ekf_pose_with_covariance関連は使わないのでコメントアウト
    // void GnssPoser::ekf_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
    // {
    //     ekf_pose_with_covariance_ = *msg;
    // }

    double GnssPoser::get_pub_rate(){
        return pub_rate_;
    }

    void GnssPoser::loop(void)
    {
        if(!ekf_first_receive_) return;

        // gnss_pose_with_covarianceのデータをそのまま転記
        gnss_ekf_pose_with_covariance_ = gnss_pose_with_covariance_;
        gnss_ekf_pose_with_covariance_.header.frame_id = frame_id_;
        gnss_ekf_pose_with_covariance_.header.stamp = now();

        // debug_orientationの計算にはgnss_pose_with_covarianceの姿勢を使う
        tf2::Quaternion q(
            gnss_pose_with_covariance_.pose.pose.orientation.x,
            gnss_pose_with_covariance_.pose.pose.orientation.y,
            gnss_pose_with_covariance_.pose.pose.orientation.z,
            gnss_pose_with_covariance_.pose.pose.orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // ラジアンを度に変換
        roll = roll * 180.0 / M_PI;
        pitch = pitch * 180.0 / M_PI;
        yaw = yaw * 180.0 / M_PI;

        // デバッグ用トピックにパブリッシュ
        geometry_msgs::msg::Vector3 debug_orientation;
        debug_orientation.x = roll;
        debug_orientation.y = pitch;
        debug_orientation.z = yaw;
        pub_debug_orientation_->publish(debug_orientation);

        // gnss_ekf_pose_with_covarianceをパブリッシュ
        pub_gnss_ekf_pose_with_covariance_->publish(gnss_ekf_pose_with_covariance_);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gnss2map::GnssPoser>();
    rclcpp::Rate rate(node->get_pub_rate());
    while(rclcpp::ok()){
        node->loop();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}