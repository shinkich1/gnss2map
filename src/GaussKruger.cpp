// SPDX-FileCopyrightText: 2023 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include "gnss2map/GaussKruger.hpp"
#include <cmath> // std::abs を使
#include <vector>
#include <array>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>


namespace gnss2map{
    // 四角形領域（対角線2点）を表す構造体
    struct RectArea {
        double x1, y1, x2, y2;
    };

    // VPS推定座標が誤った領域内なら外側の最近点に補正
    void correctVpsPosition(double& x, double& y) {
        static const std::vector<RectArea> error_areas = {
            {-1000, -1000, 27, 1183}, {-1000, -1000, 106, 88}, {-1000, -1000, 255, 27},
            {38, 147, 107, 93}, {114, 146, 128, 92}, {114, 146, 154, 137},
            {139, 130, 157, 88}, {157, 84, 111, 45}, {207, 133, 165, 71},
            {241, 39, 1277, 1183}, {-1000, 157, 1277, 1183}
        };

        // 誤った領域に入っていれば外側の最近点に補正
        for (const auto& area : error_areas) {
            double min_x = std::min(area.x1, area.x2);
            double max_x = std::max(area.x1, area.x2);
            double min_y = std::min(area.y1, area.y2);
            double max_y = std::max(area.y1, area.y2);
            if (min_x <= x && x <= max_x && min_y <= y && y <= max_y) {
                double new_x = (x - min_x < max_x - x) ? min_x : max_x;
                double new_y = (y - min_y < max_y - y) ? min_y : max_y;
                double diff_x = std::abs(x - new_x);
                double diff_y = std::abs(y - new_y);
                double min_val = std::min(diff_x, diff_y);
                double max_val = std::max(diff_x, diff_y);

                // 比が2以上なら移動量が小さい方だけ移動
                if (max_val / min_val >= 1.05) {
                    if (diff_x < diff_y) {
                        x = new_x; // xだけ移動
                        // yは変更しない
                    } else {
                        y = new_y; // yだけ移動
                        // xは変更しない
                    }
                } else {
                    // 両方移動
                    x = new_x;
                    y = new_y;
                }
            }
        }
    }

    GaussKruger::GaussKruger() : Node("gauss_kruger")
    {
        setParam();
        getParam();
        initPubSub();
        initVariable();
    }

    GaussKruger::~GaussKruger(){}

    void GaussKruger::setParam()
    {
        this->declare_parameter("p0", std::vector<double>(3, 0.0));
        this->declare_parameter("gnss0", std::vector<double>(3, 0.0));
        this->declare_parameter("p1", std::vector<double>(2, 0.0));
        this->declare_parameter("gnss1", std::vector<double>(2, 0.0));
        this->declare_parameter("a", 6378137.0);
        this->declare_parameter("F", 298.257222);
        this->declare_parameter("m0", 0.9999);
        this->declare_parameter("ignore_th_cov", 1000.0);
        // this->declare_parameter("range_limit", std::vector<double>(4, 0.0));
    }

    void GaussKruger::getParam()
    {
        this->get_parameter("p0", p0_);
        this->get_parameter("gnss0", gnss0_);
        this->get_parameter("p1", p1_);
        this->get_parameter("gnss1", gnss1_);
        this->get_parameter("a", a_);
        this->get_parameter("F", F_);
        this->get_parameter("m0", m0_);
        this->get_parameter("ignore_th_cov", ignore_th_cov_);
        // this->get_parameter("range_limit", range_limit_);
    }

    void GaussKruger::initPubSub()
    {
        // VPSトピックの購読
        sub_vps_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "vps/fix", 2, std::bind(&GaussKruger::cbVpsFix, this, std::placeholders::_1));

        sub_vps_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "vps/pose", 2, std::bind(&GaussKruger::cbVpsPose, this, std::placeholders::_1));

        // 出力トピックのパブリッシャー
        pub_gnss_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "gnss_pose_with_covariance", 2);
    }

    void GaussKruger::cbGnss(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
    {
        std::array<double, 9UL> cov = msg->position_covariance;
        RCLCPP_DEBUG(this->get_logger(), "cov (xx, yy): (%lf, %lf)", cov[0], cov[4]);
        double x, y, z = msg->altitude + offset_z_;
        double t = 0.0;

        // 条件なしで常にパブリッシュ
        double rad_phi = msg->latitude * M_PI / 180;
        double rad_lambda = msg->longitude * M_PI / 180;
        gaussKruger(rad_phi, rad_lambda, x, y);
        if(!calc_direction_){
            calc_direction_ = true;
        }else{
            t = calcDirection(x, y);
        }
        pre_x_ = x;
        pre_y_ = y;

        pubGnssPose(x, y, z, t, cov[0], cov[4], cov[8]);
    }

    // 追加: データ受信フラグ
    bool vps_fix_received_ = false;
    bool vps_pose_received_ = false;

    void GaussKruger::cbVpsFix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
    {
        double rad_phi = msg->latitude * M_PI / 180;
        double rad_lambda = msg->longitude * M_PI / 180;
        double x, y;

        gaussKruger(rad_phi, rad_lambda, x, y);
        double z = msg->altitude + offset_z_;

        // --- 座標補正 ---
        correctVpsPosition(x, y);

        current_position_ = {x, y, z};
        vps_fix_received_ = true;

        // 両方揃ったらパブリッシュ
        if (vps_pose_received_) {
            pubGnssPose(
                current_position_[0], current_position_[1], current_position_[2],
                current_orientation_,
                current_covariance_[0], current_covariance_[7], current_covariance_[14]
            );
            vps_fix_received_ = false;
            vps_pose_received_ = false;
        }
    }

    void GaussKruger::cbVpsPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        double map_yaw = yaw - rad_theta_offset_;
        while (map_yaw > M_PI) map_yaw -= 2 * M_PI;
        while (map_yaw < -M_PI) map_yaw += 2 * M_PI;

        current_orientation_ = map_yaw;
        current_covariance_ = msg->pose.covariance;

        // 最新の位置情報を使って毎回パブリッシュ
        pubGnssPose(
            current_position_[0], current_position_[1], current_position_[2],
            current_orientation_,
            current_covariance_[0], current_covariance_[7], current_covariance_[14]
        );
    }

    void GaussKruger::initVariable()
    {
        for(int i=0; i<2; ++i){
            gnss0_[i] *= M_PI/180;
            gnss1_[i] *= M_PI/180;
        }
        offset_z_ = p0_[2] - gnss0_[2];
        double n = 1 / (2 * F_ - 1);
        double n2 = pow(n, 2), n3 = pow(n, 3), n4 = pow(n, 4), n5 = pow(n, 5);
        alpha_[0] = n/2 - 2.*n2/3 + 5.*n3/16 + 41.*n4/180 - 127.*n5/288;
        alpha_[1] = 13.*n2/48 - 3.*n3/5 + 557.*n4/1440 + 281.*n5/630;
        alpha_[2] = 61.*n3/240 - 103.*n4/140 + 15061.*n5/26880;
        alpha_[3] = 49561.*n4/161280 - 179.*n5/168;
        alpha_[4] = 34729.*n5/80640;

        A_[0] = 1. + n2/4 + n4/64;
        A_[1] = -3./2*(n - n3/8 - n5/64);
        A_[2] = 15./16*(n2 - n4/4);
        A_[3] = -35./48*(n3 - 5.*n5/16);
        A_[4] = 315.*n4/512;
        A_[5] = -693.*n5/1280;

        A_bar_ = m0_ * a_ * A_[0]/(1 + n);

        S_bar_phi0_ = A_[0]*gnss0_[0];
        for(int i=1; i<=5; ++i) S_bar_phi0_ += A_[i]*sin(2*i*gnss0_[0]);
        S_bar_phi0_ *= m0_*a_ /(1+n);

        kt_ = 2*sqrt(n) / (1+n);

        K_ << 1., 0., 0., 1.;
        rad_theta_offset_ = 0.0;
        R_ = Eigen::Rotation2Dd(rad_theta_offset_);

        double x0, y0, x1, y1;
        gaussKruger(gnss0_[0], gnss0_[1], x0, y0);
        gaussKruger(gnss1_[0], gnss1_[1], x1, y1);
        rad_theta_offset_ = atan2(y1-y0, x1-x0) - atan2(p1_[1]-p0_[1], p1_[0]-p0_[0]);
        R_ = Eigen::Rotation2Dd(rad_theta_offset_);

        gaussKruger(gnss1_[0], gnss1_[1], x1, y1);
        K_ << p1_[0] / x1, 0., 0., p1_[1] / y1;
        RCLCPP_INFO(this->get_logger(), "kx: %lf, ky: %lf, theta: %lf", K_(0, 0), K_(1, 1), R_.angle());

        vel_to_dir_ = 0.;
    }

    void GaussKruger::gaussKruger(double rad_phi, double rad_lambda, double &x, double &y)
    {
        double t = sinh(atanh(sin(rad_phi)) - kt_*atanh(kt_*sin(rad_phi)));
        double t_bar = sqrt(1+pow(t, 2));
        double diff_lambda = rad_lambda - gnss0_[1];
        double lambda_cos = cos(diff_lambda), lambda_sin = sin(diff_lambda);
        double zeta = atan2(t, lambda_cos), eta = atanh(lambda_sin / t_bar);
        Eigen::Vector2d p;
        p(0) = zeta;
        p(1) = eta;
        for(int i=1; i<=5; ++i){
            p(0) += alpha_[i-1] * sin(2*i*zeta) * cosh(2*i*eta);
            p(1) += alpha_[i-1] * cos(2*i*zeta) * sinh(2*i*eta);
        }
        p(0) = p(0) * A_bar_ - S_bar_phi0_;
        p(1) *= A_bar_;
        p = K_ * R_ * p;
        x = p(0) + p0_[0];
        y = -p(1) + p0_[1];
    }

    void GaussKruger::pubGnssPose(double x, double y, double z, double t, double dev_x, double dev_y, double dev_z)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;
        pose.pose.pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(0, 0, t);
        pose.pose.pose.orientation.x = q[0];
        pose.pose.pose.orientation.y = q[1];
        pose.pose.pose.orientation.z = q[2];
        pose.pose.pose.orientation.w = q[3];

        pose.pose.covariance[0] = dev_x;
        pose.pose.covariance[7] = dev_y;
        pose.pose.covariance[14] = dev_z;

        pub_gnss_pose_->publish(pose);
    }
	
	double GaussKruger::calcDirection(double cur_x, double cur_y)
	{
		double t = atan2(cur_y-pre_y_, cur_x-pre_x_);
		return t;
	}
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gnss2map::GaussKruger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
