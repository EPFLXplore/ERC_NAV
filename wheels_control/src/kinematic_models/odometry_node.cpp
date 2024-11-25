#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "custom_msg/msg/motornavstatus.hpp"
#include "definition.hpp"

using std::placeholders::_1;

class ForwardKinematicsNode : public rclcpp::Node {
public:
    ForwardKinematicsNode()
        : Node("forward_kinematics_node"),
          wheel_speeds_(Eigen::VectorXd::Zero(4)),
          wheel_angles_(Eigen::VectorXd::Zero(4)),
          pos_x_(0.0),
          pos_y_(0.0),
          pos_theta_(0.0) {
        prev_time_ = this->get_clock()->now();

        // Subscriber to wheel speeds and angles
        subscription_ = this->create_subscription<custom_msg::msg::MotorNavStatus>(
            "/NAV/motor_nav_status", 10,
            std::bind(&ForwardKinematicsNode::publish_odometry, this, _1));

        // Publisher for odometry
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);

        RCLCPP_INFO(this->get_logger(), "Forward Kinematics Node Initialized.");
    }

private:
    void publish_odometry(const custom_msg::msg::MotorNavStatus::SharedPtr msg) {
        if (msg->velocity.size() != 4 || msg->position.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Invalid input data! Expecting 4 velocities and 4 positions.");
            return;
        }

        for (int i = 0; i < 4; ++i) {
            wheel_speeds_[i] = msg->velocity[i];
            wheel_angles_[i] = msg->position[i];
        }

        constexpr double d1 = 0.12, d2 = 0.15;
        const Eigen::Vector2d wheel_positions[4] = {
            {-LENGTH / 2 - d2, WIDTH / 2 + d1},
            {-LENGTH / 2 - d2, -WIDTH / 2 - d1},
            {LENGTH / 2 + d2, WIDTH / 2 + d1},
            {LENGTH / 2 + d2, -WIDTH / 2 - d1}
        };

        Eigen::MatrixXd A(8, 3);
        Eigen::VectorXd b(8);
        b.setZero();

        for (int i = 0; i < 4; ++i) {
            double cos_alpha = std::cos(wheel_angles_[i]);
            double sin_alpha = std::sin(wheel_angles_[i]);
            double x = wheel_positions wheel_positions ;

        A.row(2 * i) << cos_alpha, sin_alpha, -y * cos_alpha + x * sin_alpha;
            A.row(2 * i + 1) << -sin_alpha, cos_alpha, y * sin_alpha + x * cos_alpha;

            b(2 * i) = wheel_speeds_[i];
        }

        auto now = this->get_clock()->now();
        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

        double dt = (now - prev_time_).seconds();
        if (dt <= 0.0) dt = 1.0 / 100.0;

        Eigen::Vector3d k1, k2, k3, k4;

        k1 << x[0] * std::cos(pos_theta_) - x[1] * std::sin(pos_theta_),
              x[0] * std::sin(pos_theta_) + x[1] * std::cos(pos_theta_),
              x[2];
        double k2_theta = pos_theta_ + dt / 2.0 * k1[2];
        k2 << x[0] * std::cos(k2_theta) - x[1] * std::sin(k2_theta),
              x[0] * std::sin(k2_theta) + x[1] * std::cos(k2_theta),
              x[2];
        double k3_theta = pos_theta_ + dt / 2.0 * k2[2];
        k3 << x[0] * std::cos(k3_theta) - x[1] * std::sin(k3_theta),
              x[0] * std::sin(k3_theta) + x[1] * std::cos(k3_theta),
              x[2];
        double k4_theta = pos_theta_ + dt * k3[2];
        k4 << x[0] * std::cos(k4_theta) - x[1] * std::sin(k4_theta),
              x[0] * std::sin(k4_theta) + x[1] * std::cos(k4_theta),
              x[2];

        pos_x_ += dt / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
        pos_y_ += dt / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
        pos_theta_ += dt * x[2];

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = pos_x_;
        odom.pose.pose.position.y = pos_y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pos_theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom_publisher_->publish(odom);

        prev_time_ = now;
    }

    rclcpp::Subscription<custom_msg::msg::MotorNavStatus>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    Eigen::VectorXd wheel_speeds_;
    Eigen::VectorXd wheel_angles_;
    double pos_x_, pos_y_, pos_theta_;
    rclcpp::Time prev_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
