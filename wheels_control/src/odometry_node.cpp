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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "custom_msg/msg/motor_status.hpp"
#include "wheels_control/definition.hpp"


using std::placeholders::_1;

const double WHEEL_RADIUS = 0.1325;

class ForwardKinematicsNode : public rclcpp::Node {
public:
    ForwardKinematicsNode()
        : Node("wheel_odometry_node"),
          wheel_speeds_(Eigen::VectorXd::Zero(4)),
          wheel_angles_(Eigen::VectorXd::Zero(4)),
          pos_x_(0.0),
          pos_y_(0.0),
          pos_theta_(0.0) {
        prev_time_ = this->get_clock()->now();

        subscription_ = this->create_subscription<custom_msg::msg::MotorStatus>(
            "/NAV/motor_nav_status", 10,
            std::bind(&ForwardKinematicsNode::publish_odometry, this, _1));

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);

        RCLCPP_INFO(this->get_logger(), "Wheel Odometry Node Initialized.");
    }

private:

    void publish_odometry(const custom_msg::msg::MotorStatus::SharedPtr msg) {
        //if(current_rover_state.rover_mode == OFF){
        //    RCLCPP_ERROR(this->get_logger(), "Can't publish wheel odom if rover is OFF");
        //    return;
        //}
        
        if (msg->velocity.size() != 4 || msg->position.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Invalid input data! Expecting 4 velocities and 4 positions.");
            return;
        }
        

            // in motors_cmds_node_lifecycle:
            // IDs [0,1,2,3] are the nodes for the driving
            // IDs [4,5,6,7] are the nodes for the steering

            // in definition.hpp:
            // #define FRONT_LEFT_DRIVE 1
            // #define FRONT_RIGHT_DRIVE 2
            // #define BACK_RIGHT_DRIVE 3
            // #define BACK_LEFT_DRIVE 4
            // #define FRONT_LEFT_STEER 5
            // #define FRONT_RIGHT_STEER 6
            // #define BACK_RIGHT_STEER 7
            // #define BACK_LEFT_STEER 8

            //wheel_speeds_[0] --> Front left drive = msg->velocity[0]
            //wheel_speeds_[1] --> front right drive
            //wheel_speeds_[2] --> Back right drive
            //wheel_speeds_[3] --> Back left drive
            
            //wheel_angles_[0] --> front left steering  = msg->position[4]
            //wheel_angles_[1] --> front right steering = msg->position[5]
            //wheel_angles_[2] --> back right steering = msg->position[6]
            //wheel_angles_[3] --> back left steering = msg->position[7]
        
        //rpm to m/s
        //2pi/60seconds

        //max speed : 10 revolutions in 17.85s
        //rpm = (10/17.85) * 60 = 33rpm
        //m/s = rpm * 2*pi*r/60 = 0.471 m/s
        
        const double rpm_to_ms = (2*M_PI*WHEEL_RADIUS)/(60.0);
        const double gear_ratio = 1.0/53;

        //msg->velocity does NOT take into account the gear ratio 1/53 of the wheels.
        //we must first multiply the value from the message by this gear ratio
        //then convert it to m/s. The non gear-ratio corrected speed should produce 
        //a max value of 1800 rpm. This should result in 0.471 m/s we the gear ratio taken into account

        wheel_speeds_[0] = msg->velocity[0] * rpm_to_ms * gear_ratio;
        wheel_speeds_[1] = msg->velocity[1]* rpm_to_ms * gear_ratio;
        wheel_speeds_[2] = msg->velocity[2]* rpm_to_ms * gear_ratio;
        wheel_speeds_[3] = msg->velocity[3]* rpm_to_ms * gear_ratio;
        
        wheel_angles_[0] = msg->position[4];
        wheel_angles_[1] = msg->position[5];
        wheel_angles_[2] = msg->position[6];
        wheel_angles_[3] = msg->position[7];

        //correct the -1 signs cuz some motors are wired backwards

        wheel_speeds_[1] = (-1.0)*wheel_speeds_[1];
        wheel_speeds_[2] = (-1.0)*wheel_speeds_[2];      

        RCLCPP_INFO(this->get_logger(), "corr[0]: %f", wheel_speeds_[0]);
        RCLCPP_INFO(this->get_logger(), "corr[1]: %f", wheel_speeds_[1]);
        RCLCPP_INFO(this->get_logger(), "corr[2]: %f", wheel_speeds_[2]);
        RCLCPP_INFO(this->get_logger(), "corr[3]: %f", wheel_speeds_[3]);


        constexpr double d1 = 0.0, d2 = 0.0;
        const Eigen::Vector2d wheel_positions[4] = {
            {-LENGTH / 2 - d2, WIDTH / 2 + d1},     //back left wheel
            {-LENGTH / 2 - d2, -WIDTH / 2 - d1},    //back right wheel
            {LENGTH / 2 + d2, WIDTH / 2 + d1},      //front left wheel
            {LENGTH / 2 + d2, -WIDTH / 2 - d1}      //front right wheel
        };

        Eigen::MatrixXd A(8, 3);
        Eigen::VectorXd b(8);
        b.setZero();

        for (int i = 0; i < 4; ++i) {
            double cos_alpha = std::cos(wheel_angles_[i]);
            double sin_alpha = std::sin(wheel_angles_[i]);
            double x = wheel_positions[i][0];
            double y = wheel_positions[i][1];

        A.row(2 * i) << cos_alpha, sin_alpha, -y * cos_alpha + x * sin_alpha;
            A.row(2 * i + 1) << -sin_alpha, cos_alpha, y * sin_alpha + x * cos_alpha;

            b(2 * i) = wheel_speeds_[i];
        }


        auto now = this->get_clock()->now();
        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

        double dt = (now - prev_time_).seconds();
        if (dt <= 0.0) dt = 1.0 / 100.0;

        // Eigen::Vector3d k1, k2, k3, k4;

        // k1 << x[0] * std::cos(pos_theta_) - x[1] * std::sin(pos_theta_),
        //       x[0] * std::sin(pos_theta_) + x[1] * std::cos(pos_theta_),
        //       x[2];
        // double k2_theta = pos_theta_ + dt / 2.0 * k1[2];
        // k2 << x[0] * std::cos(k2_theta) - x[1] * std::sin(k2_theta),
        //       x[0] * std::sin(k2_theta) + x[1] * std::cos(k2_theta),
        //       x[2];
        // double k3_theta = pos_theta_ + dt / 2.0 * k2[2];
        // k3 << x[0] * std::cos(k3_theta) - x[1] * std::sin(k3_theta),
        //       x[0] * std::sin(k3_theta) + x[1] * std::cos(k3_theta),
        //       x[2];
        // double k4_theta = pos_theta_ + dt * k3[2];
        // k4 << x[0] * std::cos(k4_theta) - x[1] * std::sin(k4_theta),
        //       x[0] * std::sin(k4_theta) + x[1] * std::cos(k4_theta),
        //       x[2];

        // pos_x_ += dt / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
        // pos_y_ += dt / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
        // pos_theta_ += dt * x[2];

        pos_x_ += dt * x[0];
        pos_y_ += dt * x[1];
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

    rclcpp::Subscription<custom_msg::msg::MotorStatus>::SharedPtr subscription_;
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
