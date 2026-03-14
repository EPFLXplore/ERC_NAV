/*
pkg:    wheels_commands
node:   NAV_motor_cmds
topics:
        publish:    /NAV/steering_servoing
        subscribe:  /NAV/motor_nav_status & /NAV/displacement
description:    
    - Check that all the motors are connected
    - Send the commands of speed or position to the motors
    - Motors steering: control in position
    - Motors driving: control in velocity
*/

#include <rclcpp/rclcpp.hpp>
#include "custom_msg/msg/motorcmds.hpp"
#include "custom_msg/msg/motor_status.hpp"
#include <array>
#include <cmath>
#include "wheels_control/definition.hpp"


#define ANGLE_TOLERANCE 0.2 // radians
const double incr_to_rad = 2*M_PI/(pow(2,STEERING_RESOLUTION_BITS));//increments = 2^(14)
const double ANGLE_TOLERANCE_INCR = ANGLE_TOLERANCE / incr_to_rad; // convert to increments

class MotorSteeringServoingNode : public rclcpp::Node
{    
public:
    MotorSteeringServoingNode() : Node("motor_steering_servoing")
    {

        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(1));
        qos_best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_best_effort.durability(rclcpp::DurabilityPolicy::Volatile);

        sub_target_steering_angles_ = this->create_subscription<custom_msg::msg::Motorcmds>(
            "/NAV/displacement", qos_best_effort,
            std::bind(&MotorSteeringServoingNode::set_target_steering_angles, this, std::placeholders::_1));
            
        sub_current_steering_angles_ = this->create_subscription<custom_msg::msg::MotorStatus>(
            "/NAV/motor_nav_status", qos_best_effort,
            std::bind(&MotorSteeringServoingNode::set_current_steering_angles, this, std::placeholders::_1));

        pub_motor_cmds_ = this->create_publisher<custom_msg::msg::Motorcmds>(
            "/NAV/steering_servoing", qos_best_effort);

        RCLCPP_INFO(this->get_logger(), "Motor steering servoing node initialized");
    }
    
private:
    void set_target_steering_angles(const custom_msg::msg::Motorcmds::SharedPtr msg)
    {
        target_steering_angles_ = msg->steer;
        future_motor_cmds_ = *msg;
        received_target_ = true;
        check_and_send_motor_commands();
    }
    
    void set_current_steering_angles(const custom_msg::msg::MotorStatus::SharedPtr msg)
    {
        current_steering_angles_ = msg->position;
        received_current_ = true;
        check_and_send_motor_commands();
    }
    
    void check_and_send_motor_commands()
    {
        if (!(received_target_ && received_current_)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for both target and current steering angles...");
            return; // Wait until both messages have arrived
        }

        bool all_within_tolerance = true;
        for (size_t i = 0; i < 4; ++i) {
            if (std::fabs(target_steering_angles_[i] - current_steering_angles_[i]) >= ANGLE_TOLERANCE_INCR) {
                all_within_tolerance = false;
                RCLCPP_INFO(this->get_logger(), "Motor %zu not within tolerance: target=%.2f, current=%.2f", 
                            i, target_steering_angles_[i], current_steering_angles_[i]);
                break;
            }
        }

        if (all_within_tolerance)
        {
            pub_motor_cmds_->publish(future_motor_cmds_);
        }
        else
        {
            auto zero_cmds = future_motor_cmds_;
            zero_cmds.drive = {0.0, 0.0, 0.0, 0.0}; // Stop driving until steering is aligned
            pub_motor_cmds_->publish(zero_cmds);
        }

        RCLCPP_INFO(this->get_logger(), "Target steering angles: [%.2f, %.2f, %.2f, %.2f]",
                    target_steering_angles_[0], target_steering_angles_[1],
                    target_steering_angles_[2], target_steering_angles_[3]);
    }

    // Subscriptions et Publishers
    rclcpp::Subscription<custom_msg::msg::Motorcmds>::SharedPtr sub_target_steering_angles_;
    rclcpp::Subscription<custom_msg::msg::MotorStatus>::SharedPtr sub_current_steering_angles_;
    rclcpp::Publisher<custom_msg::msg::Motorcmds>::SharedPtr pub_motor_cmds_;

    // État des moteurs
    std::array<double, 4> current_steering_angles_{};
    std::array<double, 4> target_steering_angles_{};
    custom_msg::msg::Motorcmds future_motor_cmds_;

    // Flags
    bool received_target_ = false;
    bool received_current_ = false;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorSteeringServoingNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Spinning Motor Steering Servoing Node...");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}