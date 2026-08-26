/*
pkg:    wheels_commands
node:   NAV_motor_cmds
topics:
        publish:    /NAV/displacement
        subscribe:  /NAV/motor_nav_status & /NAV/steering_servoing

        #publish:    /NAV/steering_servoing
        #subscribe:  /NAV/motor_nav_status & /NAV/displacement
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
#include <chrono>
#include <cmath>
#include "wheels_control/definition.hpp"


class MotorSteeringServoingNode : public rclcpp::Node
{    
public:
    MotorSteeringServoingNode() : Node("motor_steering_servoing")
    {
        declare_parameter<double>("ANGLE_TOLERANCE", 0.5); // 0.5 rad = 28°
        angle_tolerance_ = get_parameter("ANGLE_TOLERANCE").as_double();
        // Commands are re-emitted on a fixed timer instead of once per /NAV/motor_nav_status.
        // Publishing from the status callback closed a loop through the CAN bus: every status
        // message triggered a command burst, which blocked NAV_motor_cmds' status timer on
        // can_mutex_, which slowed the status stream, which slowed /wheel_odom. Each command
        // message costs 8 blocking SDO writes, so this rate directly eats into the bus budget
        // that the status timer needs. Raise it only if /NAV/motor_nav_status keeps up.
        declare_parameter<double>("PUBLISH_RATE_HZ", 10.0);
        declare_parameter<double>("CMD_TIMEOUT_S", 0.5);
        publish_rate_hz_ = get_parameter("PUBLISH_RATE_HZ").as_double();
        if (publish_rate_hz_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(),
                "PUBLISH_RATE_HZ must be > 0, got %.3f, falling back to 10 Hz", publish_rate_hz_);
            publish_rate_hz_ = 10.0;
        }
        cmd_timeout_ = rclcpp::Duration::from_seconds(get_parameter("CMD_TIMEOUT_S").as_double());
        const double incr_to_rad = 2.0 * M_PI / std::pow(2.0, STEERING_RESOLUTION_BITS);
        angle_tolerance_incr_ = angle_tolerance_ / incr_to_rad;


        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(1));
        qos_best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_best_effort.durability(rclcpp::DurabilityPolicy::Volatile);

        sub_target_steering_angles_ = this->create_subscription<custom_msg::msg::Motorcmds>(
            "/NAV/steering_servoing", qos_best_effort,
            std::bind(&MotorSteeringServoingNode::set_target_steering_angles, this, std::placeholders::_1));
            
        sub_current_steering_angles_ = this->create_subscription<custom_msg::msg::MotorStatus>(
            "/NAV/motor_nav_status", qos_best_effort,
            std::bind(&MotorSteeringServoingNode::set_current_steering_angles, this, std::placeholders::_1));

        pub_motor_cmds_ = this->create_publisher<custom_msg::msg::Motorcmds>(
            "/NAV/displacement", qos_best_effort);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_hz_),
            std::bind(&MotorSteeringServoingNode::check_and_send_motor_commands, this));

        RCLCPP_INFO(this->get_logger(), "Motor steering servoing node initialized (%.1f Hz)",
                    publish_rate_hz_);
    }
    
private:
    // Both callbacks only cache. The actual publish is driven by timer_ so that the
    // command rate is decoupled from the status rate.
    void set_target_steering_angles(const custom_msg::msg::Motorcmds::SharedPtr msg)
    {
        target_steering_angles_ = msg->steer;
        future_motor_cmds_ = *msg;
        received_target_ = true;
        last_target_time_ = this->now();
    }
    
    void set_current_steering_angles(const custom_msg::msg::MotorStatus::SharedPtr msg)
    {
        current_steering_angles_ = msg->position;
        received_current_ = true;
    }
    
    void check_and_send_motor_commands()
    {
        if (!(received_target_ && received_current_)) {
            // RCLCPP_INFO(this->get_logger(), "Waiting for both target and current steering angles...");
            return; // Wait until both messages have arrived
        }

        // Publishing from a timer means a dead upstream no longer stops the commands on its
        // own, so hold the drive at zero when the last target has gone stale.
        if ((this->now() - last_target_time_) > cmd_timeout_) {
            auto stale_cmds = future_motor_cmds_;
            stale_cmds.drive = {0.0, 0.0, 0.0, 0.0};
            pub_motor_cmds_->publish(stale_cmds);
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            //     "No /NAV/steering_servoing target for more than %.2f s, holding drive at zero",
            //     cmd_timeout_.seconds());
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "angle tolerance incr %.2f",angle_tolerance_incr_);

        bool all_within_tolerance = true;
        for (size_t i = 0; i < 4; ++i) {
            // RCLCPP_INFO(this->get_logger(), "target steering angle %.2f",target_steering_angles_[i]);
            // RCLCPP_INFO(this->get_logger(), "current steering angle %.2f",current_steering_angles_[i]);
            if (std::fabs(target_steering_angles_[i] - current_steering_angles_[i]) >= angle_tolerance_incr_) {
                all_within_tolerance = false;
                // RCLCPP_INFO(this->get_logger(), "Motor %zu not within tolerance: target=%.2f, current=%.2f", 
                //             i, target_steering_angles_[i], current_steering_angles_[i]);
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

        // RCLCPP_INFO(this->get_logger(), "Target steering angles: [%.2f, %.2f, %.2f, %.2f]",
        //             target_steering_angles_[0], target_steering_angles_[1],
        //             target_steering_angles_[2], target_steering_angles_[3]);
    }

    // Subscriptions et Publishers
    rclcpp::Subscription<custom_msg::msg::Motorcmds>::SharedPtr sub_target_steering_angles_;
    rclcpp::Subscription<custom_msg::msg::MotorStatus>::SharedPtr sub_current_steering_angles_;
    rclcpp::Publisher<custom_msg::msg::Motorcmds>::SharedPtr pub_motor_cmds_;
    rclcpp::TimerBase::SharedPtr timer_;

    // État des moteurs
    std::array<double, 4> current_steering_angles_{};
    std::array<double, 4> target_steering_angles_{};
    custom_msg::msg::Motorcmds future_motor_cmds_;

    // Angle tolerance
    double angle_tolerance_incr_;
    double angle_tolerance_;

    // Command publication pacing
    double publish_rate_hz_;
    rclcpp::Duration cmd_timeout_{0, 0};
    rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};

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