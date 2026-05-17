#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <math.h>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "custom_msg/msg/motor_status.hpp"
#include "wheels_control/definition.hpp"
#include "builtin_interfaces/msg/time.hpp"


using std::placeholders::_1;

const double WHEEL_RADIUS = 0.12; //measured experimentally
const double rpm_to_ms = (2*M_PI*WHEEL_RADIUS)/(60.0);
const double gear_ratio = 1.0/53;
const double incr_to_rad = 2*M_PI/(pow(2,STEERING_RESOLUTION_BITS));//increments = 2^(14)
const double deg_to_rad = M_PI/(180.0);


//WARNING: this code assumes the wheels are properly homed before starting the node!

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
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast{10}).best_effort();
        subscription_ = this->create_subscription<custom_msg::msg::MotorStatus>(
            "/NAV/motor_nav_status", sub_qos,
            std::bind(&ForwardKinematicsNode::publish_odometry, this, _1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/olive/imu/id001/ahrs", 10,
        std::bind(&ForwardKinematicsNode::imu_callback, this, std::placeholders::_1));

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);

        time_publisher_ = this->create_publisher<builtin_interfaces::msg::Time>("/NAV/nav_time", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(300), 
                                         std::bind(&ForwardKinematicsNode::timer_callback, this)
                                        );

        RCLCPP_INFO(this->get_logger(), "Wheel Odometry Node Initialized.");
    }

private:

    void timer_callback(){
        builtin_interfaces::msg::Time time_msg = this->get_clock()->now();
        time_publisher_->publish(time_msg);
    }


    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        pos_theta_ = yaw;
    }


    void publish_odometry(const custom_msg::msg::MotorStatus::SharedPtr msg) {

        if (msg->velocity.size() != 4 || msg->position.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Invalid input data! Expecting 4 velocities and 4 positions.");
            return;
        }
        
        //max speed : 10 revolutions in 17.85s
        //rpm = (10/17.85) * 60 = 33rpm
        //m/s = rpm * 2*pi*r/60 = 0.471 m/s

        //the motor status custom msg msg->velocity does NOT take into account the gear ratio 1/53 of the wheels.
        //we must first multiply the value from the message by this gear ratio
        //then convert it to m/s. The non gear-ratio-corrected speed should produce 
        //a max value of 1800 rpm. This should result in 0.471 m/s with the gear ratio taken into account

        wheel_speeds_[0] = msg->velocity[0] * rpm_to_ms * gear_ratio;
        wheel_speeds_[1] = msg->velocity[1] * rpm_to_ms * gear_ratio * (-1.0); // wired backwards
        wheel_speeds_[2] = msg->velocity[2] * rpm_to_ms * gear_ratio * (-1.0); // wired backwards
        wheel_speeds_[3] = msg->velocity[3] * rpm_to_ms * gear_ratio;

        // RCLCPP_INFO(this->get_logger(), "front left m/S: %f", wheel_speeds_[0]);
        // RCLCPP_INFO(this->get_logger(), "front right m/S: %f", wheel_speeds_[1]);
        // RCLCPP_INFO(this->get_logger(), "back right m/s: %f", wheel_speeds_[2]);
        // RCLCPP_INFO(this->get_logger(), "back right m/s: %f", wheel_speeds_[3]);

        // MotorStatus position/velocity are published in wheel-corner order:
        // [front-left, front-right, back-right, back-left].

        wheel_angles_[0] = (msg->position[0] * incr_to_rad);// * deg_to_rad;
        wheel_angles_[1] = (msg->position[1] * incr_to_rad);// * deg_to_rad;
        wheel_angles_[2] = (msg->position[2] * incr_to_rad);// * deg_to_rad;
        wheel_angles_[3] = (msg->position[3] * incr_to_rad);// * deg_to_rad;

        //the wheels have slip rings, so one might as well have an angle between [-PI, PI]
        for(unsigned int i=0; i<4; i++){
            if(wheel_angles_[i] > M_PI){
                wheel_angles_[i] -= 2*M_PI;
            }else if (wheel_angles_[i] < -M_PI){
                wheel_angles_[i] +=2*M_PI;
            }
        }
        //check in motors_cmds_lifecycle, %f prints a double/float, %d prints an int
        // RCLCPP_INFO(this->get_logger(), "front left ang: %f", wheel_angles_[0] * (1/deg_to_rad));
        // RCLCPP_INFO(this->get_logger(), "front right ang: %f", wheel_angles_[1] * (1/deg_to_rad));
        // RCLCPP_INFO(this->get_logger(), "back right ang: %f", wheel_angles_[2] * (1/deg_to_rad));
        // RCLCPP_INFO(this->get_logger(), "back left ang: %f", wheel_angles_[3] * (1/deg_to_rad));

        // auto now = this->get_clock()->now();
        // double dt = (now - prev_time_).seconds();
        // if (dt <= 0.0) dt = 1.0 / 100.0;

        /////////////////////////////////// ICR Least Squares //////////////////////////////////

        static const Eigen::Vector2d wheel_positions[4] = {
            {  LENGTH/2.0,  WIDTH/2.0},   // 0 – front-left
            {  LENGTH/2.0, -WIDTH/2.0},   // 1 – front-right
            { -LENGTH/2.0, -WIDTH/2.0},   // 2 – back-right
            { -LENGTH/2.0,  WIDTH/2.0}    // 3 – back-left
        };

        /* Build A·twist = b   (twist = [v_x v_y ω]ᵀ   in  base_link frame) */
        Eigen::Matrix<double,8,3> A;
        Eigen::Matrix<double,8,1> b;
        for (int i = 0; i < 4; ++i){

            double sa  = std::sin(wheel_angles_[i]);
            double ca  = std::cos(wheel_angles_[i]);
            double xw  = wheel_positions[i][0];
            double yw  = wheel_positions[i][1];

            /* lateral (no-slip) row */
            A(2*i,0) = -sa;
            A(2*i,1) =  ca;
            A(2*i,2) =  sa*yw + ca*xw;
            b(2*i)   =  0.0;

            /* longitudinal speed row */
            A(2*i+1,0) =  ca;
            A(2*i+1,1) =  sa;
            A(2*i+1,2) = -ca*yw + sa*xw;
            b(2*i+1)   =  wheel_speeds_[i];
        }

        Eigen::Vector3d twist = A.colPivHouseholderQr().solve(b);
        double v_x     = twist(0);     // body-frame forward
        double v_y     = twist(1);     // body-frame left
        double omega_z = -twist(2);     // yaw rate

        auto now = this->get_clock()->now();
        double dt = (now - prev_time_).seconds();
        if (dt <= 0.0) dt = 1.0 / 100.0;

        double world_vx =  v_x*std::cos(pos_theta_) - v_y*std::sin(pos_theta_);
        double world_vy =  v_x*std::sin(pos_theta_) + v_y*std::cos(pos_theta_);

        pos_x_     += world_vx * dt;
        pos_y_     += world_vy * dt;

        /* keep θ in (-π, π] */
        pos_theta_ = std::atan2(std::sin(pos_theta_), std::cos(pos_theta_));

        prev_time_ = now;

        //////////////////////////////////////////////////////////////////////////////////



        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = pos_x_;
        odom.pose.pose.position.y = pos_y_;
        // odom.twist.twist.linear.x = world_vx;
        // odom.twist.twist.linear.y = world_vy;
        // odom.twist.twist.angular.z = omega_z;

        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.linear.y = v_y;
        odom.twist.twist.angular.z = omega_z;

        double yaw_uncertainty = 0.1 + 0.02*(abs(wheel_speeds_[0]) + abs(wheel_speeds_[1]) + abs(wheel_speeds_[2]) + abs(wheel_speeds_[3]))/4.0;
        //RCLCPP_INFO(this->get_logger(), "yaw cov: %f", yaw_uncertainty);


        std::array<double, 36> pose_covariance = {
            0.05, 0, 0, 0, 0, 0,
            0, 0.05, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, yaw_uncertainty
        };	

        std::array<double, 36> twist_covariance = {
                    0.03, 0, 0, 0, 0, 0,
                    0, 0.03, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, yaw_uncertainty
        };
        
        odom.pose.covariance = pose_covariance;
        odom.twist.covariance = twist_covariance;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pos_theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom_publisher_->publish(odom);

        prev_time_ = now;
    }

    rclcpp::Subscription<custom_msg::msg::MotorStatus>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;



    Eigen::VectorXd wheel_speeds_;
    Eigen::VectorXd wheel_angles_;
    double pos_x_, pos_y_, pos_theta_;
    rclcpp::Time prev_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto odom_node = std::make_shared<ForwardKinematicsNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(odom_node);
  RCLCPP_INFO(odom_node->get_logger(), "Spinning odometry with MultiThreadedExecutor");
  executor.spin();
  rclcpp::shutdown();
  return 0;
}