#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <math.h>
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
const double rpm_to_ms = (2*M_PI*WHEEL_RADIUS)/(60.0);
const double gear_ratio = 1.0/53;
const double steer_ang_scaling = 30.0/6075; //measured experimentally, increments = 65536 = 2^16
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

        subscription_ = this->create_subscription<custom_msg::msg::MotorStatus>(
            "/NAV/motor_nav_status", 10,
            std::bind(&ForwardKinematicsNode::publish_odometry, this, _1));

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);

        RCLCPP_INFO(this->get_logger(), "Wheel Odometry Node Initialized.");
    }

private:

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


        //in the definition.hpp file :
        // #define FRONT_LEFT_DRIVE 1
        // #define FRONT_RIGHT_DRIVE 2
        // #define BACK_RIGHT_DRIVE 3
        // #define BACK_LEFT_DRIVE 4

        // #define FRONT_LEFT_STEER 5  --> index 0 here
        // #define FRONT_RIGHT_STEER 6 --> index 1 here
        // #define BACK_RIGHT_STEER 7  --> index 2 here
        // #define BACK_LEFT_STEER 8   --> index 3 here

        wheel_angles_[0] = (msg->position[0] * steer_ang_scaling) * deg_to_rad;
        wheel_angles_[1] = (msg->position[1] * steer_ang_scaling) * deg_to_rad;
        wheel_angles_[2] = (msg->position[2] * steer_ang_scaling) * deg_to_rad;
        wheel_angles_[3] = (msg->position[3] * steer_ang_scaling) * deg_to_rad;

        //the wheels have slip rings, so one might as well have an angle between [-PI, PI]
        for(unsigned int i=0; i<4; i++){
            if(wheel_angles_[i] > M_PI){
                wheel_angles_[i] -= 2*M_PI;
            }else if (wheel_angles_[i] < -M_PI){
                wheel_angles_[i] +=2*M_PI;
            }
        }
        //check in motors_cmds_lifecycle, %f prints a double/float, %d prints an int
        //RCLCPP_INFO(this->get_logger(), "front left ang: %f", wheel_angles_[0] * (1/deg_to_rad));
        //RCLCPP_INFO(this->get_logger(), "front right ang: %f", wheel_angles_[1] * (1/deg_to_rad));
        //RCLCPP_INFO(this->get_logger(), "back right ang: %f", wheel_angles_[2] * (1/deg_to_rad));
        //RCLCPP_INFO(this->get_logger(), "back left ang: %f", wheel_angles_[3] * (1/deg_to_rad));


        constexpr double d1 = 0.0;
        constexpr double d2 = 0.0;
        const Eigen::Vector2d wheel_positions[4] = {
            {-WIDTH/2 - d1, LENGTH/2 + d2},       //index 0 --> front left
            {WIDTH/2 + d1, LENGTH/2 + d2},        //index 1 --> front right
            {WIDTH/2 + d1, -LENGTH/2 - d2},       //index 2 --> back right
            {-WIDTH/2 - d1, -LENGTH/2 - d2}       //index 3 --> back left
        };

        Eigen::MatrixXd A(8, 3);
        Eigen::VectorXd b(8);
        b.setZero();

        //dynamic slip factor : the more you turn the more you slip sideways
        //if you turn right you will slip and move a bit to the left
        //we want a 3 degree error at 30 degrees of steering angle
        double slip_factor = (6.0/30.0) * (0.25)*(std::fabs(wheel_angles_[0]) + std::fabs(wheel_angles_[1]) + std::fabs(wheel_angles_[2]) + std::fabs(wheel_angles_[3]));
        //RCLCPP_INFO(this->get_logger(), "slip factor %f", slip_factor);
        
        //check here if the +-1 signs are correct for the slip factor
        if(pos_theta_ >= 0.0){
            //if turning right
            slip_factor = (1.0)*std::fabs(slip_factor);
        }else{
            slip_factor = (-1.0)*std::fabs(slip_factor);
        }

        for (int i = 0; i < 4; ++i) {
            double cos_alpha = std::cos(wheel_angles_[i]);
            double sin_alpha = std::sin(wheel_angles_[i]);
            double x = wheel_positions[i][0];
            double y = wheel_positions[i][1];

            A.row(2 * i) << 1, 0, -y;
            A.row(2 * i + 1) << 0, 1, x;

            b(2 * i) = slip_factor * wheel_speeds_[i] * cos_alpha + (1-slip_factor)*wheel_speeds_[i] * sin_alpha;
            b(2 * i + 1) = (-1.0)*slip_factor*wheel_speeds_[i] * sin_alpha + (1-slip_factor)*wheel_speeds_[i] * cos_alpha;
        }


        auto now = this->get_clock()->now();
        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

        //RCLCPP_INFO(this->get_logger(), "state vx %f", x[0]);
        //RCLCPP_INFO(this->get_logger(), "state vy %f", x[1]);
        //RCLCPP_INFO(this->get_logger(), "state wz %f", x[2]);

        ////////////////////////////////////////////

        //Closed form double Ackermann forward kinematics

        //mean of the right side wheel steering angle in the reference of the front wheel
        //if we go right, the front right wheel [1] will have a +alpha_l angle whilst the back right wheel will have -alpha_l
        // double alpha_r = (wheel_angles_[1] - wheel_angles_[2])/2.0;
        // //if we go right the front left wheel will have a steering angle of +alpha_l whilst the back left wheel will have a steering angle of -alpha_l
        // double alpha_l = (wheel_angles_[0] - wheel_angles_[3])/2.0;

        // double v_l = (wheel_speeds_[0] + wheel_speeds_[3])/2.0;
        // double v_r = (wheel_speeds_[1] + wheel_speeds_[2])/2.0;

        // double R1 = 0.0;
        // double R2 = 0.0;
        // if(std::tan(alpha_l) == 0){
        //     R1 = 9999999;
        // }else{
        //     R1 = LENGTH/(2*std::tan(alpha_l)) - (WIDTH/2);
        // }

        // if(std::tan(alpha_r) == 0){
        //     R2 = 9999999;
        // }else{
        //     R2 = LENGTH/(2*std::tan(alpha_r)) + (WIDTH/2);

        // }

        // double R = (R1+R2)/2.0;
        // double r_left =  std::sqrt((R + WIDTH/2.0)*(R + WIDTH/2.0) + (LENGTH/2)*(LENGTH/2));
        // double r_right = std::sqrt((R - WIDTH/2.0)*(R - WIDTH/2.0) + (LENGTH/2)*(LENGTH/2));

        // double omega = ((v_l/r_left) + (v_r/r_right))/2.0;
        // double est_vx = omega * R;


        ///////////////////////////////////////////


        double dt = (now - prev_time_).seconds();
        if (dt <= 0.0) dt = 1.0 / 100.0;


        //we switch referentials from the math one to the ros2 one (x forward, y left, z up)
        double ros_vx = x[1];
        double ros_vy = -x[0];

        // Transform velocities to world frame
        double world_vx = ros_vx * std::cos(pos_theta_) - ros_vy * std::sin(pos_theta_);
        double world_vy = ros_vx * std::sin(pos_theta_) + ros_vy * std::cos(pos_theta_);


        pos_x_ += world_vx * dt;
        pos_y_ += world_vy * dt;
        pos_theta_ += dt * x[2] * fabs((0.35 + slip_factor*10));


        //normalize [-pi, pi]
        if (pos_theta_ > M_PI) {
            pos_theta_ -= 2 * M_PI;
        } else if (pos_theta_ < -M_PI) {
            pos_theta_ += 2 * M_PI;
        }

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = pos_x_;
        odom.pose.pose.position.y = pos_y_;
        odom.twist.twist.linear.x = world_vx;
        odom.twist.twist.linear.y = world_vy;
        odom.twist.twist.angular.z = x[2];

        double yaw_uncertainty = 0.05 + (fabs(slip_factor) * 0.9);
        
        std::array<double, 36> pose_covariance = {
            0.08, 0, 0, 0, 0, 0,
            0, 0.08, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, yaw_uncertainty
        };	

        std::array<double, 36> twist_covariance = {
                    0.08, 0, 0, 0, 0, 0,
                    0, 0.08, 0, 0, 0, 0,
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
