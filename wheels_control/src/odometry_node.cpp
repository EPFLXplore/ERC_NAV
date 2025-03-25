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

const double WHEEL_RADIUS = 0.1325; //measured experimentally
const double rpm_to_ms = (2*M_PI*WHEEL_RADIUS)/(60.0);
const double gear_ratio = 1.0/53;
const double steer_ang_scaling = 30.0/6075; //measured experimentally, increments = 65536 = 2^16
const double deg_to_rad = M_PI/(180.0);
const double ANGLE_THRESHOLD = 1.0 * deg_to_rad;
const double ROTATION_ANGLE_THRESHOLD = 25.0 * deg_to_rad;
const double SPEED_EPSILON = 0.02; // m/s


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

        auto now = this->get_clock()->now();
        double dt = (now - prev_time_).seconds();
        if (dt <= 0.0) dt = 1.0 / 100.0;

        double a_fl = wheel_angles_[0];
        double a_fr = wheel_angles_[1];
        double a_br = wheel_angles_[2];
        double a_bl = wheel_angles_[3];

        // RCLCPP_INFO(this->get_logger(), "front left ang: %f", wheel_angles_[0]);
        // RCLCPP_INFO(this->get_logger(), "front right ang: %f", wheel_angles_[1]);
        // RCLCPP_INFO(this->get_logger(), "back right ang: %f", wheel_angles_[2]);
        // RCLCPP_INFO(this->get_logger(), "back left ang: %f", wheel_angles_[3]);

        double v_fl = wheel_speeds_[0];
        double v_fr = wheel_speeds_[1];
        double v_br = wheel_speeds_[2];
        double v_bl = wheel_speeds_[3];

        //double avg_angle_front = (a_fl + a_fr) / 2.0;
        //double avg_angle_back  = (a_bl + a_br) / 2.0;
        double avg_abs_angle = (std::fabs(a_fl) + std::fabs(a_fr) + std::fabs(a_br) + std::fabs(a_bl)) / 4.0;
        double avg_speed = (v_fl + v_fr + v_br + v_bl) / 4.0;
        bool going_right = false;
        bool going_left = false;
        //RCLCPP_INFO(this->get_logger(), "avg abs angle %f", avg_abs_angle);

    
        double v_x = 0.0;
        double v_y = 0.0;
        double omega_z = 0.0;
        // Check for straight driving       
        bool all_angles_small = avg_abs_angle < ANGLE_THRESHOLD;
        bool all_angles_tiny = avg_abs_angle < (ANGLE_THRESHOLD*0.1);
        // Check for in-place rotation (symmetrical opposing angles, opposing speeds)
        bool possible_rotation = (std::abs(a_fl + a_bl) < ANGLE_THRESHOLD) &&
                                 (std::abs(a_fr + a_br) < ANGLE_THRESHOLD) &&
                                 (std::abs(v_fl + v_bl) < SPEED_EPSILON)   &&
                                 (std::abs(v_fr + v_br) < SPEED_EPSILON)   &&
                                 (avg_abs_angle > ROTATION_ANGLE_THRESHOLD);


        if (all_angles_small) {
	        //RCLCPP_INFO(this->get_logger(), "translation");
            // Straight motion
            double v_avg = (v_fl + v_fr + v_bl + v_br) / 4.0;
            v_x = v_avg;
            v_y = 0.0;
            omega_z = 0.0;
            //RCLCPP_INFO(this->get_logger(), "speed %f", v_avg);
        } else if (possible_rotation) {
            //RCLCPP_INFO(this->get_logger(), "rotation sur place");

            // In-place rotation
            double v_rot = (v_fr - v_fl + v_br - v_bl) / 4.0;
            // Estimate radius of rotation circle
            double r = std::sqrt((LENGTH / 2.0) * (LENGTH / 2.0) + (WIDTH / 2.0) * (WIDTH / 2.0));
            omega_z = v_rot / r;
            //RCLCPP_INFO(this->get_logger(), "omega z: %f", omega_z);

            v_x = 0.0;
            v_y = 0.0;
        } else {

            // Curved translation (double Ackermann)
            // Four cases: forwards curving right, forwards curving left, backwards curving left, backwards curving right
            double alpha_ext = a_fr;
            double alpha_int = -a_fl;
            double v_ext = v_fr;
            double v_int = v_fl;

            if(a_fl >= 0 && a_fr >= 0 && a_fr > a_fl &&
               a_bl <= 0 && a_br <= 0 && a_br < a_bl){
                //going forwards right or backwards right
                going_right = true;
                going_left=false;

                v_ext = (0.5) * (v_fl + v_bl);
                v_int = (0.5) * (v_fr + v_br);

                alpha_int = (0.5) * (a_fl - a_bl);
                alpha_ext = (0.5) * (a_fr - a_br);

            }else if(a_bl >= 0 && a_br >= 0 && a_br < a_bl &&
                a_fl <= 0 && a_fr <= 0 && a_fr > a_fl){
                //going forwards left or backwards left
                going_left = true;
                going_right=false;

                v_int = (0.5) * (v_fl + v_bl);
                v_ext = (0.5) * (v_fr + v_br);

                alpha_ext = (0.5) * (a_fl - a_bl);
                alpha_int = (0.5) * (a_fr - a_br);
            }

	        if(std::abs(alpha_ext)>0 && std::abs(alpha_int)>0){

                double r_ext = std::sqrt(std::pow(WIDTH / 2.0 + LENGTH / (2.0 * std::tan(alpha_ext)), 2) + std::pow(LENGTH / 2.0, 2));
                double r_int = std::sqrt(std::pow(WIDTH / 2.0 - LENGTH / (2.0 * std::tan(alpha_int)), 2) + std::pow(LENGTH / 2.0, 2));
                
                double R_geo = 0.25 * LENGTH * (1.0 / std::tan(alpha_ext) + 1.0 / std::tan(alpha_int));

                double omega_ext = v_ext / r_ext;
                double omega_int = v_int / r_int;

                omega_z = (omega_ext + omega_int) / 2.0;
                double R_vel = 0.0;

                if(std::abs(omega_z)<1e-6){
                    omega_z = 1e-6;
                    R_vel = (v_ext / omega_z + v_int / omega_z) / 2.0;
                }else{
                    R_vel = (v_ext / omega_z + v_int / omega_z) / 2.0;
                }
                    
                //double R = (R_geo + R_vel) / 2.0;

                v_x = omega_z * R_vel;


                v_y = 0.0;
	        }
        }

        // Transform velocities to world frame
        double world_vx = v_x * std::cos(pos_theta_) - v_y * std::sin(pos_theta_);
        double world_vy = v_x * std::sin(pos_theta_) + v_y * std::cos(pos_theta_);


        pos_x_ += world_vx * dt;
        pos_y_ += world_vy * dt;
        if(going_left){
            if(avg_speed>=0){
                pos_theta_ += dt * std::abs(omega_z);
            }else{
                pos_theta_ -= dt * std::abs(omega_z);
            }
        }else if(going_right){
            if(avg_speed>=0){
                pos_theta_ -= dt * std::abs(omega_z);
            }else{
                pos_theta_ += dt * std::abs(omega_z);
            }
        }


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
        odom.twist.twist.angular.z = omega_z;

        double yaw_uncertainty = 0.005 + 0.02*(abs(wheel_speeds_[0]) + abs(wheel_speeds_[1]) + abs(wheel_speeds_[2]) + abs(wheel_speeds_[3]))/4.0;
        
        std::array<double, 36> pose_covariance = {
            0.07, 0, 0, 0, 0, 0,
            0, 0.07, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, yaw_uncertainty
        };	

        std::array<double, 36> twist_covariance = {
                    0.07, 0, 0, 0, 0, 0,
                    0, 0.07, 0, 0, 0, 0,
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
