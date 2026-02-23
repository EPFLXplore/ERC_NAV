
// Extended Kalman Filter for Navigation
// Fusing Wheel Odometry, IMU (9-axis), (and LiDAR-based odometry (not for now))
// author: Arno Laurie


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>
#include "custom_msg/msg/motor_status.hpp"
#include "vector"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#define IDX_X    0
#define IDX_Y    1
#define IDX_YAW  2
#define IDX_VX   3
#define IDX_VY   4

// Odometry covariance indices
#define COV_XX   0
#define COV_YY   7
#define COV_YAW  35

#define STEERING_RESOLUTION_BITS 14
#define WIDTH 0.75
#define LENGTH 0.87
const double WHEEL_RADIUS = 0.12; //measured experimentally
const double rpm_to_ms = (2*M_PI*WHEEL_RADIUS)/(60.0);
const double gear_ratio = 1.0/53;
const double incr_to_rad = 2*M_PI/(pow(2,STEERING_RESOLUTION_BITS));//increments = 2^(14)
const double deg_to_rad = M_PI/(180.0);
const double ANGLE_THRESHOLD = 0.8 * deg_to_rad;
const double ROTATION_ANGLE_THRESHOLD = 0.642;//max mean ackerman angle is 36.6°=0.639
const double SPEED_EPSILON = 0.02; // m/s


class ExtendedKalmanFilter2D {
public:
  Eigen::Matrix<double,5,1> x;   // state vector [x, y, yaw, vx, vy]
  Eigen::Matrix<double,5,5> P;   // 5×5 covariance
  Eigen::Matrix<double,5,5> Q;   // 5x5 process noise
  double                    R_yaw;  // IMU yaw variance
  Eigen::Matrix2d           R_accel;// IMU accel covariance (ax, ay)
  Eigen::Matrix2d           R_xy;   // wheel-odom x,y variance
  Eigen::Matrix2d           R_xy_lidar;   // lidar x,y variance
  Eigen::Matrix3d R_wheel_vel; // wheel velocity measurement noise



  ExtendedKalmanFilter2D() {
    x.setZero();
    P = Eigen::Matrix<double,5,5>::Identity() * 1e-2;
    Q.setZero();
    Q(IDX_X,IDX_X)     = 0.01;
    Q(IDX_Y,IDX_Y)     = 0.01;
    Q(IDX_YAW,IDX_YAW) = 0.01;
    Q(IDX_VX,IDX_VX)   = 0.1;
    Q(IDX_VY,IDX_VY)   = 0.2;
    R_yaw   = 0.0001;
    R_accel = Eigen::Matrix2d::Identity() * 0.1;  // default, override per IMU msg
    R_xy    = Eigen::Matrix2d::Identity() * 0.08;
    R_xy_lidar    = Eigen::Matrix2d::Identity() * 0.01; // Var of 0.01 => std of 0.1: 10cm 
    R_wheel_vel = Eigen::Matrix3d::Identity() * 0.1;  // wheel velocity measurement noise

    prev_vx = 0.0;
    prev_vy = 0.0;
  }

  void initialize(double x0, double y0, double yaw0) {
    x << x0, y0, yaw0, 0.0, 0.0;
  }

  static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  std::vector<double> calculate_wheel_odom(double a_fr, double a_br, double a_fl, double a_bl, 
               double v_fr, double v_br, double v_fl, double v_bl, double gyro_z){

    double avg_abs_angle = (std::fabs(a_fl) + std::fabs(a_fr) + std::fabs(a_br) + std::fabs(a_bl)) / 4.0;
    double avg_speed = (v_fl + v_fr + v_br + v_bl) / 4.0;
    bool going_right = false;
    bool going_left = false;

    double v_x = 0.0;
    double v_y = 0.0;
    double omega_z = 0.0;
    // Check for straight driving       
    bool all_angles_small = avg_abs_angle < ANGLE_THRESHOLD;
    bool possible_rotation = (avg_abs_angle > ROTATION_ANGLE_THRESHOLD);

    if(all_angles_small){
    //RCLCPP_INFO(this->get_logger(), "translation");
      // Straight motion
      double v_avg = (v_fl + v_fr + v_bl + v_br) / 4.0;
      v_x = v_avg;
      v_y = 0.0;
      omega_z = 0.0;
      //RCLCPP_INFO(this->get_logger(), "speed %f", v_avg);
    }else if (possible_rotation){
      //RCLCPP_INFO(this->get_logger(), "rotation sur place");

      // In-place rotation
      double v_rot = (v_fr - v_fl + v_br - v_bl) / 4.0;
      
      // Estimate radius of rotation circle
      double r = std::sqrt((LENGTH / 2.0) * (LENGTH / 2.0) + (WIDTH / 2.0) * (WIDTH / 2.0));
      omega_z = v_rot / r;
      //RCLCPP_INFO(this->get_logger(), "omega z: %f", omega_z);
      //RCLCPP_INFO(this->get_logger(), "v_rot: %f", v_rot);

      v_x = 0.0;
      v_y = 0.0;
    }else{
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
            //RCLCPP_INFO(this->get_logger(), "ackerman");

            double r_ext = std::sqrt(std::pow(WIDTH / 2.0 + LENGTH / (2.0 * std::tan(alpha_ext)), 2) + std::pow(LENGTH / 2.0, 2));
            double r_int = std::sqrt(std::pow(WIDTH / 2.0 - LENGTH / (2.0 * std::tan(alpha_int)), 2) + std::pow(LENGTH / 2.0, 2));
            
            double R_geo = 0.25 * LENGTH * (1.0 / std::tan(alpha_ext) + 1.0 / std::tan(alpha_int));

            double omega_ext = v_ext / r_ext;
            double omega_int = v_int / r_int;

            omega_z = (omega_ext + omega_int) / 2.0;
            //RCLCPP_INFO(this->get_logger(), "omega z ackerman: %f", omega_z);

            double R_vel = 0.0;

            if(std::abs(omega_z)<1e-6){
                omega_z = 1e-6;
                R_vel = (v_ext / omega_z + v_int / omega_z) / 2.0;
            }else{
                R_vel = (v_ext / omega_z + v_int / omega_z) / 2.0;
            }
                
            double R = 0.5*(R_geo + R_vel);

            v_x = omega_z * R;
            v_y = 0.0;
      }
    }

    if(going_left){
        v_x = -v_x;
    }

    double world_vx = v_x * std::cos(x(IDX_YAW)) - v_y * std::sin(x(IDX_YAW));
    double world_vy = v_x * std::sin(x(IDX_YAW)) + v_y * std::cos(x(IDX_YAW));

    if(going_left && avg_speed>=0){
      omega_z = (-1.0)*omega_z;
    }

    if(going_left){
        if(avg_speed<0){
            omega_z = (-1.0)*omega_z;
        }
    }else if(going_right){
        if(avg_speed>=0){
            omega_z = (-1.0)*omega_z;
        }
    }

    std::vector<double>forward_kinematics = {world_vx, world_vy, omega_z};

    return forward_kinematics;
  }

  void predict(double dt, double gyro_z, 
               double steer_fr, double steer_br, double steer_fl, double steer_bl, 
               double vel_fr, double vel_br, double vel_fl, double vel_bl) {
    if (dt <= 0.0) return;
    double yaw    = x(IDX_YAW);
    double yaw_n  = normalize_angle(yaw + gyro_z * dt);
    double vx = x(IDX_VX), vy = x(IDX_VY);

    if(!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw)){
      return;
    }

    //calculate wheel odometry
    //double odom_vx = 0.0;
    //double odom_vy = 0.0;
    //double odom_omega = 0.0;
    std::vector<double> odom_output = calculate_wheel_odom(steer_fr, steer_br, steer_fl, steer_bl, vel_fr, vel_br, vel_fl, vel_bl, gyro_z);
    double odom_vx = odom_output[0];
    double odom_vy = odom_output[1];

    //rotate body-frame to odom frame
    Eigen::Matrix2d R;
    R << std::cos(yaw), -std::sin(yaw),
         std::sin(yaw), std::cos(yaw);

    Eigen::Vector2d d_body(odom_vx*dt, odom_vy*dt);

    //rotate to odom frame
    Eigen::Vector2d d_world = R * d_body;
    //Eigen::Vector2d a_world = R * Eigen::Vector2d(a_x, a_y);

    x(IDX_X)   += (d_world.x());// + 0.5*a_world.x()*dt*dt);
    x(IDX_Y)   += (d_world.y());// + 0.5*a_world.y()*dt*dt);
    x(IDX_YAW)  = yaw_n;
    // x(IDX_VX) += a_world.x() * dt;
    // x(IDX_VY) += a_world.y() * dt;
    x(IDX_VX) = vx;
    x(IDX_VY) = vy;

    Eigen::Matrix<double,5,5> F = Eigen::Matrix<double,5,5>::Identity();    
    F(IDX_X, IDX_VX) = std::cos(yaw)*dt;
    F(IDX_X, IDX_VY) = (-1.0)*std::sin(yaw)*dt;
    F(IDX_Y, IDX_VX) = std::sin(yaw)*dt;
    F(IDX_Y, IDX_VY) = (-1.0)*std::cos(yaw)*dt;
    F(IDX_X, IDX_YAW) = -vx * dt * std::sin(yaw) - vy*std::cos(yaw)*dt;
    F(IDX_Y, IDX_YAW) = vx * dt * std::cos(yaw) + vy*std::sin(yaw)*dt;

    Eigen::Matrix<double, 5, 5> Qdt = Q * dt;
    P = F * P * F.transpose() + Qdt;
  }

  void updateYaw(double meas_yaw, double meas_var) {
    Eigen::RowVector<double,5> H;
    H << 0, 0, 1, 0, 0;
    double innov = normalize_angle(meas_yaw - x(IDX_YAW));
    double S     = H * P * H.transpose() + meas_var;
    Eigen::Vector<double,5> K = P * H.transpose() / S;
    x += K * innov;
    x(IDX_YAW) = normalize_angle(x(IDX_YAW));
    P = (Eigen::Matrix<double,5,5>::Identity() - K * H) * P;
  }

  void updatePosition(double meas_x, double meas_y, Eigen::Matrix2d R_cov) {
    Eigen::Matrix<double,2,5> H = Eigen::Matrix<double,2,5>::Zero();
    H(0, IDX_X) = 1;
    H(1, IDX_Y) = 1;
    Eigen::Vector2d z, h;
    z << meas_x, meas_y;
    h << x(IDX_X), x(IDX_Y);
    Eigen::Vector2d innov = z - h;
    Eigen::Matrix2d S   = H * P * H.transpose() + R_cov;
    Eigen::Matrix<double,5,2> K = P * H.transpose() * S.inverse();
    x += K * innov;
    P = (Eigen::Matrix<double,5,5>::Identity() - K * H) * P;
  }

  void updateWheelVelocities(double vx_meas, double vy_meas) {
    Eigen::Vector2d z;
    z << vx_meas, vy_meas;

    Eigen::Vector2d h;
    h << x(IDX_VX), x(IDX_VY);

    Eigen::Matrix<double,2,5> H = Eigen::Matrix<double,2,5>::Zero();
    H(0, IDX_VX) = 1.0;
    H(1, IDX_VY) = 1.0;

    Eigen::Vector2d innov = z - h;
    Eigen::Matrix2d S = H * P * H.transpose() + R_wheel_vel.topLeftCorner<2,2>();
    Eigen::Matrix<double,5,2> K = P * H.transpose() * S.inverse();

    x += K * innov;
    P = (Eigen::Matrix<double,5,5>::Identity() - K * H) * P;
  }

  // void updateAccel(double ax, double ay, double dt){
  //   // We have measurements: a_x and a_y
  //   Eigen::Vector2d meas_a(ax, ay);
  //   // Measurement function: h(x) = [(vx - vx_prev)/dt; (vy - vy_prev)/dt]
  //   // So Jacobian rows are:
  //   // ∂a_x/∂x = 0, ∂a_x/∂y = 0, ∂a_x/∂ψ = 0,
  //   // ∂a_x/∂vx = 1/dt, ∂a_x/∂vy = 0
  //   Eigen::RowVector<double,5> H1 = Eigen::RowVector<double,5>::Zero();
  //   H1(3) = 1.0/dt;  // ∂h1/∂vx

  //   // ∂a_y/∂x = 0, ∂a_y/∂y = 0, ∂a_y/∂ψ = 0,
  //   // ∂a_y/∂vx = 0, ∂a_y/∂vy = 1/dt
  //   Eigen::RowVector<double,5> H2 = Eigen::RowVector<double,5>::Zero();
  //   H2(4) = 1.0/dt;  // ∂h2/∂vy

  //   Eigen::Vector2d pred_a((x(IDX_VX) - prev_vx)/dt,
  //                          (x(IDX_VY) - prev_vy)/dt);
  //   Eigen::Vector2d innov = meas_a - pred_a;

  //   // Innovation covariance
  //   Eigen::Matrix2d S;
  //   S(0,0) = H1 * P * H1.transpose() + R_accel(0,0);
  //   S(1,1) = H2 * P * H2.transpose() + R_accel(1,1);
  //   S(0,1) = S(1,0) = 0;

  //   // Kalman gains for both measurements
  //   Eigen::Matrix<double,5,2> K;
  //   K.col(0) = P * H1.transpose() / S(0,0);
  //   K.col(1) = P * H2.transpose() / S(1,1);

  //   // State update
  //   x   += K * innov;
  //   P   = (Eigen::Matrix<double,5,5>::Identity() - K * (Eigen::Matrix<double,2,5>() << H1, H2).finished()) * P;

  //   // Save velocities for next prediction
  //   prev_vx = x(IDX_VX);
  //   prev_vy = x(IDX_VY);
  // }
private:
  double prev_vx = 0.0;
  double prev_vy = 0.0;
  rclcpp::Time last_imu_stamp_;   // to compute IMU‐dt
};

class NavEKFNode : public rclcpp::Node {
public:
  NavEKFNode()
  : Node("nav_ekf_node")
  , last_gyro_z_(0.0)
  , last_vel_(0.0)
  {
    ekf_ = std::make_shared<ExtendedKalmanFilter2D>();
    ekf_->initialize(0.0, 0.0, 0.0);

    //dynamic param to include or not the LiDAR GLIM SLAM Output.

    this->declare_parameter<bool>("include_lidar", false);
    include_lidar_ = this->get_parameter("include_lidar").as_bool();
    RCLCPP_INFO(this->get_logger(), "Lidar included in EKF ? : %s", include_lidar_ ? "True" : "False");

    // IMU calibration services
    bias_client_      = create_client<std_srvs::srv::Trigger>("/olive/imu/id001/setBias");
    zero_quat_client_ = create_client<std_srvs::srv::Trigger>("/olive/imu/id001/setZeroQuaternion");
    zero_pose_client_ = create_client<std_srvs::srv::Trigger>("/olive/imu/id001/setZeroPose");
    wait_for_service(bias_client_);
    wait_for_service(zero_quat_client_);
    wait_for_service(zero_pose_client_);
    call_trigger(bias_client_,      "setBias");
    call_trigger(zero_quat_client_, "setZeroQuaternion");
    call_trigger(zero_pose_client_, "setZeroPose");


    auto qos = rclcpp::QoS{rclcpp::KeepLast{1}}.best_effort();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/olive/imu/id001/ahrs", qos,
      std::bind(&NavEKFNode::imu_callback, this, std::placeholders::_1));

    
    lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_glim_repub", qos,
      std::bind(&NavEKFNode::lidar_callback, this, std::placeholders::_1));


    wheel_info_sub_ = create_subscription<custom_msg::msg::MotorStatus>(
      "/NAV/motor_nav_status", qos,
      std::bind(&NavEKFNode::wheel_info_callback, this, std::placeholders::_1));
    

    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", qos,
      std::bind(&NavEKFNode::odom_callback, this, std::placeholders::_1));

      //publsher

    ekf_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/fused_nav_ekf_odom", qos);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&NavEKFNode::timer_callback, this));
  }

private:
  void wait_for_service(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client) {
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(get_logger(), "Service not available");
    }
  }

  void call_trigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
                    const std::string &name)
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), fut,
          std::chrono::seconds(5))
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto res = fut.get();
      RCLCPP_INFO(get_logger(), "Service '%s': %s", name.c_str(), res->message.c_str());
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time now(msg->header.stamp);
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if(!initial_frame_set_){
      initial_yaw_ = yaw;
      initial_pos_ = Eigen::Vector2d(ekf_->x(IDX_X), ekf_->x(IDX_Y));
      initial_frame_set_ = true;
    }

    double var = msg->orientation_covariance[8];
    if (var > 0.0) ekf_->R_yaw = var;
    ekf_->updateYaw(yaw, ekf_->R_yaw);
    last_gyro_z_ = msg->angular_velocity.z;

    // last_accel_x = msg->linear_acceleration.x;
    // last_accel_y = msg->linear_acceleration.y;
    double var_ax = msg->linear_acceleration_covariance[0];
    double var_ay = msg->linear_acceleration_covariance[4];
    ekf_->R_accel << var_ax, 0.0,
                   0.0,   var_ay;

    // compute IMU dt, falling back to 1/50s on the very first message
    double dt_imu = (last_imu_stamp_.nanoseconds() == 0) ? (1.0/50.0) : (now - last_imu_stamp_).seconds();
    last_imu_stamp_ = now;
    if (dt_imu <= 0.0 || !std::isfinite(dt_imu)){
      return;
    }
    //ekf_->updateAccel(msg->linear_acceleration.x, 0.0, dt_imu);

  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ekf_->updatePosition(msg->pose.pose.position.x,
                         msg->pose.pose.position.y, ekf_->R_xy);

    last_vel_ = msg->twist.twist.linear.x;
  }


  void lidar_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if(!this->include_lidar_){
      return;
    }

    const rclcpp::Time t_meas = msg->header.stamp;
    double dt = (t_meas - last_time_).seconds();

    if (std::isfinite(dt) && dt > 0.0 && dt < 1.0) {
      ekf_->predict(dt, last_gyro_z_,
                    steer_fr_, steer_br_, steer_fl_, steer_bl_,
                    vel_fr_,  vel_br_,  vel_fl_,  vel_bl_);
      last_time_ = t_meas;
    }
    double mx = msg->pose.pose.position.x;
    double my = msg->pose.pose.position.y;

    // Mahalanobis gating (2 DoF, ~95% -> 5.99; ~99% -> 9.21) to avoid shitty measurements from the LiDAR
    // used to measure how far a measurement is from the current EKF prediction
    Eigen::Vector2d z(mx, my);
    Eigen::Vector2d h(ekf_->x(IDX_X), ekf_->x(IDX_Y));

    Eigen::Matrix<double,2,5> H = Eigen::Matrix<double,2,5>::Zero();
    H(0, IDX_X) = 1.0;  H(1, IDX_Y) = 1.0;

    Eigen::Vector2d innov = z - h;
  
    Eigen::Matrix2d S = H * ekf_->P * H.transpose() + ekf_->R_xy_lidar;
    double maha2 = innov.transpose() * S.inverse() * innov;

    if (std::isfinite(maha2) && maha2 < 7.0) {
      ekf_->updatePosition(mx, my, ekf_->R_xy_lidar);
    } else {
      RCLCPP_WARN(this->get_logger(), "Reject LiDAR odom: maha^2=%.2f", maha2);
    }

  }

  void wheel_info_callback(const custom_msg::msg::MotorStatus::SharedPtr msg){
    if (msg->velocity.size() != 4 || msg->position.size() != 4) {
      RCLCPP_ERROR(this->get_logger(), "Invalid input data! Expecting 4 velocities and 4 positions.");
      return;
    }

        //in the definition.hpp file :
        // #define FRONT_LEFT_DRIVE 1
        // #define FRONT_RIGHT_DRIVE 2
        // #define BACK_RIGHT_DRIVE 3
        // #define BACK_LEFT_DRIVE 4

        // #define FRONT_LEFT_STEER 5  --> index 0 here
        // #define FRONT_RIGHT_STEER 6 --> index 1 here
        // #define BACK_RIGHT_STEER 7  --> index 2 here
        // #define BACK_LEFT_STEER 8   --> index 3 here
    std::vector<double> wheel_speeds_;
    std::vector<double> wheel_angles_;
    wheel_speeds_.reserve(4);
    wheel_speeds_.resize(4);

    wheel_angles_.reserve(4);
    wheel_angles_.reserve(4);



    wheel_speeds_[0] = msg->velocity[0] * rpm_to_ms * gear_ratio;
    wheel_speeds_[1] = msg->velocity[1] * rpm_to_ms * gear_ratio * (-1.0); // wired backwards
    wheel_speeds_[2] = msg->velocity[2] * rpm_to_ms * gear_ratio * (-1.0); // wired backwards
    wheel_speeds_[3] = msg->velocity[3] * rpm_to_ms * gear_ratio;
    wheel_angles_[0] = (msg->position[0] * incr_to_rad);// * deg_to_rad;
    wheel_angles_[1] = (msg->position[1] * incr_to_rad);// * deg_to_rad;
    wheel_angles_[2] = (msg->position[2] * incr_to_rad);// * deg_to_rad;
    wheel_angles_[3] = (msg->position[3] * incr_to_rad);// * deg_to_rad;

    for(unsigned int i=0; i<4; i++){
      if(wheel_angles_[i] > M_PI){
        wheel_angles_[i] -= 2*M_PI;
      }else if (wheel_angles_[i] < -M_PI){
        wheel_angles_[i] +=2*M_PI;
      }
    }

    steer_fl_ = wheel_angles_[0];
    steer_fr_ = wheel_angles_[1];
    steer_br_ = wheel_angles_[2];
    steer_bl_ = wheel_angles_[3];

    vel_fl_ = wheel_speeds_[0];
    vel_fr_ = wheel_speeds_[1];
    vel_br_ = wheel_speeds_[2];
    vel_bl_ = wheel_speeds_[3];
  }

  void timer_callback() {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt < 0.0 || dt > 1.0) return;

    // ekf_->predict(dt, last_gyro_z_, last_accel_x, last_accel_y);
    ekf_->predict(dt, last_gyro_z_, steer_fr_, steer_br_, steer_fl_, steer_bl_, vel_fr_, vel_br_, vel_fl_, vel_bl_);
    
    std::vector<double> odom = ekf_->calculate_wheel_odom(steer_fr_, steer_br_, steer_fl_, steer_bl_, vel_fr_, vel_br_, vel_fl_, vel_bl_, last_gyro_z_);
    double cos_yaw = std::cos(ekf_->x(IDX_YAW));
    double sin_yaw = std::sin(ekf_->x(IDX_YAW));
    double odom_vx_world = cos_yaw * odom[0] - sin_yaw*odom[1];
    double odom_vy_world = sin_yaw * odom[0] + cos_yaw*odom[1];
    ekf_->updateWheelVelocities(odom_vx_world, odom_vy_world);
    
    nav_msgs::msg::Odometry out;
    out.header.stamp    = now;
    out.header.frame_id = "odom";
    out.child_frame_id  = "base_link";

    out.pose.pose.position.x = ekf_->x(IDX_X);
    out.pose.pose.position.y = ekf_->x(IDX_Y);
    tf2::Quaternion qt;
    qt.setRPY(0, 0, ekf_->x(IDX_YAW));
    qt.normalize();
    out.pose.pose.orientation = tf2::toMsg(qt);

    out.pose.covariance[COV_XX]  = ekf_->P(IDX_X,IDX_X);
    out.pose.covariance[COV_YY]  = ekf_->P(IDX_Y,IDX_Y);
    out.pose.covariance[COV_YAW] = ekf_->P(IDX_YAW,IDX_YAW);

    ekf_pub_->publish(out);


    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = now;
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";

    transform_stamped.transform.translation.x = ekf_->x(IDX_X);
    transform_stamped.transform.translation.y = ekf_->x(IDX_Y);
    transform_stamped.transform.translation.z = 0.0;

    qt.setRPY(0, 0, ekf_->x(IDX_YAW));
    qt.normalize();
    transform_stamped.transform.rotation = tf2::toMsg(qt);

    tf_broadcaster_->sendTransform(transform_stamped);


  }

  std::shared_ptr<ExtendedKalmanFilter2D> ekf_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;
  rclcpp::Subscription<custom_msg::msg::MotorStatus>::SharedPtr wheel_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    ekf_pub_;
  rclcpp::TimerBase::SharedPtr                             timer_;
  rclcpp::Time                                             last_time_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr        bias_client_, zero_quat_client_, zero_pose_client_;
  double last_gyro_z_, last_vel_;
  bool initial_frame_set_ = false;
  double initial_yaw_ = 0.0;
  bool include_lidar_ = true;
  Eigen::Vector2d initial_pos_{0.0, 0.0};

  // double last_accel_x = 0.0;
  // double last_accel_y = 0.0;
  rclcpp::Time last_imu_stamp_;

  double steer_fr_, steer_br_, steer_fl_, steer_bl_, vel_fr_, vel_br_, vel_fl_, vel_bl_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavEKFNode>());
  rclcpp::shutdown();
  return 0;
}