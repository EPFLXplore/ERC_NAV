
// Extended Kalman Filter for Navigation
// Fusing Wheel Odometry, IMU (9-axis), and LiDAR-based odometry
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
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>


#define IDX_X    0
#define IDX_Y    1
#define IDX_YAW  2
#define IDX_VX   3
#define IDX_VY   4

// Odometry covariance indices
#define COV_XX   0
#define COV_YY   7
#define COV_YAW  35

class ExtendedKalmanFilter2D {
public:
  Eigen::Matrix<double,5,1> x;   // state vector [x, y, yaw, vx, vy]
  Eigen::Matrix<double,5,5> P;   // 5×5 covariance
  Eigen::Matrix<double,5,5> Q;   // 5x5 process noise
  double                    R_yaw;  // IMU yaw variance
  Eigen::Matrix2d           R_accel;// IMU accel covariance (ax, ay)
  Eigen::Matrix2d           R_xy;   // wheel-odom x,y variance
  Eigen::Matrix2d           R_xy_lidar;   // lidar x,y variance
  Eigen::Matrix2d           R_xy_aruco;   // aruco x,y variance
  Eigen::Matrix2d           R_xy_vio;     // vio x,y variance
  Eigen::Matrix3d           R_wheel_vel; // wheel velocity measurement noise



  ExtendedKalmanFilter2D() {
    x.setZero();
    P = Eigen::Matrix<double,5,5>::Identity() * 1e-2;
    Q.setZero();
    Q(IDX_X,IDX_X)     = 1e-4; // position is driven by wheel velocity, so keep direct drift small
    Q(IDX_Y,IDX_Y)     = 1e-4;
    Q(IDX_YAW,IDX_YAW) = 0.001; // process noise for yaw
    Q(IDX_VX,IDX_VX)   = 0.1; // process noise for vx : from wheel odometry
    Q(IDX_VY,IDX_VY)   = 0.2; // process noise for vy : from wheel odometry
    R_yaw   = 0.0001; // measurement noise for yaw
    R_accel = Eigen::Matrix2d::Identity() * 0.15;  // default, override per IMU msg
    R_xy    = Eigen::Matrix2d::Identity() * 0.08;
    R_xy_lidar    = Eigen::Matrix2d::Identity() * 0.01; // Var of 0.01 => std of 0.1: 10cm 
    R_xy_aruco    = Eigen::Matrix2d::Identity() * 0.01; // Var of 0.0025 => std of 0.05: 5cm 
    R_xy_vio      = Eigen::Matrix2d::Identity() * 0.01; // Var of 0.0025 => std of 0.05: 5cm
    R_wheel_vel   = Eigen::Matrix3d::Identity() * 0.1;  // wheel velocity measurement noise

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

  void predict(double dt, double gyro_z) {
    if (dt <= 0.0) return;
    double yaw    = x(IDX_YAW);
    double yaw_n  = normalize_angle(yaw + gyro_z * dt);
    double vx = x(IDX_VX), vy = x(IDX_VY);

    if(!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw)){
      return;
    }

    // Wheel odometry publishes body-frame twist. Rotate it once into odom.
    Eigen::Matrix2d R;
    R << std::cos(yaw), -std::sin(yaw),
         std::sin(yaw), std::cos(yaw);

    Eigen::Vector2d d_body(vx*dt, vy*dt);
    Eigen::Vector2d d_world = R * d_body;

    x(IDX_X)   += d_world.x();
    x(IDX_Y)   += d_world.y();
    x(IDX_YAW)  = yaw_n;
    x(IDX_VX) = vx;
    x(IDX_VY) = vy;

    Eigen::Matrix<double,5,5> F = Eigen::Matrix<double,5,5>::Identity();    
    F(IDX_X, IDX_VX) = std::cos(yaw)*dt;
    F(IDX_X, IDX_VY) = (-1.0)*std::sin(yaw)*dt;
    F(IDX_Y, IDX_VX) = std::sin(yaw)*dt;
    F(IDX_Y, IDX_VY) = std::cos(yaw)*dt;
    F(IDX_X, IDX_YAW) = -vx * dt * std::sin(yaw) - vy*std::cos(yaw)*dt;
    F(IDX_Y, IDX_YAW) = vx * dt * std::cos(yaw) - vy*std::sin(yaw)*dt;

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
    const Eigen::Matrix<double,5,5> I = Eigen::Matrix<double,5,5>::Identity();
    const Eigen::Matrix<double,5,5> IKH = I - K * H;
    P = IKH * P * IKH.transpose() + K * meas_var * K.transpose();
    P = 0.5 * (P + P.transpose());
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
    const Eigen::Matrix<double,5,5> I = Eigen::Matrix<double,5,5>::Identity();
    const Eigen::Matrix<double,5,5> IKH = I - K * H;
    P = IKH * P * IKH.transpose() + K * R_cov * K.transpose();
    P = 0.5 * (P + P.transpose());
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
    const Eigen::Matrix<double,5,5> I = Eigen::Matrix<double,5,5>::Identity();
    const Eigen::Matrix<double,5,5> IKH = I - K * H;
    P = IKH * P * IKH.transpose() + K * R_wheel_vel.topLeftCorner<2,2>() * K.transpose();
    P = 0.5 * (P + P.transpose());
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

    this->declare_parameter<bool>("include_aruco", false);
    include_aruco_ = this->get_parameter("include_aruco").as_bool();
    RCLCPP_INFO(this->get_logger(), "ArUco included in EKF ? : %s", include_aruco_ ? "True" : "False");

    this->declare_parameter<bool>("include_vio", false);
    include_vio_ = this->get_parameter("include_vio").as_bool();
    RCLCPP_INFO(this->get_logger(), "VIO included in EKF ? : %s", include_vio_ ? "True" : "False");


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


    auto sensor_qos = rclcpp::QoS{rclcpp::KeepLast{1}}.best_effort();
    auto pub_qos = rclcpp::QoS{rclcpp::KeepLast{1}}.reliable();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/olive/imu/id001/ahrs", sensor_qos,
      std::bind(&NavEKFNode::imu_callback, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_glim_repub", sensor_qos,
      std::bind(&NavEKFNode::lidar_callback, this, std::placeholders::_1));

    aruco_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aruco_rover_pos", sensor_qos,
      std::bind(&NavEKFNode::aruco_callback, this, std::placeholders::_1));

    vio_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/zed/zed_node/pose_with_covariance_restamped", sensor_qos,
      std::bind(&NavEKFNode::vio_callback, this, std::placeholders::_1));

    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", sensor_qos,
      std::bind(&NavEKFNode::wheel_odom_callback, this, std::placeholders::_1));

    ekf_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/fused_nav_ekf_odom", pub_qos);
    //needed for the ZED 2i for time sync
    ros_time_pub_ = create_publisher<builtin_interfaces::msg::Time>(
      "/NAV/ros_time", pub_qos);

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

  void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double vx_body = msg->twist.twist.linear.x;
    const double vy_body = msg->twist.twist.linear.y;
    if (!std::isfinite(vx_body) || !std::isfinite(vy_body)) {
      return;
    }

    ekf_->updateWheelVelocities(vx_body, vy_body);
    last_vel_ = vx_body;
  }


  void lidar_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if(!this->include_lidar_){
      return;
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

  void aruco_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if(!this->include_aruco_){
      return;
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
  
    Eigen::Matrix2d S = H * ekf_->P * H.transpose() + ekf_->R_xy_aruco;
    double maha2 = innov.transpose() * S.inverse() * innov;

    if (std::isfinite(maha2) && maha2 < 7.0) {
      ekf_->updatePosition(mx, my, ekf_->R_xy_aruco);
    } else {
      RCLCPP_WARN(this->get_logger(), "Reject ArUco odom: maha^2=%.2f", maha2);
    }

  }

  void vio_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if(!this->include_vio_){
      return;
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
  
    Eigen::Matrix2d S = H * ekf_->P * H.transpose() + ekf_->R_xy_vio;
    double maha2 = innov.transpose() * S.inverse() * innov;

    if (std::isfinite(maha2) && maha2 < 7.0) {
      ekf_->updatePosition(mx, my, ekf_->R_xy_vio);
    } else {
      RCLCPP_WARN(this->get_logger(), "Reject VIO odom: maha^2=%.2f", maha2);
    }

  }

  void timer_callback() {
    auto now = this->now();
    builtin_interfaces::msg::Time ros_time_msg;
    ros_time_msg.sec = static_cast<int32_t>(now.seconds());
    ros_time_msg.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    ros_time_pub_->publish(ros_time_msg);

    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt < 0.0 || dt > 1.0) return;

    ekf_->predict(dt, last_gyro_z_);
    
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

    out.twist.twist.linear.x = ekf_->x(IDX_VX);
    out.twist.twist.linear.y = ekf_->x(IDX_VY);
    out.twist.twist.angular.z = last_gyro_z_;
    out.twist.covariance[COV_XX] = ekf_->P(IDX_VX, IDX_VX);
    out.twist.covariance[COV_YY] = ekf_->P(IDX_VY, IDX_VY);
    out.twist.covariance[COV_YAW] = ekf_->R_yaw;

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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr aruco_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vio_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    ekf_pub_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr ros_time_pub_;
  rclcpp::TimerBase::SharedPtr                             timer_;
  rclcpp::Time                                             last_time_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr        bias_client_, zero_quat_client_, zero_pose_client_;
  double last_gyro_z_, last_vel_;
  bool initial_frame_set_ = false;
  double initial_yaw_ = 0.0;
  bool include_lidar_ = true;
  bool include_aruco_ = true;
  bool include_vio_   = true;
  Eigen::Vector2d initial_pos_{0.0, 0.0};

  // double last_accel_x = 0.0;
  // double last_accel_y = 0.0;
  rclcpp::Time last_imu_stamp_;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavEKFNode>());
  rclcpp::shutdown();
  return 0;
}