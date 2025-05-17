
// Extended Kalman Filter for Navigation
// Fusing Wheel Odometry, IMU (9-axis), (and LiDAR-based odometry (not for now))
// author: Arno Laurie

//2D state vector: x, y, yaw

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>

// 2D EKF: state = [x, y, yaw]
#define IDX_X    0
#define IDX_Y    1
#define IDX_YAW  2

// Odometry covariance indices
#define COV_XX   0
#define COV_YY   7
#define COV_YAW  35

class ExtendedKalmanFilter2D {
public:
  Eigen::Vector3d x;      // [x, y, yaw]
  Eigen::Matrix3d P;      // covariance
  Eigen::Matrix3d Q;      // process noise
  double         R_yaw;   // IMU yaw variance
  Eigen::Matrix2d R_xy;   // wheel-odom x,y variance

  ExtendedKalmanFilter2D() {
    x.setZero();
    P = Eigen::Matrix3d::Identity() * 1e-2;
    Q.setZero();
    Q(0,0) = 0.01;
    Q(1,1) = 0.01;
    Q(2,2) = 0.001;
    R_yaw = 0.01;
    R_xy  = Eigen::Matrix2d::Identity() * 0.05;
  }

  void initialize(double x0, double y0, double yaw0) {
    x << x0, y0, yaw0;
  }

  static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  void predict(double dt, double gyro_z, double v) {
    if (dt <= 0.0) return;
    double yaw    = x(IDX_YAW);
    double yaw_n  = normalize_angle(yaw + gyro_z * dt);
    double dx     = v * dt * std::cos(yaw);
    double dy     = v * dt * std::sin(yaw);

    x(IDX_X)   += dx;
    x(IDX_Y)   += dy;
    x(IDX_YAW)  = yaw_n;

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(IDX_X,IDX_YAW) = -v * dt * std::sin(yaw);
    F(IDX_Y,IDX_YAW) =  v * dt * std::cos(yaw);

    P = F * P * F.transpose() + Q;
  }

  void updateYaw(double meas_yaw, double meas_var) {
    double y_pred = x(IDX_YAW);
    double innov  = normalize_angle(meas_yaw - y_pred);

    Eigen::RowVector3d H; H << 0, 0, 1;
    double S = H * P * H.transpose() + meas_var;
    Eigen::Vector3d K = P * H.transpose() / S;

    x += K * innov;
    x(IDX_YAW) = normalize_angle(x(IDX_YAW));
    P = (Eigen::Matrix3d::Identity() - K * H) * P;
  }

  void updatePosition(double meas_x, double meas_y) {
    Eigen::Vector2d z;    z << meas_x, meas_y;
    Eigen::Vector2d h;    h << x(IDX_X), x(IDX_Y);
    Eigen::Vector2d innov = z - h;

    Eigen::Matrix<double,2,3> H;
    H << 1,0,0,
         0,1,0;

    Eigen::Matrix2d S = H * P * H.transpose() + R_xy;
    Eigen::Matrix<double,3,2> K = P * H.transpose() * S.inverse();

    Eigen::Vector3d delta3 = K * innov;
    x(IDX_X) += delta3(0);
    x(IDX_Y) += delta3(1);
    // yaw unchanged

    P = (Eigen::Matrix3d::Identity() - K * H) * P;
  }
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

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/olive/imu/id001/ahrs", 50,
      std::bind(&NavEKFNode::imu_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", 50,
      std::bind(&NavEKFNode::odom_callback, this, std::placeholders::_1));

    ekf_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/fused_nav_ekf_odom", 10);

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
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double var = msg->orientation_covariance[8];
    if (var > 0.0) ekf_->R_yaw = var;
    ekf_->updateYaw(yaw, ekf_->R_yaw);
    last_gyro_z_ = msg->angular_velocity.z;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ekf_->updatePosition(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y);
    last_vel_ = msg->twist.twist.linear.x;
  }

  void timer_callback() {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    ekf_->predict(dt, last_gyro_z_, last_vel_);

    nav_msgs::msg::Odometry out;
    out.header.stamp    = now;
    out.header.frame_id = "odom";
    out.child_frame_id  = "base_link";

    out.pose.pose.position.x = ekf_->x(IDX_X);
    out.pose.pose.position.y = ekf_->x(IDX_Y);
    tf2::Quaternion qt;
    qt.setRPY(0, 0, ekf_->x(IDX_YAW));
    out.pose.pose.orientation = tf2::toMsg(qt);

    out.pose.covariance[COV_XX]  = ekf_->P(IDX_X,IDX_X);
    out.pose.covariance[COV_YY]  = ekf_->P(IDX_Y,IDX_Y);
    out.pose.covariance[COV_YAW] = ekf_->P(IDX_YAW,IDX_YAW);

    ekf_pub_->publish(out);
  }

  std::shared_ptr<ExtendedKalmanFilter2D> ekf_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    ekf_pub_;
  rclcpp::TimerBase::SharedPtr                             timer_;
  rclcpp::Time                                             last_time_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr        bias_client_, zero_quat_client_, zero_pose_client_;
  double last_gyro_z_, last_vel_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavEKFNode>());
  rclcpp::shutdown();
  return 0;
}
