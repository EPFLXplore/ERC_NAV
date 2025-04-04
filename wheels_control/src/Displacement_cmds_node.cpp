/*
pkg:    wheels_commands
node:   NAV_displacement_cmds
topics:
        publish:    /NAV/displacement
        subscribe:  /ROVER/NAV_gamepad  - /NAV/absolute_encoders - /NAV/cmd_vel_final  -

description:  - Take the rover velocity and compute the position of the steering and the velocity of the driving
              - published the motors commands every delta time (500ms)
              - there are minus in the motors commands in the publisher because right motors run in the opposite direction than left motors

*/

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include "wheels_control/utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "custom_msg/msg/motor_status.hpp"
#include "custom_msg/msg/motorcmds.hpp"
#include "custom_msg/msg/wheelstatus.hpp"

#include "wheels_control/definition.hpp"
#include "wheels_control/normal_kinematic_model.hpp"
#include "wheels_control/normal_kinematic_model_slow.hpp"
#include "wheels_control/lateral_kinematic_model.hpp"

using namespace std::chrono_literals;

motors_obj current_motors_cmds = {{""}, {0, 0, 0, 0}, {0, 0, 0, 0}};
motors_obj current_motors_position = {{0, 0, 0, 0}, {0, 0, 0, 0}};
float speed_rover = 0.7; // initial value inside the kinematics model

int wheels_angle_for_rotation = 0; //  internal encoder = 2900000/8 = 362 500 unit: increment
int wheels_angle_for_rotation_with_translation = 0;

//------------------------------------NODE DEFINITION---------------------------------------

_Float64 get_wheels_angle_inc_for_rotation()
{
  /*
  Calculating the angle of rotation from the tangent angle of the rectangle's circle structure of the rover
  tan(angle of rotation) = width/length
  angle of rotation = arctan(length/width)

  Given values for Kerby rover:
          width = 736 mm
          length = 833 mm
          prediction of angle of rotation = 48.52 degrees
  */

  _Float64 angle_of_rotation_radians = 0;
  _Float64 angle_of_rotation_increment = 0;

  angle_of_rotation_radians = atan(WIDTH / LENGTH);
  angle_of_rotation_increment = (angle_of_rotation_radians * (pow(2, TOUR_RESOLUTION_BITS))) / (2 * M_PI);

  return angle_of_rotation_increment;
}

class DisplacementCmds : public rclcpp::Node
{
public:
  DisplacementCmds() : Node("NAV_displacement_cmds"), count_(0)
  {
    bool motor_cmds;
    this->declare_parameter("motor_cmds", true);
    if (this->get_parameter("motor_cmds", motor_cmds))
    {
      RCLCPP_INFO(this->get_logger(), "Got motor_cmds: %s", motor_cmds ? "true" : "false");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get motor_cmds");
    }

    normalKinematicModel.motor_cmds = motor_cmds;

    pub_kinematic = this->create_publisher<custom_msg::msg::Motorcmds>("/NAV/displacement", 10);

    sub_topic_absolute_encoders = this->create_subscription<custom_msg::msg::MotorStatus>(
        "/NAV/motor_nav_status", 1, std::bind(&DisplacementCmds::callback_absolute_encoders, this, std::placeholders::_1));

    // Listens on the NAVCSInterface for the actual mode of the rover
    sub_state_system = this->create_subscription<std_msgs::msg::String>(
        "/NAV/NAV_mode", 1, std::bind(&DisplacementCmds::callback_state_mode, this, std::placeholders::_1));

    sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "/NAV/cmd_vel_final", 1, std::bind(&DisplacementCmds::callback_cmd_vel, this, std::placeholders::_1));
      
    // Listens to the gamepad topic of the CS to see if we are going crab mode to avoid the wheels going in crabe mode when homing
    sub_cs_gamepad = this->create_subscription<sensor_msgs::msg::Joy>(
      "/ROVER/NAV_gamepad", 10, std::bind(&DisplacementCmds::callback_gamepad, this, std::placeholders::_1));

    sub_speed_rover = this->create_subscription<std_msgs::msg::Float32>(
        "/ROVER/change_NAV_speed", 1, std::bind(&DisplacementCmds::callback_speed_rover, this, std::placeholders::_1));

    
    wheels_angle_for_rotation = get_wheels_angle_inc_for_rotation(); // unit: increment - value around 8 300
    wheels_angle_for_rotation_with_translation = (20 * (pow(2, 16))) / (360);

    current_rover_state = ROVER_MODE::OFF;
    
    // Init normal model. The lateral does not have an init function
    normalKinematicModel.init(current_motors_position, wheels_angle_for_rotation);
  }


private:
  bool go_left = false;
  bool go_right = false;
  bool crab_mode = false;

  void callback_gamepad(const sensor_msgs::msg::Joy::SharedPtr msg){
    if(msg->buttons[GP_BUTTON_JOYSTICK_LEFT] == 1){
      crab_mode = true;
    }else{
      crab_mode = false;
    }
  }

  /**
  * @brief Callback function for the state of the rover
  */
  void callback_state_mode(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "Auto")
    {
      current_rover_state = ROVER_MODE::AUTO;
    }
    else if (msg->data == "Ackermann")
    {
      // check different state
      current_rover_state = ROVER_MODE::ACKERMANN;
    }
    else if (msg->data == "Omni")
    {
      current_rover_state = ROVER_MODE::OMNI_DIRECTIONAL;
    }
    else if (msg->data == "Off")
    {
      current_rover_state = ROVER_MODE::OFF;
    }
  }

  void callback_speed_rover(const std_msgs::msg::Float32::SharedPtr msg)
  {
    speed_rover = msg->data;
  }

  void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    float r_z = msg->angular.z;
    float v_x = msg->linear.x;
    float v_y = msg->linear.y;

    /*Run the kinematics manager to compute the motion*/
    if (current_rover_state == ROVER_MODE::ACKERMANN || current_rover_state == ROVER_MODE::AUTO) {
      if(current_rover_state == ROVER_MODE::AUTO && v_x < 0){
        r_z = -r_z;
      }

      current_motors_cmds = normalKinematicModel.run(current_motors_position, v_x, v_y, r_z, speed_rover, crab_mode);
   
    } else if(current_rover_state == ROVER_MODE::OMNI_DIRECTIONAL) {
      current_motors_cmds = lateralKinematicModel.run(go_left, go_right);
    }

    send_kinematic_msg();
  }

  void send_kinematic_msg() const
  {
    auto message = custom_msg::msg::Motorcmds();

    message.drive = {
        current_motors_cmds.drive[0],
        -current_motors_cmds.drive[1], // node 2 driving motor mounted in reverse
        -current_motors_cmds.drive[2], // node3 driving motor mounted in reverse
        current_motors_cmds.drive[3]};

    message.steer = {
        current_motors_cmds.steer[0],
        -current_motors_cmds.steer[1], // steering motor mounted in reverse
        current_motors_cmds.steer[2],
        -current_motors_cmds.steer[3] // steering motor mounted in reverse
    };
    // RCLCPP_INFO(get_logger(), "steer0 sent: %f", current_motors_cmds.steer[0]);
    // RCLCPP_INFO(get_logger(), "steer1 sent: %f", -current_motors_cmds.steer[1]);
    // RCLCPP_INFO(get_logger(), "steer2 sent: %f", current_motors_cmds.steer[2]);
    // RCLCPP_INFO(get_logger(), "steer3 sent: %f", -current_motors_cmds.steer[3]);


    message.modedeplacement = current_rover_state;
    message.info = "to be removed";

    pub_kinematic->publish(message);
  }

  void callback_absolute_encoders(const custom_msg::msg::MotorStatus::SharedPtr msg)
  {
    current_motors_position.drive[FRONT_LEFT] = 0;
    current_motors_position.drive[FRONT_RIGHT] = 0;
    current_motors_position.drive[BACK_RIGHT] = 0;
    current_motors_position.drive[BACK_LEFT] = 0;

    current_motors_position.steer[FRONT_LEFT] = msg->position[FRONT_LEFT];
    current_motors_position.steer[FRONT_RIGHT] = msg->position[FRONT_RIGHT];
    current_motors_position.steer[BACK_RIGHT] = msg->position[BACK_RIGHT];
    current_motors_position.steer[BACK_LEFT] = msg->position[BACK_LEFT];
  }

  RoverNormalKinematicModel normalKinematicModel;
  RoverLateralKinematicModel lateralKinematicModel;

  rclcpp::Publisher<custom_msg::msg::Motorcmds>::SharedPtr pub_kinematic;
  size_t count_;
  ROVER_MODE current_rover_state;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_system;
  rclcpp::Subscription<custom_msg::msg::MotorStatus>::SharedPtr sub_topic_absolute_encoders;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_cs_gamepad;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_rover;
};

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisplacementCmds>());

  rclcpp::shutdown();

  return 0;
}
