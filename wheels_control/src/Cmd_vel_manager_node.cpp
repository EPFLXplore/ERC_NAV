/*
pkg:    wheels_commands
node:   NAV_cmdvel_manager
topics: 
        publish:    /NAV/cmd_vel_final
        subscribe:  /NAV/cmd_vel_manual - /cmd_vel
        
description:  
        ros2 parameters: autonomous_navigation

*/


#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include "wheels_control/utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "custom_msg/msg/motorcmds.hpp" 
#include "custom_msg/msg/wheelstatus.hpp"


#include "wheels_control/definition.hpp"


using namespace std::chrono_literals;


//bool autonomous_navigation = false;


//------------------------------------NODE DEFINITION---------------------------------------



class CmdvelManager : public rclcpp::Node
{

  public:
    CmdvelManager() : Node("NAV_cmd_vel_manager"), count_(0)
    {
      pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/NAV/cmd_vel_final", 10); 
      
      sub_cmd_vel_manual = this->create_subscription<geometry_msgs::msg::Twist>(
        "/NAV/cmd_vel_manual", 10, std::bind(&CmdvelManager::callback_cmd_vel_manual, this, std::placeholders::_1));

      sub_cmd_vel_auto = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CmdvelManager::callback_cmd_vel_auto, this, std::placeholders::_1));
    
      // Listens on the NAVCSInterface for the actual mode of the rover
      sub_state_system = this->create_subscription<std_msgs::msg::String>(
        "/NAV/NAV_mode", 1, std::bind(&CmdvelManager::callback_state_mode, this, std::placeholders::_1));
    

      current_rover_state = ROVER_MODE::OFF;
    }

  private:

    /**
    * @brief Callback function for the state of the rover
    */
    void callback_state_mode(const std_msgs::msg::String::SharedPtr msg)
    {
      if (msg->data == "Auto")
      {
        current_rover_state = ROVER_MODE::AUTO;
        //RCLCPP_INFO(this->get_logger(), "went auto mode");

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

    void callback_cmd_vel_manual(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      
      if (current_rover_state == ROVER_MODE::ACKERMANN || 
          current_rover_state == ROVER_MODE::OMNI_DIRECTIONAL)
      {

        auto message = geometry_msgs::msg::Twist(); 

        message.linear.x = msg->linear.x;
        message.linear.y = msg->linear.y;
        message.linear.z = 0;

        message.angular.x = 0;
        message.angular.y = 0;
        message.angular.z = msg->angular.z;
        
        pub_cmd_vel->publish(message);

      }
    }

    void callback_cmd_vel_auto(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "calback before");

      //RCLCPP_INFO(this->get_logger(), "before call, rover mode: %s", current_rover_state);

      if (current_rover_state == ROVER_MODE::AUTO)
      {
        //RCLCPP_INFO(this->get_logger(), "callback after");


        auto message = geometry_msgs::msg::Twist(); 

        message.linear.x = msg->linear.x;
        message.linear.y = msg->linear.y;
        message.linear.z = 0;

        message.angular.x = 0;
        message.angular.y = 0;
        message.angular.z = msg->angular.z;
        
        pub_cmd_vel->publish(message);

      }
    }

    ROVER_MODE current_rover_state;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;      

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_auto;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_manual;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_system;
    size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdvelManager>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Spinning cmd_vel_manager with MultiThreadedExecutor");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}