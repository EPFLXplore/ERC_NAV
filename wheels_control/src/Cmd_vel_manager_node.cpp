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
      this->declare_parameter("autonomous_navigation", false);
      autonomous_navigation = this->get_parameter("autonomous_navigation").as_bool();
      autonomous_navigation = false;

      pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/NAV/cmd_vel_final", 10); 
      

      sub_cmd_vel_manual = this->create_subscription<geometry_msgs::msg::Twist>(
        "/NAV/cmd_vel_manual", 10, std::bind(&CmdvelManager::callback_cmd_vel_manual, this, std::placeholders::_1));

      sub_cmd_vel_auto = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CmdvelManager::callback_cmd_vel_auto, this, std::placeholders::_1));

      sub_nav_nodes_state = this->create_subscription<std_msgs::msg::String>(
        "NAV/nav_auto_state", 1, std::bind(&CmdvelManager::callback_nav_auto_state, this, std::placeholders::_1));

      sub_mode_nav = this->create_subscription<std_msgs::msg::String>(
        "/ROVER/NAV_mode", 1, std::bind(&CmdvelManager::callback_mode_nav, this, std::placeholders::_1));
    }


  private:

    void callback_mode_nav(std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "auto")   
        {
          autonomous_navigation = true;
          RCLCPP_INFO(get_logger(), "[cmd_vel_maganager] auto");
        }  
        else if (msg->data == "manual")  
        {
          autonomous_navigation = false;
          RCLCPP_INFO(get_logger(), "[cmd_vel_maganager] manual");
        }

    }


    void callback_nav_auto_state(std_msgs::msg::String::SharedPtr msg)
    {
        
        std::string instruction = msg->data; 
        //RCLCPP_INFO(get_logger(), "[cmd_vel_maganager] '%s'", instruction.c_str());

        if (instruction == "NAV_AUTONOMOUS_START")   
        {
            autonomous_navigation = true;
            RCLCPP_INFO(get_logger(), "[cmd_vel_maganager] '%s'", msg->data.c_str());
            RCLCPP_INFO_ONCE(get_logger(), "[cmd_vel_maganager] auto nav value: '%d'", autonomous_navigation);
        } 
        else if (msg->data == "NAV_AUTONOMOUS_END")   
        {
            autonomous_navigation = false;
            RCLCPP_INFO(get_logger(), "[cmd_vel_maganager] '%s'", msg->data.c_str());
            RCLCPP_INFO_ONCE(get_logger(), "[cmd_vel_maganager] auto nav value: '%d'", autonomous_navigation);
        }         
    }



    void callback_cmd_vel_manual(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      
      if (autonomous_navigation == false)
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
      
      if (autonomous_navigation == true)
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

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;      

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_auto;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_manual;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_nav_nodes_state;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_shutdown;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_nav;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr destroy_sub_;


    size_t count_;
    bool autonomous_navigation;

};


int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdvelManager>());

  rclcpp::shutdown();

  return 0;

}


