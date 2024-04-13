/*
pkg:    wheels_commands
node:   NAV_gamepad_interface
topics: 
        publish:    /NAV/cmd_vel_manual
        subscribe:  /CS/NAV_gamepad 
        
description:  

*/


#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include "wheels_control/utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "custom_msg/msg/motorcmds.hpp" 
#include "custom_msg/msg/wheelstatus.hpp"


#include "wheels_control/definition.hpp"


using namespace std::chrono_literals;

int windowSize = 10;
std::vector<double> buffer_x;
std::vector<double> buffer_z;

//------------------------------------NODE DEFINITION---------------------------------------



class GamepadInterface : public rclcpp::Node
{
  public:
    GamepadInterface() : Node("NAV_gamepad_interface"), count_(0)
    {
      pub_cmd_vel_manual = this->create_publisher<geometry_msgs::msg::Twist>("/NAV/cmd_vel_manual", 10); 
      pub_nav_auto_state = this->create_publisher<std_msgs::msg::String>("/NAV/nav_auto_state", 10); 
      


      sub_cs_gamepad = this->create_subscription<sensor_msgs::msg::Joy>(
        "/CS/NAV_gamepad", 10, std::bind(&GamepadInterface::callback_gamepad, this, std::placeholders::_1));

      
      sub_cmds_shutdown = this->create_subscription<std_msgs::msg::String>(
        "CS/nav_shutdown_cmds", 1, std::bind(&GamepadInterface::callback_shutdown, this, std::placeholders::_1));

      destroy_sub_ = this->create_subscription<std_msgs::msg::String>("ROVER/NAV_status", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&GamepadInterface::destroy_callback, this, std::placeholders::_1));


    }

    double filter(double newValue, std::vector<double> buffer) {
        // Add the new value to the buffer
        buffer.push_back(newValue);

        // If the buffer size exceeds the specified window size, remove the oldest value
        if (buffer.size() > windowSize) {
            buffer.erase(buffer.begin());
        }

        // Calculate the average of the values in the buffer
        double sum = 0.0;
        for (double value : buffer) {
            sum += value;
        }

        return sum / buffer.size();
    }


  private:
    void callback_shutdown(std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "NAV_SHUTDOWN")   
        {
            throw std::runtime_error("Shutdown requested");
        }          

    }

     void destroy_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if(msg->data == "abort") rclcpp::shutdown();
    }

    void callback_gamepad(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        
      float r_z = (msg->axes[0]);  
      float v_x = 0;
      float v_y = 0;

      bool change_kinematics_state = msg->buttons[8];
      bool auto_state =  msg->buttons[1]; // button O
      bool manual_state =  msg->buttons[2]; // button O
      string nav_message = "";
      


      if (msg->axes[5] > 0)
        v_x = msg->axes[5];
      else
        v_x = -msg->axes[2];

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", r_z);

      // if (r_z != 0)
      //   r_z = r_z * 2 -1; // to be between -1 and 1

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", v_x);


      // if (r_z != 0)
      //   r_z = r_z * 2 -1; // to be between -1 and 1

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", r_z);


      if (auto_state == 1)
      {
        nav_message = "NAV_AUTONOMOUS_START";
      }
      else if (manual_state == 1)
      {
        nav_message = "NAV_AUTONOMOUS_END";
      }
      

      auto message = geometry_msgs::msg::Twist(); 
      message.linear.x = filter(v_x, buffer_x);
      message.linear.y = v_y;
      message.linear.z = 0;

      message.angular.x = 0;
      message.angular.y = 0;

      message.angular.z = -filter(r_z, buffer_z);

      cout<< -r_z << endl;

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", r_z);

      pub_cmd_vel_manual->publish(message);


      auto msg_nav_auto_state = std_msgs::msg::String(); 
      msg_nav_auto_state.data = nav_message;

      pub_nav_auto_state->publish(msg_nav_auto_state);
      
    }


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_manual; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_nav_auto_state;       


    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_cs_gamepad;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmds_shutdown;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr destroy_sub_;

    

    size_t count_;


};


int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadInterface>());

  rclcpp::shutdown();

  return 0;

}


