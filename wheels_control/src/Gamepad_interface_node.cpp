/*
pkg:    wheels_commands
node:   NAV_gamepad_interface
topics: 
        publish:    /NAV/cmd_vel_manual
        subscribe:  /CS/NAV_gamepad 
        
Last updated:       22/11/2024
Rewritting author:  Cyril Goffin
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

int windowSize = 30;
int windowSizeSteering = 50;
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
      
    }

    double filter(double newValue, std::vector<double> buffer) 
    {
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

    double filter_steering(double newValue, std::vector<double> buffer) 
    {
        buffer.push_back(newValue);

        if (buffer.size() > windowSizeSteering) {
            buffer.erase(buffer.begin());
        }

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


    void callback_gamepad(const sensor_msgs::msg::Joy::SharedPtr msg)
    {


      // Gamepad buttons and axes attribution
      int GP_axis_R2 = 5;
      int GP_axis_L2 = 2;
      int GP_axis_joystick_left_horizontal = 0;
      int GP_button_joystick_left = 7;
      int GP_button_round = 1;
      int GP_button_cross = 0;

      float v_x = 0;
      float v_y = 0;
      float r_z = 0;

      _Float64 joystick_threadhold = 0.00001;
      


      if (std::abs(msg->axes[GP_axis_joystick_left_horizontal]) > joystick_threadhold)
      {
        if ((msg->buttons[GP_button_joystick_left]) == 1)
        {
          // ROTATION ON ITSELF (CRAB)
            r_z = msg->axes[msg->axes[GP_axis_joystick_left_horizontal]];  
            v_x = 0;
            v_y = 0;
        }
        else
        {
          // ROTATION AND TRANSLATION
          if ((msg->axes[GP_axis_R2] > joystick_threadhold) || (msg->axes[GP_axis_L2] > joystick_threadhold))
          {
            if (((msg->axes[GP_axis_R2]) > joystick_threadhold) && ((msg->axes[GP_axis_L2]) < joystick_threadhold)) 
            {
              r_z = msg->axes[GP_axis_joystick_left_horizontal];  
              v_x = msg->axes[GP_axis_R2];
              v_y = 0;
            }
            else if (((msg->axes[GP_axis_R2]) < joystick_threadhold) && ((msg->axes[GP_axis_L2]) > joystick_threadhold))
            {
              r_z = msg->axes[GP_axis_joystick_left_horizontal];  
              v_x = -msg->axes[GP_axis_L2];
              v_y = 0;
            }
            else
            {
              // DON'T MOVE
              r_z = 0;  
              v_x = 0;
              v_y = 0;
            }
          }
          else 
          {
            // DON'T MOVE
            r_z = 0;  
            v_x = 0;
            v_y = 0;
          } 
        }
      }
     
      else if ((msg->axes[GP_axis_R2] > joystick_threadhold) || (msg->axes[GP_axis_L2] > joystick_threadhold))
      {
        // ONLY TRANSLATION
        if (((msg->axes[GP_axis_R2]) > joystick_threadhold) && ((msg->axes[GP_axis_L2]) < joystick_threadhold)) 
        {
          // FORWARD TRANSLATION
          r_z = 0;  
          v_x = msg->axes[GP_axis_R2];
          v_y = 0;
        }
        else if (((msg->axes[GP_axis_R2]) < joystick_threadhold) && ((msg->axes[GP_axis_L2]) > joystick_threadhold))
        {
          // BACKWARD TRANSLATION
          r_z = 0;  
          v_x = -msg->axes[GP_axis_L2];
          v_y = 0;
        }
        else
        {
          // DON'T MOVE
          r_z = 0;  
          v_x = 0;
          v_y = 0;
        }
      }

      else 
      {
        // DON'T MOVE
        r_z = 0;  
        v_x = 0;
        v_y = 0;
      }

      // Switch between manual and autonomous mode
      
      // current_rover_state.rover_mode = MANUAL; //manual mode by default

      // std::string nav_message = "";

      // if (msg->buttons[GP_button_round]) {
      //   //if current mode is manual, switch to autonomous, else switch to manual
      //   current_rover_state.rover_mode = (current_rover_state.rover_mode == MANUAL) ? AUTO : MANUAL;
      // }

      // nav_message = (current_rover_state.rover_mode == MANUAL) ? NAV_AUTO_END : NAV_AUTO_START;

      // // Switch between kinematics state
      
      // current_rover_state.motion_mode = NORMAL; //normal kinematic mode by default

      // std::string nav_kinematics_message = "";

      // if (msg->buttons[GP_button_cross]) {
      //   //if current mode is normal, switch to lateral, else switch to normal
      //   current_rover_state.motion_mode = (current_rover_state.motion_mode == NORMAL) ? LATERAL : NORMAL;
      // }
      // //if rover in normal mode, send normal kinematic message, else send lateral kinematic message
      // nav_kinematics_message = (current_rover_state.motion_mode == NORMAL) ? NAV_NORMAL_KINEMATIC : NAV_LATERAL_KINEMATIC;
      
      auto message = geometry_msgs::msg::Twist(); 
      message.linear.x = filter(v_x, buffer_x);
      message.linear.y = v_y;
      message.linear.z = 0;

      message.angular.x = 0;
      message.angular.y = 0;

      message.angular.z = -filter_steering(r_z, buffer_z);

      pub_cmd_vel_manual->publish(message);
      /*
      auto msg_nav_auto_state = std_msgs::msg::String(); 
      msg_nav_auto_state.data = nav_kinematics_message;

      pub_nav_auto_state->publish(msg_nav_auto_state);
      */
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