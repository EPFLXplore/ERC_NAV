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

      // Listens to the gamepad topic of the CS
      sub_cs_gamepad = this->create_subscription<sensor_msgs::msg::Joy>(
        "/CS/NAV_gamepad", 10, std::bind(&GamepadInterface::callback_gamepad, this, std::placeholders::_1));
      
      // Listens on the NAVCSInterface for the actual mode of the rover
      sub_state_system = this->create_subscription<std_msgs::msg::String>(
        "/NAV/NAV_mode", 1, std::bind(&GamepadInterface::callback_state_mode, this, std::placeholders::_1));
    
      current_rover_state = ROVER_MODE::OFF;
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

    void callback_gamepad(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      float v_x = 0;
      float v_y = 0;
      float r_z = 0;

      if (current_rover_state == ROVER_MODE::OMNI_DIRECTIONAL)
      {
        if (msg->axes[GP_AXIS_R2] >= JOYSTICK_THRESHOLD)
        {
          v_x = msg->axes[GP_AXIS_R2];
          v_y = 0;

        } else if(msg->axes[GP_AXIS_L2] >= JOYSTICK_THRESHOLD)
        {
          v_x = -msg->axes[GP_AXIS_L2];
          v_y = 0;
        }
        
        if (msg->axes[GP_AXIS_JOYSTICK_LEFT_VERTICAL] >= JOYSTICK_THRESHOLD) //[0, pi]
        {
          r_z = msg->axes[atan2(msg->axes[GP_AXIS_JOYSTICK_LEFT_VERTICAL], msg->axes[GP_AXIS_JOYSTICK_LEFT_HORIZONTAL])];
        } else
        {
          r_z = msg->axes[M_PI - atan2(msg->axes[GP_AXIS_JOYSTICK_LEFT_VERTICAL], msg->axes[GP_AXIS_JOYSTICK_LEFT_HORIZONTAL])];
        }
        


      } else if (current_rover_state == ROVER_MODE::ACKERMANN)
      {
       // RCLCPP_INFO(this->get_logger(), "ebntered");
        if (std::abs(msg->axes[GP_AXIS_JOYSTICK_LEFT_HORIZONTAL]) > JOYSTICK_THRESHOLD)
        {

          if ((msg->buttons[GP_BUTTON_JOYSTICK_LEFT]) == 1)
          {
            // ROTATION ON ITSELF (CRAB)
              r_z = msg->axes[msg->axes[GP_AXIS_JOYSTICK_LEFT_HORIZONTAL]];  
              v_x = 0;
              v_y = 0;
          }
          else
          {
            // ROTATION AND TRANSLATION
            if ((msg->axes[GP_AXIS_R2] > JOYSTICK_THRESHOLD) || (msg->axes[GP_AXIS_L2] > JOYSTICK_THRESHOLD))
            {
              if (((msg->axes[GP_AXIS_R2]) > JOYSTICK_THRESHOLD) && ((msg->axes[GP_AXIS_L2]) < JOYSTICK_THRESHOLD)) 
              {
                r_z = msg->axes[GP_AXIS_JOYSTICK_LEFT_HORIZONTAL];  
                v_x = msg->axes[GP_AXIS_R2];
                v_y = 0;
              }
              else if (((msg->axes[GP_AXIS_R2]) < JOYSTICK_THRESHOLD) && ((msg->axes[GP_AXIS_L2]) > JOYSTICK_THRESHOLD))
              {
                r_z = msg->axes[GP_AXIS_JOYSTICK_LEFT_HORIZONTAL];  
                v_x = -msg->axes[GP_AXIS_L2];
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
     
        else if ((msg->axes[GP_AXIS_R2] > JOYSTICK_THRESHOLD) || (msg->axes[GP_AXIS_L2] > JOYSTICK_THRESHOLD))
        {
          // ONLY TRANSLATION
          if (((msg->axes[GP_AXIS_R2]) > JOYSTICK_THRESHOLD) && ((msg->axes[GP_AXIS_L2]) < JOYSTICK_THRESHOLD)) 
          {
            // FORWARD TRANSLATION
            r_z = 0;  
            v_x = msg->axes[GP_AXIS_R2];
            v_y = 0;
          }
          else if (((msg->axes[GP_AXIS_R2]) < JOYSTICK_THRESHOLD) && ((msg->axes[GP_AXIS_L2]) > JOYSTICK_THRESHOLD))
          {
            // BACKWARD TRANSLATION
            r_z = 0;  
            v_x = -msg->axes[GP_AXIS_L2];
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

      
      auto message = geometry_msgs::msg::Twist(); 
      message.linear.x = filter(v_x, buffer_x);
      message.linear.y = v_y;
      message.linear.z = 0;

      message.angular.x = 0;
      message.angular.y = 0;

      message.angular.z = -filter_steering(r_z, buffer_z);

      pub_cmd_vel_manual->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_manual; 
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_cs_gamepad;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_system;

    size_t count_;
    ROVER_MODE current_rover_state;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadInterface>());

  rclcpp::shutdown();

  return 0;
}
