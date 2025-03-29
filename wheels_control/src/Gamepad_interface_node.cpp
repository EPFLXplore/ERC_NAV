/*
pkg:    wheels_commands
node:   NAV_gamepad_interface
topics: 
        publish:    /NAV/cmd_vel_manual
        subscribe:  /ROVER/NAV_gamepad 
        
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

const int windowSize = 30;
const int windowSizeSteering = 50;
std::vector<double> buffer_x;
std::vector<double> buffer_z;
static ROVER_MODE previous_rover_mode = ROVER_MODE::OFF;


//------------------------------------NODE DEFINITION---------------------------------------

class GamepadInterface : public rclcpp::Node
{
  public:
    GamepadInterface() : Node("NAV_gamepad_interface"), count_(0)
    {
      pub_cmd_vel_manual = this->create_publisher<geometry_msgs::msg::Twist>("/NAV/cmd_vel_manual", 10); 

      // Listens to the gamepad topic of the CS
      sub_cs_gamepad = this->create_subscription<sensor_msgs::msg::Joy>(
        "/ROVER/NAV_gamepad", 10, std::bind(&GamepadInterface::callback_gamepad, this, std::placeholders::_1));
      
      // Listens on the NAVCSInterface for the actual mode of the rover
      sub_state_system = this->create_subscription<std_msgs::msg::String>(
        "/NAV/NAV_mode", 1, std::bind(&GamepadInterface::callback_state_mode, this, std::placeholders::_1));
    
      current_rover_state = ROVER_MODE::OFF;
    }

    double apply_deadzone(double value, double deadzone = 0.2){
      double abs_val = std::abs(value);
      if(abs_val < deadzone){
        return 0.0;
      }else{
        double scaled = (abs_val - deadzone) / (1.0 - deadzone);
        return (value > 0 ? 1: -1) * scaled;
      }
    }

    double filter(double newValue, std::vector<double>& buffer)
    {
        //newValue = apply_deadzone(newValue);
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

    double filter_steering(double newValue, std::vector<double>& buffer) 
    {
        //newValue = apply_deadzone(newValue);
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
        previous_rover_mode = current_rover_state;
      }
      else if (msg->data == "Ackermann")
      {
        // check different state
        current_rover_state = ROVER_MODE::ACKERMANN;
        previous_rover_mode = current_rover_state;
      }
      else if (msg->data == "Omni")
      {
        current_rover_state = ROVER_MODE::OMNI_DIRECTIONAL;
        previous_rover_mode = current_rover_state;
      }
      else if (msg->data == "Off")
      {
        current_rover_state = ROVER_MODE::OFF;
        previous_rover_mode = current_rover_state;
      }
    }

    void callback_gamepad(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      float v_x = 0;
      float v_y = 0;
      float r_z = 0;

      float R2_val = apply_deadzone(msg->axes[GP_AXIS_R2], 0.1);
      float L2_val = apply_deadzone(msg->axes[GP_AXIS_L2], 0.1);
      float joy_left_vert = apply_deadzone(msg->axes[GP_AXIS_JOYSTICK_LEFT_VERTICAL]);
      float joy_left_horiz = apply_deadzone(msg->axes[GP_AXIS_JOYSTICK_LEFT_HORIZONTAL]);


      if (current_rover_state == ROVER_MODE::OMNI_DIRECTIONAL)
      {
        //RCLCPP_INFO(this->get_logger(), "Omni Mode");

        if (R2_val >= JOYSTICK_THRESHOLD)
        {
          v_x = R2_val;
          v_y = 0;

        } else if(L2_val >= JOYSTICK_THRESHOLD)
        {
          v_x = -L2_val;
          v_y = 0;
        }
        
        if (joy_left_vert >= JOYSTICK_THRESHOLD) //[0, pi]
        {
          r_z = atan2(joy_left_vert, joy_left_horiz);
        } else
        {
          r_z = M_PI - atan2(joy_left_vert, joy_left_horiz);
        }
      
      } else if (current_rover_state == ROVER_MODE::ACKERMANN)
      {
        //RCLCPP_INFO(this->get_logger(), "Acker Mode");
        if (std::abs(joy_left_horiz) > JOYSTICK_THRESHOLD)
        { // if we want to turn

          // if ((msg->buttons[GP_BUTTON_JOYSTICK_LEFT]) == 1)
          // {
          //   current_rover_state == ROVER_MODE::CRAB;
          //   RCLCPP_INFO(this->get_logger(), "Acker Crab");

          //   // ROTATION ON ITSELF (CRAB)
          //     r_z = joy_left_horiz;
          //     v_y = 0;
          //     if (R2_val >= JOYSTICK_THRESHOLD)
          //     {
          //       v_x = R2_val;
      
          //     } else if(L2_val >= JOYSTICK_THRESHOLD)
          //     {
          //       v_x = -L2_val;
          //     }
          // }
          
          // ROTATION AND TRANSLATION
          //RCLCPP_INFO(this->get_logger(), "Classic Curve");

          if ((R2_val > JOYSTICK_THRESHOLD) || (L2_val > JOYSTICK_THRESHOLD))
          { //if any trigger is pressed => moving
            //RCLCPP_INFO(this->get_logger(), "Acker Classic");

            if (((R2_val) > JOYSTICK_THRESHOLD) && ((L2_val) < JOYSTICK_THRESHOLD)) 
            { //if going forwards
              //RCLCPP_INFO(this->get_logger(), "acker forwards");

              r_z = joy_left_horiz;
              v_x = R2_val;
              v_y = 0;
            }
            else if (((R2_val) < JOYSTICK_THRESHOLD) && ((L2_val) > JOYSTICK_THRESHOLD))
            {//if going backwards
              //RCLCPP_INFO(this->get_logger(), "acker backwards");

              r_z = joy_left_horiz;  
              v_x = -L2_val;
              v_y = 0;
            }
            else
            {
            //RCLCPP_INFO(this->get_logger(), "Acker dont move");

              // DON'T MOVE
              r_z = 0;  
              v_x = 0;
              v_y = 0;
            }
          }else if((msg->buttons[GP_BUTTON_JOYSTICK_LEFT]) == 1){
            r_z = joy_left_horiz;
            v_y = 0;
            v_x = 0;
            //RCLCPP_INFO(this->get_logger(), "Acker Crab 22");

          }else 
          {
            //RCLCPP_INFO(this->get_logger(), "Acker dont move 2");

            // DON'T MOVE
            r_z = 0;  
            v_x = 0;
            v_y = 0;
          } 
          
        }
     
        else if ((R2_val > JOYSTICK_THRESHOLD) || (L2_val > JOYSTICK_THRESHOLD))
        {
          // ONLY TRANSLATION
          //RCLCPP_INFO(this->get_logger(), "translation only");

          if (((R2_val) > JOYSTICK_THRESHOLD) && ((L2_val) < JOYSTICK_THRESHOLD)) 
          {
            //RCLCPP_INFO(this->get_logger(), "forwards translation");

            // FORWARD TRANSLATION
            r_z = 0;  
            v_x = R2_val;
            v_y = 0;
          }
          else if (((R2_val) < JOYSTICK_THRESHOLD) && ((L2_val) > JOYSTICK_THRESHOLD))
          {
            //RCLCPP_INFO(this->get_logger(), "backwards translation");

            // BACKWARD TRANSLATION
            r_z = 0;  
            v_x = -L2_val;
            v_y = 0;
          }
          else
          {
            //RCLCPP_INFO(this->get_logger(), "dont move translation");

            // DON'T MOVE
            r_z = 0;  
            v_x = 0;
            v_y = 0;
          }
        }

        else 
        {
          //RCLCPP_INFO(this->get_logger(), "dont move dawg");

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
      //RCLCPP_INFO(this->get_logger(), "Final gpd interf : vx: %.3f, vy: %.3f, rz: %.3f", message.linear.x, message.linear.y, message.angular.z);


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
