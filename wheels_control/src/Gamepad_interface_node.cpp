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
const double CMD_TIMEOUT = 0.4; //seconds

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

      last_command_time = this->now();

      timeout_timer = this->create_wall_timer(
        100ms, std::bind(&GamepadInterface::check_command_timeout, this));
      
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

    void check_command_timeout()
    {
      rclcpp::Time now = this->now();
      rclcpp::Duration elapsed = now - last_command_time;

      if (elapsed.seconds() > CMD_TIMEOUT)
      {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.linear.y = 0.0;
        stop_msg.linear.z = 0.0;
        stop_msg.angular.x = 0.0;
        stop_msg.angular.y = 0.0;
        stop_msg.angular.z = 0.0;

        pub_cmd_vel_manual->publish(stop_msg);
        buffer_x.clear();
        buffer_z.clear();
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

      float R2_val = apply_deadzone(msg->axes[GP_AXIS_R2], 0.08);
      float L2_val = apply_deadzone(msg->axes[GP_AXIS_L2], 0.08);
      float joy_left_vert = apply_deadzone(msg->axes[GP_AXIS_JOYSTICK_LEFT_VERTICAL]);
      float joy_left_horiz = apply_deadzone(msg->axes[GP_AXIS_JOYSTICK_LEFT_HORIZONTAL]);


      if (current_rover_state == ROVER_MODE::OMNI_DIRECTIONAL)
      {
        //RCLCPP_INFO(this->get_logger(), "Omni Mode");

        if (R2_val >= JOYSTICK_THRESHOLD)
        {
          v_x = R2_val;
          v_y = 0;
          //RCLCPP_INFO(this->get_logger(), "vx omni begin r2: %.3f", v_x);


        } else if(L2_val >= JOYSTICK_THRESHOLD)
        {
          v_x = -L2_val;
          v_y = 0;
          //RCLCPP_INFO(this->get_logger(), "vx omni begin l2: %.3f", v_x);

        }


        
        if (std::fabs(joy_left_vert)>= JOYSTICK_THRESHOLD || std::fabs(joy_left_horiz) >= JOYSTICK_THRESHOLD) //[0, pi]
        {

          // RCLCPP_INFO(this->get_logger(), "angle up: %.3f", r_z*180/3.1415);
          // RCLCPP_INFO(this->get_logger(), "angle up after: %.3f", r_z*180/3.1415);
          const float PI = 3.14159265358979323846f;
          const float DELTA_THRESHOLD = 0.096;
          const float RAD_TO_DEG = 180.0f / PI;
          // const double TURN_THRESHOLD = 300.0f; // this should be the thrshold that once you attain we can consider that we passed a turn
      
          static float prev_angle = 0.0f;
          static float prev_delta = 0.0f;
          static float prev_wrapped_angle = 0.0f;
          static bool ccw_turn = false;
          static bool cw_turn = false;
          // static int num_turns = 0;
      
          // float remainder = 0.0f;
      
          // if (reset) {
          //     prev_angle = 0.0f;
          //     prev_delta = 0.0f;
          //     prev_wrapped_angle = 0.0f;
          //     ccw_turn = false;
          //     cw_turn = false;
          //     num_turns = 0;
          //     return 0.0f;
          // }
      
          if (std::abs(joy_left_vert) >= JOYSTICK_THRESHOLD && std::abs(joy_left_horiz) >= JOYSTICK_THRESHOLD) {
              float curr_angle = std::atan2(joy_left_horiz, joy_left_vert); // [-pi, pi]
              //RCLCPP_INFO(this->get_logger(), "curent: : %.3f", curr_angle*180/3.1415);

              float delta_angle = curr_angle - prev_angle;
              //RCLCPP_INFO(this->get_logger(), "delta: : %.3f", delta_angle*180/3.1415);

              float sign_curr_prev = curr_angle * prev_angle;
              if(sign_curr_prev == 0){
                //RCLCPP_INFO(this->get_logger(), "turn case equal");

                r_z = prev_angle;
              }
              else if (sign_curr_prev > 0 && fabs(delta_angle) > DELTA_THRESHOLD) {
                  cw_turn = delta_angle > DELTA_THRESHOLD;

                  ccw_turn = delta_angle <= -DELTA_THRESHOLD;

                  if(cw_turn){
                    //RCLCPP_INFO(this->get_logger(), "case 0, CW TURN");
                  }
                  if(ccw_turn){
                    //RCLCPP_INFO(this->get_logger(), "case 0, CCW TURN");
                  }

                  //RCLCPP_INFO(this->get_logger(), "turn case 0");

              } else {
                  float curr_deg = curr_angle * RAD_TO_DEG; // de base on a: < < > >
                  //RCLCPP_INFO(this->get_logger(), "curent case 5: : %.3f", curr_deg);
                  //RCLCPP_INFO(this->get_logger(), "delta case 5: %.3f", delta_angle*180/3.1415); 
                  if (delta_angle < -DELTA_THRESHOLD && curr_deg < -90) {
                      // RCLCPP_INFO(this->get_logger(), "turn case 1");

                      cw_turn = true;
                      ccw_turn = false;
                  } else if (delta_angle < -DELTA_THRESHOLD && curr_deg > -90) {
                      // RCLCPP_INFO(this->get_logger(), "turn case 2");

                      cw_turn = false;
                      ccw_turn = true;
                  } else if (delta_angle > DELTA_THRESHOLD && curr_deg > 90) {
                      // RCLCPP_INFO(this->get_logger(), "turn case 3");

                      cw_turn = false;
                      ccw_turn = true;
                  } else if (delta_angle > DELTA_THRESHOLD && curr_deg < 90) {
                      // RCLCPP_INFO(this->get_logger(), "turn case 4");

                      cw_turn = true;
                      ccw_turn = false;
                  }else{
                      // RCLCPP_INFO(this->get_logger(), cw_turn, "turn case 8", ccw_turn);


                  }
              }
      
              float wrapped_angle = curr_angle; 
              if (cw_turn && curr_angle < 0 && std::fmod(std::abs(prev_wrapped_angle), 2 * PI) > 0) {
                  // RCLCPP_INFO(this->get_logger(), "wrapped clockwise");

                  wrapped_angle = curr_angle + 2 * PI; 
              } else if (ccw_turn && curr_angle > 0 && std::fmod(std::abs(prev_wrapped_angle), 2 * PI) < 0) {
                  //RCLCPP_INFO(this->get_logger(), "wrapped anti-clockwise");

                  wrapped_angle = curr_angle - 2 * PI;
              }
      
              float wrapped_deg = wrapped_angle * RAD_TO_DEG;
      
              prev_angle = curr_angle;
              prev_delta = delta_angle;
              prev_wrapped_angle = wrapped_angle;
      
              //return wrapped_angle * RAD_TO_DEG;
              //RCLCPP_INFO(this->get_logger(), "in wrapped");

              r_z = wrapped_angle;
              
          } 
            // else if (std::abs(prev_delta * RAD_TO_DEG) > 30 &&
            //          std::abs(joy_left_vert) <= JOYSTICK_THRESHOLD &&
            //          std::abs(joy_left_horiz) <= JOYSTICK_THRESHOLD) {
            //   if(prev_wrapped_angle > 0) {
            //       remainder = std::fmod(std::abs(prev_wrapped_angle), 2 * PI) * RAD_TO_DEG;
            //   }
            //   else {
            //       remainder = -std::fmod(std::abs(prev_wrapped_angle), 2 * PI) * RAD_TO_DEG;
            //   }
      
            //   float desired_angle = 0.0f;
      
            //   //std::cout << "Remainder: " << remainder << "\n";
            //   RCLCPP_INFO(this->get_logger(), "Remainder : %.3f", remainder);

            //   //std::cout << "Num Turns: " << num_turns << "\n";
            //   RCLCPP_INFO(this->get_logger(), "Num turns : %.3f", num_turns);

              
              // if (remainder > 180 && num_turns >= 0) {
              //     desired_angle = (num_turns + 1) * 2 * PI;
              // } else if (remainder > 180 && num_turns <= 0) {
              //     desired_angle = (num_turns - 1) * 2 * PI;
              // } else if (remainder < -180 && num_turns <= 0) {
              //     desired_angle = (num_turns - 1) * 2 * PI;
              // } else if (remainder > -180 && num_turns >= 0) {
              //     desired_angle = (num_turns + 1) * 2 * PI;
              // } else {
              //     desired_angle = num_turns * 2 * PI;
              // }
              // r_z = desired_angle;
          //} 
          else {
            // RCLCPP_INFO(this->get_logger(), "keeping prev wrapped angle");

            r_z = prev_wrapped_angle;
          }

          //RCLCPP_INFO(this->get_logger(), "r_z omni: : %.3f", r_z*180/3.1415);


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
      //RCLCPP_INFO(this->get_logger(), "vx pre-final: %.3f", v_x);

      message.linear.x = filter(v_x, buffer_x);
      message.linear.y = v_y;
      message.linear.z = 0;

      message.angular.x = 0;
      message.angular.y = 0;

      message.angular.z = -filter_steering(r_z, buffer_z);
      //RCLCPP_INFO(this->get_logger(), "Final gamepad : vx: %.3f, vy: %.3f, rz: %.3f", message.linear.x, message.linear.y, message.angular.z);


      pub_cmd_vel_manual->publish(message);
      last_command_time = this->now();

    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_manual; 
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_cs_gamepad;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_system;

    rclcpp::Time last_command_time;
    rclcpp::TimerBase::SharedPtr timeout_timer;

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
