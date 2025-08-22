/*
pkg:    wheels_commands
node:   NAV_gamepad_interface
topics: 
        publish:    /NAV/cmd_vel_manual
        subscribe:  /ROVER/NAV_gamepad, /NAV/NAV_mode
        
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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/qos.hpp"

#include "custom_msg/msg/motorcmds.hpp" 
#include "custom_msg/msg/wheelstatus.hpp"


#include "wheels_control/definition.hpp"


using namespace std::chrono_literals;

const int windowSize = 2;
const int windowSizeSteering = 2;
std::vector<double> buffer_x;
std::vector<double> buffer_z;
static ROVER_MODE previous_rover_mode = ROVER_MODE::OFF;
const double CMD_TIMEOUT = 0.4; //seconds, the CS should be publishing at 30Hz = 0.3s

//------------------------------------NODE DEFINITION---------------------------------------

class GamepadInterface : public rclcpp::Node
{
  public:
    GamepadInterface() : Node("NAV_gamepad_interface"), count_(0)
    {


      auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(1));
      qos_best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);
      qos_best_effort.durability(rclcpp::DurabilityPolicy::Volatile);

      pub_cmd_vel_manual = this->create_publisher<geometry_msgs::msg::Twist>("/NAV/cmd_vel_manual", 10); 

      // Listens to the gamepad topic of the CS
      sub_cs_gamepad = this->create_subscription<sensor_msgs::msg::Joy>(
      "/ROVER/NAV_gamepad", qos_best_effort, std::bind(&GamepadInterface::callback_gamepad, this, std::placeholders::_1));

        
      // Listens on the NAVCSInterface for the actual mode of the rover
      sub_state_system = this->create_subscription<std_msgs::msg::String>(
        "/NAV/NAV_mode", 1, std::bind(&GamepadInterface::callback_state_mode, this, std::placeholders::_1));
    
      current_rover_state = ROVER_MODE::OFF;

      last_command_time = this->now();

      timeout_timer = this->create_wall_timer(
        100ms, std::bind(&GamepadInterface::check_command_timeout, this));
      
    }

    double apply_deadzone(double value, double deadzone = 0.1){
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
          static float prev_wrapped_angle = 0.0f;
          static bool ccw_turn = false;
          static bool cw_turn = false;

      
          if (std::abs(joy_left_vert) >= JOYSTICK_THRESHOLD && std::abs(joy_left_horiz) >= JOYSTICK_THRESHOLD) {
              float curr_angle = std::atan2(joy_left_horiz, joy_left_vert); // [-pi, pi]
              //RCLCPP_INFO(this->get_logger(), "curent ANG : %.3f °", curr_angle*180/3.1415);

              float delta_angle = curr_angle - prev_angle;
                  
              float curr_deg = curr_angle * RAD_TO_DEG; // de base on a: < < > >

              float prev_deg = prev_angle * RAD_TO_DEG;
              
              //RCLCPP_INFO(this->get_logger(), "delta: : %.3f", delta_angle*180/3.1415);

              float sign_curr_prev = curr_angle * prev_angle;

              if(sign_curr_prev == 0 or std::abs(delta_angle) <= DELTA_THRESHOLD){
                // If the wheels are aleady turning in a direction and we nudge the angle in the the same direction
                //RCLCPP_INFO(this->get_logger(), "keeping same angle: %.3f ", prev_angle);
                r_z = prev_angle;
              }
              else if (sign_curr_prev > 0 && fabs(delta_angle) > DELTA_THRESHOLD) {
                  //we are moving in the same direction as the previous angle.
                  
                  cw_turn = delta_angle > DELTA_THRESHOLD;
                  ccw_turn = delta_angle <= -DELTA_THRESHOLD;

              } else if(fabs(delta_angle) > DELTA_THRESHOLD) {
                  //RCLCPP_INFO(this->get_logger(), "curent case 5: : %.3f", curr_deg);
                  //RCLCPP_INFO(this->get_logger(), "delta case 5: %.3f", delta_angle*180/3.1415); 
                  float opposite_curr_angle = 0.0;
                  if(curr_deg <= 0.0){
                      opposite_curr_angle = 180.0 + curr_deg;
                  }else{
                      opposite_curr_angle = curr_deg - 180.0;
                  }

                  //delta < 0 => prev > curr
                  if (delta_angle < -DELTA_THRESHOLD && curr_deg < -90 && prev_deg > opposite_curr_angle) {
                      // RCLCPP_INFO(this->get_logger(), "turn case 1");
                      cw_turn = true;
                      ccw_turn = false;
                      

                  } else if (delta_angle < -DELTA_THRESHOLD && curr_deg < -90 && prev_deg < opposite_curr_angle){
                      ccw_turn = true;
                      cw_turn = false;
                  }
                  
                
                  else if (delta_angle < -DELTA_THRESHOLD && curr_deg > -90 && prev_deg < opposite_curr_angle) {
                      // RCLCPP_INFO(this->get_logger(), "turn case 2");
                      cw_turn = false;
                      ccw_turn = true;

                      
                  } else if (delta_angle < -DELTA_THRESHOLD && curr_deg > -90 && prev_deg > opposite_curr_angle) {
                      cw_turn = true;
                      ccw_turn = false;

                  }

                  //delta > 0 => curr > prev 
                  
                  else if (delta_angle > DELTA_THRESHOLD && curr_deg > 90 && prev_deg < opposite_curr_angle) {
                      // RCLCPP_INFO(this->get_logger(), "turn case 3");

                      cw_turn = false;
                      ccw_turn = true;
                  } else if (delta_angle > DELTA_THRESHOLD && curr_deg > 90 && prev_deg > opposite_curr_angle) {
                      cw_turn = true;
                      ccw_turn = false;
                  }

                  else if (delta_angle > DELTA_THRESHOLD && curr_deg < 90 && prev_deg < opposite_curr_angle) {
                      // RCLCPP_INFO(this->get_logger(), "turn case 4");

                      cw_turn = false;
                      ccw_turn = true;


                  } else if (delta_angle > DELTA_THRESHOLD && curr_deg < 90 && prev_deg > opposite_curr_angle) {
                      // RCLCPP_INFO(this->get_logger(), "turn case 4");

                      cw_turn = true;
                      ccw_turn = false;


                  }
              }else{
                r_z = prev_angle;
              }

      
              float wrapped_angle = curr_angle; 
              //quand on passe de 170 a -170 on doit en realite aller a 360 -170 = 190 

              //std::fmod(prev_wrapped_angle, 2 * PI) : this tells us from where the angle is coming from when there is a sign flip  
              if((prev_deg <= 90.0 &&  prev_deg >= 0.0) && (curr_deg <= 0.0 && curr_deg >= -90.0)){
                  wrapped_angle = curr_angle;
              }else if((curr_deg <= 90.0 &&  curr_deg >= 0.0) && (prev_deg <= 0.0 && prev_deg >= -90.0)){
                  wrapped_angle = curr_angle;
              }          

              else if ((cw_turn || ccw_turn) && curr_angle < 0 && std::fmod(prev_wrapped_angle, 2 * PI) > 0) {
                  //RCLCPP_INFO(this->get_logger(), "wrapped 1");
                  wrapped_angle = curr_angle + 2 * PI;
              }
              
              else if ((cw_turn || ccw_turn)  && curr_angle > 0 && std::fmod(prev_wrapped_angle, 2 * PI) < 0) {
                  //RCLCPP_INFO(this->get_logger(), "wrapped 2");

                  wrapped_angle = curr_angle - 2 * PI;
              }else{
                  wrapped_angle = curr_angle;
              }
            
              prev_angle = curr_angle;
              prev_wrapped_angle = wrapped_angle;

              r_z = wrapped_angle;
              //RCLCPP_INFO(this->get_logger(), "rz output omni: : %.3f", r_z);
              
          } 

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

  auto gamepad_node = std::make_shared<GamepadInterface>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(gamepad_node);

  RCLCPP_INFO(gamepad_node->get_logger(), "Spinning gamepad interface with MultiThreadedExecutor");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}