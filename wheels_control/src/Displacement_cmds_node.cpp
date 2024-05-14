/*
pkg:    wheels_commands
node:   NAV_displacement_cmds
topics: 
        publish:    /NAV/displacement
        subscribe:  /CS/NAV_gamepad  - /NAV/absolute_encoders - /NAV/cmd_vel_final - /ROVER/NAV_status -
        
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
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "custom_msg/msg/motorcmds.hpp" 
#include "custom_msg/msg/wheelstatus.hpp"

#include "wheels_control/definition.hpp"
#include "wheels_control/basic_kinematic_model.hpp"
#include "wheels_control/normal_kinematic_model.hpp"
#include "wheels_control/normal_kinematic_model_slow.hpp"
#include "wheels_control/lateral_kinematic_model.hpp"
#include "wheels_control/stby_kinematic_model.hpp"



using namespace std::chrono_literals;



motors_obj current_motors_cmds = {{""},{0,0,0,0},{0,0,0,0}};
motors_obj current_motors_position = {{0,0,0,0},{0,0,0,0}};




int wheels_angle_for_rotation = 0; //  encoder intern = 2900000/8 = 362 500 unit: increment
int wheels_angle_for_rotation_with_translation = 0;


// int kinematics_state = 0; // 0 rotation or translation - 1 rotation and translation
//string kinematic_state = BASIC_KINEMATIC;
string kinematic_state = NORMAL_KINEMATIC;




//------------------------------------NODE DEFINITION---------------------------------------

_Float64 get_wheels_angle_inc_for_rotation(_Float64 width = 0.833, _Float64 length = 0.736)
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

    angle_of_rotation_radians = atan(width / length);
    angle_of_rotation_increment = (angle_of_rotation_radians * (pow(2, TOUR_RESOLUTION_BITS))) / (2 * M_PI);

    return angle_of_rotation_increment;
}


class DisplacementCmds : public rclcpp::Node
{
  public:
    DisplacementCmds() : Node("NAV_displacement_cmds"), count_(0)
    {
      //RCLCPP_INFO(get_logger(), "STATE KINEMATIC INIT '%s'",  kinematic_state.c_str());

      pub_kinematic = this->create_publisher<custom_msg::msg::Motorcmds>("/NAV/displacement", 10); 
      

      sub_cs_gamepad = this->create_subscription<sensor_msgs::msg::Joy>(
        "/CS/NAV_gamepad", 1, std::bind(&DisplacementCmds::callback_gamepad, this, std::placeholders::_1));

      sub_topic_absolute_encoders = this->create_subscription<custom_msg::msg::Wheelstatus>(
        "/NAV/absolute_encoders", 1, std::bind(&DisplacementCmds::callback_absolute_encoders, this, std::placeholders::_1));

      
      sub_cmds_shutdown = this->create_subscription<std_msgs::msg::String>(
        "/ROVER/NAV_status", 1, std::bind(&DisplacementCmds::callback_abort, this, std::placeholders::_1));

      sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "/NAV/cmd_vel_final", 1, std::bind(&DisplacementCmds::callback_cmd_vel, this, std::placeholders::_1));


      sub_kinematic_state = this->create_subscription<std_msgs::msg::String>(
        "/ROVER/NAV_kinematic", 1, std::bind(&DisplacementCmds::callback_mode_kinematic, this, std::placeholders::_1));

      sub_fake_cs_crab = this->create_subscription<std_msgs::msg::String>(
        "/ROVER/l_or_r", 1, std::bind(&DisplacementCmds::callback_mode_crab, this, std::placeholders::_1));

      destroy_sub_ = this->create_subscription<std_msgs::msg::String>("ROVER/NAV_status", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&DisplacementCmds::destroy_callback, this, std::placeholders::_1));


      

      wheels_angle_for_rotation = get_wheels_angle_inc_for_rotation(); // unit: increment - value around 8 300
      wheels_angle_for_rotation_with_translation = (20 * (pow(2, 16))) / (360);

      RCLCPP_INFO(get_logger(), "ANGLE OF ROTATION INCREMENT '%d'",  wheels_angle_for_rotation);
      RCLCPP_INFO(get_logger(), "ANGLE OF ROTATION WITH TRANSLATION INCREMENT '%d'",  wheels_angle_for_rotation_with_translation);


      basicKinematicModel.init(current_motors_position, wheels_angle_for_rotation);
      normalKinematicModel.init(current_motors_position, wheels_angle_for_rotation);
      //slownormalKinematicModel.init(current_motors_position, wheels_angle_for_rotation);


    }


  private:
    bool go_left = false;
    bool go_right = false;

    void callback_abort(std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "abort")   
        {
            throw std::runtime_error("Shutdown requested");
        }          
        
    }


    void callback_mode_kinematic(std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "normal")   
        {
          kinematic_state = NORMAL_KINEMATIC;
        }  
        else if (msg->data == "basic")  
        {
          kinematic_state = BASIC_KINEMATIC;
        }
        else if (msg->data == "lateral")  
        {
          kinematic_state = LATERAL_KINEMATIC;
        else if ( msg- >data = "crab" )
        {
          kinematic_state = CRAB_KINEMATIC;
        }



    }

    void set_crab(std_msgs::msg::String::SharedPtr msg) // modifie le msg au type que je veux
    {
        if (msg->data == "left")   
        {
          stbyKinematicModel.set_sign_neg();
        }
        else if (msg->data = "right") 
        {
          stbyKinematicModel.set_sign_pos();
        }
    }


    void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      float r_z = msg->angular.z;  
      float v_x = msg->linear.x;
      float v_y = msg->linear.y;

      //RCLCPP_INFO(get_logger(), "STATE KINEMATIC callback cmdvel '%s'",  kinematic_state.c_str());

      /*Run the kinematics manager to compute the motion*/
      if(kinematic_state == NORMAL_KINEMATIC)
          // normal_kinematics_manager(v_x, v_y, r_z);  
          current_motors_cmds = normalKinematicModel.run(current_motors_position, v_x, v_y, r_z);
      else if (kinematic_state == LATERAL_KINEMATIC){
          
          current_motors_cmds = lateralKinematicModel.run(go_left,go_right);
          RCLCPP_INFO(get_logger(), "current_motors_cmds ");

      }
      else if(kinematic_state == BASIC_KINEMATIC)
      {
          // basic_kinematics_manager(v_x, v_y, r_z);
          current_motors_cmds = basicKinematicModel.run(current_motors_position, v_x, v_y, r_z);
      }else if (kinematic_state == CRAB_KINEMATIC)
      {
        current_motors_cmds = stbyKinematicModel.run(current_motors_position, v_x, v_y, r_z);
      }
      

      send_kinematic_msg();

    }


    void send_kinematic_msg()const
    {
      auto message = custom_msg::msg::Motorcmds(); 

      message.drive =  {
          current_motors_cmds.drive[0],  
          - current_motors_cmds.drive[1], // node 2 driving motor mounted in reverse
          - current_motors_cmds.drive[2], // node3 driving motor mounted in reverse
          current_motors_cmds.drive[3] 
          };      

      message.steer = {
          current_motors_cmds.steer[0], 
          - current_motors_cmds.steer[1], // steering motor mounted in reverse
          current_motors_cmds.steer[2],
          - current_motors_cmds.steer[3]  // steering motor mounted in reverse
          };  

      //RCLCPP_INFO(get_logger(), "STATE KINEMATIC",  kinematic_state);

      message.modedeplacement = kinematic_state;  
      message.info = current_motors_cmds.info;      

      //RCLCPP_INFO(get_logger(), "STATE KINEMATIC pub'%s'",  kinematic_state.c_str());                   
      
      pub_kinematic->publish(message);

    }


    void callback_gamepad(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      RCLCPP_INFO(get_logger(), "GAMEPAD PUB'%d'",  msg->buttons[3]); 
                        
      bool change_state = msg->buttons[8];
      bool lateral = (msg->buttons[3] || msg->buttons[4]);

      go_left = msg->buttons[3];
      go_right = msg->buttons[4];

      if(lateral){                  
        RCLCPP_INFO(get_logger(), "IS LATERAL ");                   
        //kinematic_state = LATERAL_KINEMATIC;
        kinematic_state = CRAB_KINEMATIC;

      }
      else if (change_state)
      {
          RCLCPP_INFO(get_logger(), "CHANGE STATE",  change_state);
          if(kinematic_state == BASIC_KINEMATIC)
          {
            kinematic_state = NORMAL_KINEMATIC;
           // RCLCPP_INFO(get_logger(), "STATE ROTATION WITH TRANSLATION",  kinematic_state);

          }
          else
          {
            kinematic_state = BASIC_KINEMATIC;
            //RCLCPP_INFO(get_logger(), "STATE ROTATION OR TRANSLATION",  kinematic_state);
          }
      } else {
        kinematic_state = NORMAL_KINEMATIC;
           // RCLCPP_INFO(get_logger(), "STATE ROTATION WITH TRANSLATION",  kinematic_state);
      }



    }


    void destroy_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if(msg->data == "abort") rclcpp::shutdown();
    }


    void callback_absolute_encoders(const custom_msg::msg::Wheelstatus::SharedPtr msg) 
    {
      /*Update the motors position*/
      // current_motors_position.drive[FRONT_LEFT] =  msg->data[FRONT_LEFT_DRIVE - 1];
      // current_motors_position.drive[FRONT_RIGHT] =  msg->data[FRONT_RIGHT_DRIVE - 1];
      // current_motors_position.drive[BACK_RIGHT] =  msg->data[BACK_RIGHT_DRIVE - 1];
      // current_motors_position.drive[BACK_LEFT] = msg->data[BACK_LEFT_DRIVE - 1];

      // current_motors_position.steer[FRONT_LEFT] = msg->data[FRONT_LEFT_STEER - 1];
      // current_motors_position.steer[FRONT_RIGHT] = msg->data[FRONT_RIGHT_STEER - 1];
      // current_motors_position.steer[BACK_RIGHT] = msg->data[BACK_RIGHT_STEER - 1];
      // current_motors_position.steer[BACK_LEFT] = msg->data[BACK_LEFT_STEER - 1];

      current_motors_position.drive[FRONT_LEFT] =  0;
      current_motors_position.drive[FRONT_RIGHT] =  0;
      current_motors_position.drive[BACK_RIGHT] =  0;
      current_motors_position.drive[BACK_LEFT] = 0;

      current_motors_position.steer[FRONT_LEFT] = msg->data[FRONT_LEFT];
      current_motors_position.steer[FRONT_RIGHT] = msg->data[FRONT_RIGHT];
      current_motors_position.steer[BACK_RIGHT] = msg->data[BACK_RIGHT];
      current_motors_position.steer[BACK_LEFT] = msg->data[BACK_LEFT];

    }


        
    RoverBasicKinematicModel basicKinematicModel;
    RoverNormalKinematicModel normalKinematicModel;
    RoverLateralKinematicModel lateralKinematicModel;
    //RoverSlowNormalKinematicModel slownormalKinematicModel;
    
    rclcpp::Publisher<custom_msg::msg::Motorcmds>::SharedPtr pub_kinematic;         
    size_t count_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_cs_gamepad;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmds_shutdown;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_kinematic_state;
    rclcpp::Subscription<custom_msg::msg::Wheelstatus>::SharedPtr sub_topic_absolute_encoders;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr destroy_sub_;

};



int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisplacementCmds>());

  rclcpp::shutdown();

  return 0;

}











