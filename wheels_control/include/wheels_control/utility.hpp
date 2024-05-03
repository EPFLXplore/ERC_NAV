#pragma once
#ifndef _UTILITY_WHEELS_COMMMANDS_H_
#define _UTILITY_WHEELS_COMMANDS_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>



using namespace std;


class ParamServer : public rclcpp::Node
{
public:
    std::string robot_id;

    //Topics
    string cmdvelManualTopic; 
    string cmdvelAutonomTopic;
    string joynavTopic;              

    //Frames
    string baselinkFrame;
    string mapFrame;


    ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
    {
        declare_parameter("cmdvelManualTopic", "/NAV/manual_cmd_vel");
        get_parameter("cmdvelManualTopic", cmdvelManualTopic);
        declare_parameter("cmdvelAutonomTopic", "/NAV/auto_cmd_vel");
        get_parameter("cmdvelAutonomTopic", cmdvelAutonomTopic);
        declare_parameter("joynavTopic", "/CS/NAV_gamepad");
        get_parameter("joynavTopic", joynavTopic); 

    }
};

#endif