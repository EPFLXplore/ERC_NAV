/*
pkg:    wheels_commands
node:   NAV_motor_cmds
topics: 
        publish:    /NAV/absolute_encoders 
        subscribe:  /CS/nav_shutdown_cmds - /NAV/displacement  

description:    Check that all the motors are connected
                Send the commands of speed or position to one motors 
                Motors steering: control in position
                Motors driving: control in velocity

Function used from motors.hpp:  - connected()
                                - get_position_is() 
                                - set_velocity_ref() 
                                - set_position_ref() 
*/


#include <chrono>
#include <cmath>
#include <functional> 
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_set>


// #include "EposCmd.h"
#include "wheels_control/definition.hpp"


#include "wheels_control/motors.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/timer.hpp>
#include "std_msgs/msg/string.hpp"
#include "custom_msg/msg/motorcmds.hpp" 
#include "custom_msg/msg/wheelstatus.hpp"
#include "custom_msg/msg/statussteering.hpp"
#include "custom_msg/msg/motorstatus.hpp"

using namespace std::chrono_literals;

const int nb_wheels(4);
const _Float64 dist_wheels(1); 
const _Float64 limit_angle(0);
const _Float64 dt(0);
const _Float64 limit_variation_angle(0);
const _Float64 limit_variation_drive(0);


// int mode_deplacement =  0;
std::string mode_deplacement = "";
bool depassement_courrant = 0;
bool fault_state = false;

_Float64 motors_cmds[8];
bool safemode = true;
long current_motor_stat[NB_MOTORS]; 
int encoder_resolution;
std::vector<NAV_Motor> motors; 
struct gateway_struct *gateway;
std::vector<unsigned short> motor_ids = {
    FRONT_LEFT_DRIVE,
    FRONT_RIGHT_DRIVE,
    BACK_RIGHT_DRIVE,
    BACK_LEFT_DRIVE,
    FRONT_LEFT_STEER, 
    FRONT_RIGHT_STEER,
    BACK_RIGHT_STEER,
    BACK_LEFT_STEER};

std::unordered_set<int> DRIVING_MOTORS = {FRONT_LEFT_DRIVE, FRONT_RIGHT_DRIVE, BACK_RIGHT_DRIVE, BACK_LEFT_DRIVE};
std::unordered_set<int> STEERING_MOTORS = {FRONT_LEFT_STEER, FRONT_RIGHT_STEER, BACK_RIGHT_STEER, BACK_LEFT_STEER};

std::map<unsigned short, int> mot_id2msg_idx;
bool verbose = true;

void publish_motors_position();


class MotorCmds : public rclcpp::Node
{
  public:
    MotorCmds()
    : Node("NAV_motor_cmds"), count_(0)
    { 
        // add all motors
        int i = 0;
        bool homing;

        // this->declare_parameter("homing");

        if (this->get_parameter("homing", homing)) {
            RCLCPP_INFO(this->get_logger(), "Got homing_param: %s", homing ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get homing_param");
        }

        connect_motors(get_logger(), get_clock(), homing);

    
        pub_absolute_encoders = this->create_publisher<custom_msg::msg::Wheelstatus>(
            "/NAV/absolute_encoders", 1);

        pub_motor_status = this->create_publisher<custom_msg::msg::Motorstatus>(
            "/NAV/motor_status", 1);
            
        timer_=this->create_wall_timer(
            100ms, std::bind(&MotorCmds::motors_param_callback, this));

       
        sub_motors_displacement = this->create_subscription<custom_msg::msg::Motorcmds>(
            "NAV/displacement", 1, std::bind(&MotorCmds::motor_cmds_callback, this, std::placeholders::_1));


        sub_cmds_shutdown = this->create_subscription<std_msgs::msg::String>(
            "CS/nav_shutdown_cmds", 1, std::bind(&MotorCmds::callback_shutdown, this, std::placeholders::_1));
        
        destroy_sub_ = this->create_subscription<std_msgs::msg::String>("ROVER/NAV_status", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&MotorCmds::destroy_callback, this, std::placeholders::_1));


        RCLCPP_INFO(get_logger(), "END CONNEXION", 4);
        // Get the size of the vector
        std::size_t size = motors.size();

        // Print the size
        // std::cout << "[NAV_motors_debugging_node] The size of the motors vector is: " << size << std::endl;
        RCLCPP_INFO(get_logger(), "The size of the motors vector is:'%d'", size);
      
    }
    
    // destructeur
    ~MotorCmds()
    {
        for (auto motor = motors.begin(); motor != motors.end(); motor++)
        {
            if (motor->connected())
                motor->set_output_state(false);
        }
         close_gateway(gateway);
    }

    

  private:
    void callback_shutdown(std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Read :'%s'", msg->data);
        
        if (msg->data == "NAV_SHUTDOWN")   
        {
            throw std::runtime_error("Shutdown requested");
        }          

    }
                 



    void motor_cmds_callback(const custom_msg::msg::Motorcmds::SharedPtr msg)   
    {
        /*Manage the communication with the controllers to execute the desire speed*/

        // RCLCPP_INFO(get_logger(), " motor_cmds_callback", 4);
        
        
        mode_deplacement = msg->modedeplacement;

        motors_cmds[0] = msg->drive[0];
        motors_cmds[1] = msg->drive[1];
        motors_cmds[2] = msg->drive[2];
        motors_cmds[3] = msg->drive[3];

        motors_cmds[4] = msg->steer[0];
        motors_cmds[5] = msg->steer[1];
        motors_cmds[6] = msg->steer[2];
        motors_cmds[7] = msg->steer[3];

    

        for (auto motor = motors.begin(); motor != motors.end(); motor ++)
        {
            if ( motor->fault_state())
            {
                fault_state = true;
            }
            
        }

        if (fault_state)
        {   
            for (auto motor = motors.begin(); motor != motors.end(); motor++)
            {
                if (motor->connected())
                    motor->set_output_state(false);
            }
            close_gateway(gateway);
            motors.clear();

            connect_motors(get_logger(), get_clock(), true);
            fault_state = false;

            for (auto motor = motors.begin(); motor != motors.end(); motor++)
            {
                int id = motor->get_id();
                if ((id == FRONT_LEFT_STEER) || (id == FRONT_RIGHT_STEER)||(id == BACK_LEFT_STEER) || (id == BACK_RIGHT_STEER))
                {
                    //motor->set_position_ref(motors_cmds[id-1]); faire le mouvement - angle
                    // //homing pour tous 


                    // Define a vector to store parameter values
                    std::vector<int> driving_params(4, 0);  // Initialize with default values
                    std::vector<double> steering_params(4, 0.0);

                    // Load the parameters from the YAML file into the vectors
                    rclcpp::Parameter parameter;
                    

                    if (this->get_parameter("steering", parameter)) 
                    {
                        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                            steering_params = parameter.as_double_array();
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "Invalid parameter type for steering.");
                        }
                    // } else {
                    //     RCLCPP_ERROR(this->get_logger(), "Failed to retrieve steering parameters from YAML file.");

                    // }
                    for (size_t i = 0; i < 4; ++i) {

                        RCLCPP_INFO(this->get_logger(), "steering%d: %lf", i + 1, steering_params[i]);
                    }
                     for(auto motor = motors.begin(); motor != motors.end(); motor++)
                    {
                        if ((id == FRONT_LEFT_STEER) || (id == FRONT_RIGHT_STEER)||(id == BACK_LEFT_STEER) || (id == BACK_RIGHT_STEER))
                        {
                            motor->set_position_ref(-steering_params[id-5]);
                            motor->homing();

                        }

                    }

                }
               
                    
                }  else  if ((id == FRONT_LEFT_DRIVE) ||  (id == BACK_LEFT_DRIVE)||(id == FRONT_RIGHT_DRIVE) || (id == BACK_RIGHT_DRIVE))
                {
                    motor->set_velocity_ref(0);
                }                       

            }

        
        } else 
        {
                // Send commands to Maxon controllers
            for (auto motor = motors.begin(); motor != motors.end(); motor++)
            {
                
                int id = motor->get_id();            

                if (motor->connected())
                {
                    if(STEERING_MOTORS.count(id) > 0)
                    {
                    
                        if ((id == FRONT_LEFT_STEER) || (id == FRONT_RIGHT_STEER)||(id == BACK_LEFT_STEER) || (id == BACK_RIGHT_STEER))
                        {
                            // RCLCPP_INFO(get_logger(), "STEER MOTOR : '%f'",motors_cmds[id-1]);

                            motor->set_position_ref(motors_cmds[id-1]);
                        }
                    }

                    else if (DRIVING_MOTORS.count(id) > 0)
                    //if ((id == FRONT_LEFT_DRIVE) ||  (id == BACK_LEFT_DRIVE)||(id == FRONT_RIGHT_DRIVE) || (id == BACK_RIGHT_DRIVE))
                    {
                        
                        if ((id == FRONT_LEFT_DRIVE) ||  (id == BACK_LEFT_DRIVE)||(id == FRONT_RIGHT_DRIVE) || (id == BACK_RIGHT_DRIVE))
                        {
                            // RCLCPP_INFO(get_logger(), "FRONT_LEFT_DRIVE MOTOR : '%f'",motors_cmds[id-1]);
                            
                            motor->set_velocity_ref(motors_cmds[id-1]);
                        
                        }

                    }
                    
                    else
                    {
                        motor->set_velocity_ref(0);
                    }
                    
                }
            }
        }

    }


    void motors_param_callback()
    {

        auto message = custom_msg::msg::Wheelstatus();
        auto motor_status = custom_msg::msg::Motorstatus();
        for (auto motor = motors.begin(); motor != motors.end(); motor++)
        {
            int id = motor->get_id();
            if (motor->connected())
            {                
                message.state[id-1] = motor->fault_state();
                message.current[id-1] = motor->get_current_is();
                

                if (id >4)
                {
                    message.data[id-5] = motor->get_position_is();

                }
                else{
                    motor_status.driving_vel[id-1] = motor->get_velocity_is();
                    motor_status.driving_curr[id-1] = motor->get_current_is();

                }
            }

        }        

        pub_absolute_encoders->publish(message);
        pub_motor_status->publish(motor_status);

    }

    bool connect_motors(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, bool homing) const
    {
        void *gateway = open_gateway();
        rclcpp::Rate reconnect_rate(0.5);
        while (safemode && !gateway)
        {
            reconnect_rate.sleep();
            gateway = open_gateway();
            if (!gateway)
            {
                RCLCPP_ERROR_THROTTLE(logger, *clock, 1000, "CAN network gateway opening has failed !" , 4);

            }
        }
        int i = 0;
        
        RCLCPP_INFO(get_logger(), "START MOTOR DETECTION", 4);
        for (auto id = motor_ids.begin(); id != motor_ids.end(); id++, i++)
        {
            switch (*id)
            {
                case FRONT_LEFT_DRIVE:
                    motors.push_back(NAV_Motor(gateway, *id, MT_EC_BLOCK_COMMUTATED_MOTOR, OMD_PROFILE_VELOCITY_MODE, false));
                    if (motors.data()->connected())
                    {
                        RCLCPP_INFO_ONCE(get_logger(), "FRONT_LEFT_DRIVE connected",  4);
                    }
                    break;

                case FRONT_RIGHT_DRIVE:
                    motors.push_back(NAV_Motor(gateway, *id, MT_EC_BLOCK_COMMUTATED_MOTOR, OMD_PROFILE_VELOCITY_MODE, false));
                    if (motors.data()->connected())
                    {
                        RCLCPP_INFO_ONCE(get_logger(), "FRONT_RIGHT_DRIVE connected",  4);
                    }
                    break;

                case BACK_RIGHT_DRIVE:
                    motors.push_back(NAV_Motor(gateway, *id, MT_EC_BLOCK_COMMUTATED_MOTOR, OMD_PROFILE_VELOCITY_MODE, false));
                    if (motors.data()->connected())
                    {
                        RCLCPP_INFO_ONCE(get_logger(), "BACK_RIGHT_DRIVE connected",  4);
                    }
                    break;

                case BACK_LEFT_DRIVE:
                    motors.push_back(NAV_Motor(gateway, *id, MT_EC_BLOCK_COMMUTATED_MOTOR, OMD_PROFILE_VELOCITY_MODE, false));
                    if (motors.data()->connected())
                    {
                        RCLCPP_INFO_ONCE(get_logger(), "BACK_LEFT_DRIVE connected",  4);
                    }
                    break;

                case FRONT_LEFT_STEER:
                    motors.push_back(NAV_Motor(gateway, *id, MT_EC_SINUS_COMMUTATED_MOTOR, OMD_PROFILE_POSITION_MODE, homing));
                    if (motors.data()->connected())
                    {
                        RCLCPP_INFO_ONCE(get_logger(), "FRONT_LEFT_STEER connected",  4);
                    }
                    break;

                case FRONT_RIGHT_STEER:
                    motors.push_back(NAV_Motor(gateway, *id, MT_EC_SINUS_COMMUTATED_MOTOR, OMD_PROFILE_POSITION_MODE, homing));
                    if (motors.data()->connected())
                    {
                        RCLCPP_INFO_ONCE(get_logger(), "FRONT_RIGHT_STEER connected",  4);
                    }
                    break;

                case BACK_RIGHT_STEER:
                    motors.push_back(NAV_Motor(gateway, *id, MT_EC_SINUS_COMMUTATED_MOTOR, OMD_PROFILE_POSITION_MODE, homing));
                    if (motors.data()->connected())
                    {
                        RCLCPP_INFO_ONCE(get_logger(), "BACK_RIGHT_STEER connected",  4);
                    }
                    break;

                case BACK_LEFT_STEER:
                    motors.push_back(NAV_Motor(gateway, *id, MT_EC_SINUS_COMMUTATED_MOTOR, OMD_PROFILE_POSITION_MODE, homing));
                    if (motors.data()->connected())
                    {
                        RCLCPP_INFO_ONCE(get_logger(), "BACK_LEFT_STEER connected", 4);
                    }
                    break;

                default:
                    RCLCPP_INFO_ONCE(get_logger(), "WARNING DEFAULT: NO MOTOR ", 4);
            }

        }
        RCLCPP_INFO(get_logger(), "END MOTOR DETECTION", 4);
        for (auto motor = motors.begin(); motor != motors.end(); motor++)
        {
            int id = motor->get_id();
            if (motor->connected())
            {
                motor->set_output_state(true);
                if ((id == FRONT_LEFT_STEER) ||  (id == BACK_LEFT_STEER)||(id == FRONT_RIGHT_STEER) ||
                        (id == BACK_RIGHT_STEER))
                {
                    
                    std::cout << "[NAV_motors_debugging_node]: motor steer " << id <<" : position "<<  motor->get_position_is() <<  std::endl;
                    motor->set_position_ref(0);
                    
                }
                else if (id== FRONT_LEFT_STEER)
                {
                    encoder_resolution = motor->get_encoder_pulse();
                    std::cout << "[NAV_motors_debugging_node]: motor steer " << id <<" : encoder pulse"<<  motor->get_encoder_pulse() <<  std::endl;
                }
                else if (((id == FRONT_LEFT_DRIVE) ||  (id == BACK_LEFT_DRIVE)||(id == FRONT_RIGHT_DRIVE) || (id == BACK_RIGHT_DRIVE)) )
                {   
                    std::cout << "[NAV_motors_debugging_node]: motor drive " << id <<" : position "<<  motor->get_position_is() <<  std::endl;
                }
            }

        }
    
    }

      void destroy_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if(msg->data == "abort") rclcpp::shutdown();
    }



    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Publisher<custom_msg::msg::Wheelstatus>::SharedPtr pub_absolute_encoders;
    rclcpp::Publisher<custom_msg::msg::Motorstatus>::SharedPtr pub_motor_status;        
    rclcpp::Subscription<custom_msg::msg::Motorcmds>::SharedPtr sub_motors_displacement;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmds_shutdown;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr destroy_sub_;

    size_t count_;
    // rclcpp::WallTimer<timer_callback<void()>> timer2_;


};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);


    try
    {
        rclcpp::spin(std::make_shared<MotorCmds>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(rclcpp::get_logger("NAV_motor_cmds"), "WARNING: node MotorCmds ended", 4);
    }


    rclcpp::shutdown();
    return 0;

}

