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
#include <signal.h>
#include <unistd.h>

#include "wheels_control/definition.hpp"
#include "wheels_control/motors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <rclcpp/timer.hpp>
#include "std_msgs/msg/string.hpp"
#include "custom_msg/msg/motorcmds.hpp"
#include "custom_msg/msg/wheelstatus.hpp"
#include "custom_msg/msg/statussteering.hpp"
#include "custom_msg/msg/motor_status.hpp"
#include "std_srvs/srv/set_bool.hpp"


using namespace std::chrono_literals;

//const _Float64 dt(0);

std::string mode_deplacement = ""; // to be removed if not using check on mode
bool fault_state = false; // ??

_Float64 motors_cmds[8];
bool safemode = true; // needed?

int encoder_resolution; // NEEDED?
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

void publish_motors_position();

class MotorCmdsLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
    MotorCmdsLifecycle()
        : rclcpp_lifecycle::LifecycleNode("NAV_motor_cmds"), count_(0)
    {
        bool homing;
        this->declare_parameter("homing", true);

        if (this->get_parameter("homing", homing))
        {
            RCLCPP_INFO(this->get_logger(), "Got homing_param: %s", homing ? "true" : "false");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get homing_param");
        }

        this->homing = homing;
    }

    // destructeur
    ~MotorCmdsLifecycle()
    {
        for (auto motor = motors.begin(); motor != motors.end(); motor++)
        {
            if (motor->connected())
                motor->set_output_state(false);
        }
        close_gateway(gateway);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
    {
        bool connected = connect_motors(get_logger(), get_clock(), this->homing);

        if (!connected)
        {
            RCLCPP_ERROR(get_logger(), "Failed to connect to navigation motors");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        this->homing = false;

        // auto timer_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // auto sub_service_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&MotorCmdsLifecycle::motors_param_callback, this)
        );

        auto qos = rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort();
        
        pub_motor_nav_status = this->create_publisher<custom_msg::msg::MotorStatus>(
            "/NAV/motor_nav_status", qos);


        // subscription into sub_service_group
        // rclcpp::SubscriptionOptions sub_opts;
        // sub_opts.callback_group = sub_service_group;
        sub_motors_displacement = this->create_subscription<custom_msg::msg::Motorcmds>(
            "/NAV/displacement", 1,
            std::bind(&MotorCmdsLifecycle::motor_cmds_callback, this, std::placeholders::_1)
        );

        reset_nav_motors_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/CS/ResetNavMotors",
            std::bind(&MotorCmdsLifecycle::handle_reset_nav_motors, this,
                        std::placeholders::_1, std::placeholders::_2)
        );

        reset_home_nav_motors_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/CS/ResetHomeNavMotors",
            std::bind(&MotorCmdsLifecycle::handle_reset_home_nav_motors, this,
                        std::placeholders::_1, std::placeholders::_2)
        );
            
        // Get the size of the vector
        std::size_t size = motors.size();
        RCLCPP_INFO(get_logger(), "The size of the motors vector is:'%d'", size);
        RCLCPP_INFO(get_logger(), "----> NAV MOTORS CONFIGURED <----");

        current_faulty_motors.resize(motors.size(), false);

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        sub_motors_displacement.reset();

        disconnect_motors();

        RCLCPP_INFO(get_logger(), "Unconfigure Navigation Motors");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_activate() called (NOT IMPLEMENTED)");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_deactivate() called (NOT IMPLEMENTED)");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        sub_motors_displacement.reset();
        disconnect_motors();

        RCLCPP_WARN(get_logger(), "Navigation Motors SHUTDOWN");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        RCLCPP_INFO(get_logger(), "on_shutdown() called");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void handle_reset_nav_motors(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        const std_srvs::srv::SetBool::Response::SharedPtr response){

        if(request->data){
            RCLCPP_INFO(get_logger(), "Received request to reset navigation motor.");
            //detect how many motors need resetting
            unsigned int num_faulty_before = 0;
            for (bool is_faulty: current_faulty_motors){
                if(is_faulty){
                    num_faulty_before++;
                }
            }

            for (auto motor = motors.begin(); motor != motors.end(); motor++){
                int id = motor->get_id();
                unsigned int error_code = 0;

                if(current_faulty_motors[id-1] == true){
                    bool cleared_fault = motor->clear_fault();
                    if (error_code != 0){
                        RCLCPP_ERROR(get_logger(), "ERR : could not reset fault for motor %d", id-1);
                    }else{
                        current_faulty_motors[id-1] = false;
                    }
                }
            }
            unsigned int num_faulty_after = 0;
            for (bool is_faulty: current_faulty_motors){
                if(is_faulty){
                    num_faulty_after++;
                }
            }

            if(num_faulty_after == 0){
                response->success = true;
                response->message = "ERR: Nav motors reset succesfully!";
            }else{
                RCLCPP_WARN(get_logger(), "%d motors failed to reset.", num_faulty_after);
                response->success = false;
                response->message = "ERR: at least one of the motors was not successfully reset";
            }
            
        } else {
            RCLCPP_WARN(get_logger(), "Nav Motor Reset request denied.");
            response->success = false;
            response->message = "Nav Motor Reset request denied.";
        }
    }
    
    void handle_reset_home_nav_motors(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        const std_srvs::srv::SetBool::Response::SharedPtr response){

        if(request->data){
            for (auto motor = motors.begin(); motor != motors.end(); motor++){
                int id = motor->get_id();
                unsigned int error_code = 0;
                bool successfully_homed = motor->homing();
                if(successfully_homed){
                    response->success = true;
                    response->message = "Successfully home all motors";
                }else{
                    RCLCPP_WARN(get_logger(), "ERR : Failed to home all motors");
                    response->success = false;
                    response->message = "ERR: Could not home all motors";
                }
            }
        }else{
            RCLCPP_WARN(get_logger(), "Nav Motor Homing Reset request denied.");
            response->success = false;
            response->message = "Nav Motor Homing Reset request denied.";
        }
    }

    void motors_param_callback(){ //right now runs at 10Hz
        static unsigned int counter = 0;
        counter++;
        bool check_faults = (counter >= 20);
        if(check_faults){
            counter = 0;
        }

        auto message_nav = custom_msg::msg::MotorStatus();

        for (auto motor = motors.begin(); motor != motors.end(); motor++){
            
            int id = motor->get_id();
            unsigned int error_code = 0;
            bool debug_verbose = false;

            if (motor->connected()){
                //Put the motor current callback at a lower requency because it severly diminishes the publishing rate
                message_nav.state[id-1] = true;
                
                if(check_faults){
                    //message_nav.current[id-1] = (double)motor->get_current_is();
                    message_nav.average_current[id-1] = (double)motor->get_current_is_averaged();
                }
                // IDs [0,1,2,3] are the nodes for the driving
                // IDs [4,5,6,7] are the nodes for the steering
                if (id > 4){
                    message_nav.position[id-5] = (double)motor->get_position_is(); //takes max 6ms
                }
                else{
                    //this returns 1800 (approx) which is correct for the max speed.
                    //to get the actual value we need to divide by the gear ration 1:53
                    //which gives us again the 33.3 rpm
                    message_nav.velocity[id-1] = motor->get_velocity_is(); //takes max 6ms
                }
            }else{
                message_nav.state[id-1] = false;
            }

            if(check_faults){
                //RCLCPP_INFO(get_logger(), "checking for faults");
                bool has_fault = motor->is_faulty(debug_verbose);
                message_nav.fault_state[id-1] = has_fault;
                current_faulty_motors[id-1] = has_fault;
            }
        }

        pub_motor_nav_status->publish(message_nav);
    }

    void motor_cmds_callback(const custom_msg::msg::Motorcmds::SharedPtr msg)
    {
        /*Manage the communication with the controllers to execute the desire speed*/

        mode_deplacement = msg->modedeplacement;
        //at max speed it sends in abs value 1800 --> gear ration 1:53 --> 1800/53=33.3rpm
        //which is what we actually measure in real life with a timer hence it is correct
        motors_cmds[0] = msg->drive[0];
        //RCLCPP_INFO(get_logger(), "drive0 sent: %f", motors_cmds[0]);
        motors_cmds[1] = msg->drive[1];
        //RCLCPP_INFO(get_logger(), "drive1 sent: %f", motors_cmds[1]);
        motors_cmds[2] = msg->drive[2];
        //RCLCPP_INFO(get_logger(), "drive2 sent: %f", motors_cmds[2]);
        motors_cmds[3] = msg->drive[3];
        //RCLCPP_INFO(get_logger(), "drive3 sent: %f", motors_cmds[3]);

        motors_cmds[4] = msg->steer[0];
        //RCLCPP_INFO(get_logger(), "lifecycle steering 4 sent: %f", motors_cmds[4]);

        motors_cmds[5] = msg->steer[1];
        //RCLCPP_INFO(get_logger(), "lifecyclesteering 5 sent: %f", motors_cmds[5]);

        motors_cmds[6] = msg->steer[2];
        //RCLCPP_INFO(get_logger(), "lifecyclesteering 6 sent: %f", motors_cmds[6]);

        motors_cmds[7] = msg->steer[3];
        //RCLCPP_INFO(get_logger(), "lifecyclesteering 7 sent: %f", motors_cmds[7]);


        do
        {
            for (auto motor = motors.begin(); motor != motors.end(); motor++)
            {
                unsigned int error_code = 0;
                bool is_fault = motor->fault_state(&error_code);

                if (error_code != 0)
                {
                    RCLCPP_ERROR(get_logger(), "Error Navigation Motors Detected");
                    this->cleanup();
                    return;
                }

                if (is_fault)
                {
                    RCLCPP_ERROR(get_logger(), "FAULT STATE Navigation Motors Detected");
                    this->cleanup();
                    return;
                }
            }
            // Send commands to Maxon controllers
            for (auto motor = motors.begin(); motor != motors.end(); motor++)
            {

                int id = motor->get_id();

                if (motor->connected())
                {
                    if (STEERING_MOTORS.count(id) > 0)
                    {

                        if ((id == FRONT_LEFT_STEER) || (id == FRONT_RIGHT_STEER) || (id == BACK_LEFT_STEER) || (id == BACK_RIGHT_STEER))
                        {
                            // RCLCPP_INFO(get_logger(), "STEER MOTOR : '%f'",motors_cmds[id-1]);

                            motor->set_position_ref(motors_cmds[id - 1]);
                        }
                    }

                    else if (DRIVING_MOTORS.count(id) > 0)
                    // if ((id == FRONT_LEFT_DRIVE) ||  (id == BACK_LEFT_DRIVE)||(id == FRONT_RIGHT_DRIVE) || (id == BACK_RIGHT_DRIVE))
                    {

                        if ((id == FRONT_LEFT_DRIVE) || (id == BACK_LEFT_DRIVE) || (id == FRONT_RIGHT_DRIVE) || (id == BACK_RIGHT_DRIVE))
                        {
                            //RCLCPP_INFO(get_logger(), "PRINT DRIVE MOTOR : '%f'",motors_cmds[id-1]);

                            motor->set_velocity_ref(motors_cmds[id - 1]);
                        }
                    }

                    else
                    {
                        motor->set_velocity_ref(0);
                    }
                }
            }
        } while (fault_state);
    }

    static void interrupt_handler(int s)
    {
        exit(1);
    }

    bool connect_motors(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, bool homing) const
    {
        void *gateway = open_gateway();
        rclcpp::Rate reconnect_rate(0.5);

        // Make sure that the check is interrupted by a CTRL+C
        struct sigaction sigIntHandler;
        sigIntHandler.sa_handler = interrupt_handler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;
        sigaction(SIGINT, &sigIntHandler, NULL);

        // Try to open the gateway for motor communication for 30 seconds
        int MAX_TRY = 30;
        while (safemode && !gateway && rclcpp::ok() && MAX_TRY--)
        {
            reconnect_rate.sleep();
            gateway = open_gateway();
            if (!gateway)
            {
                RCLCPP_WARN_THROTTLE(logger, *clock, 1000, "CAN network gateway opening has failed !", 4);
            }
        }

        // If the gateway is still not open, return false
        if (!gateway)
        {
            return false;
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
                    RCLCPP_INFO_ONCE(get_logger(), "FRONT_LEFT_DRIVE connected", 4);
                }
                break;

            case FRONT_RIGHT_DRIVE:
                motors.push_back(NAV_Motor(gateway, *id, MT_EC_BLOCK_COMMUTATED_MOTOR, OMD_PROFILE_VELOCITY_MODE, false));
                if (motors.data()->connected())
                {
                    RCLCPP_INFO_ONCE(get_logger(), "FRONT_RIGHT_DRIVE connected", 4);
                }
                break;

            case BACK_RIGHT_DRIVE:
                motors.push_back(NAV_Motor(gateway, *id, MT_EC_BLOCK_COMMUTATED_MOTOR, OMD_PROFILE_VELOCITY_MODE, false));
                if (motors.data()->connected())
                {
                    RCLCPP_INFO_ONCE(get_logger(), "BACK_RIGHT_DRIVE connected", 4);
                }
                break;

            case BACK_LEFT_DRIVE:
                motors.push_back(NAV_Motor(gateway, *id, MT_EC_BLOCK_COMMUTATED_MOTOR, OMD_PROFILE_VELOCITY_MODE, false));
                if (motors.data()->connected())
                {
                    RCLCPP_INFO_ONCE(get_logger(), "BACK_LEFT_DRIVE connected", 4);
                }
                break;

            case FRONT_LEFT_STEER:
                motors.push_back(NAV_Motor(gateway, *id, MT_EC_SINUS_COMMUTATED_MOTOR, OMD_PROFILE_POSITION_MODE, homing));
                if (motors.data()->connected())
                {
                    RCLCPP_INFO_ONCE(get_logger(), "FRONT_LEFT_STEER connected", 4);
                }
                break;

            case FRONT_RIGHT_STEER:
                motors.push_back(NAV_Motor(gateway, *id, MT_EC_SINUS_COMMUTATED_MOTOR, OMD_PROFILE_POSITION_MODE, homing));
                if (motors.data()->connected())
                {
                    RCLCPP_INFO_ONCE(get_logger(), "FRONT_RIGHT_STEER connected", 4);
                }
                break;

            case BACK_RIGHT_STEER:
                motors.push_back(NAV_Motor(gateway, *id, MT_EC_SINUS_COMMUTATED_MOTOR, OMD_PROFILE_POSITION_MODE, homing));
                if (motors.data()->connected())
                {
                    RCLCPP_INFO_ONCE(get_logger(), "BACK_RIGHT_STEER connected", 4);
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
                if ((id == FRONT_LEFT_STEER) || (id == BACK_LEFT_STEER) || (id == FRONT_RIGHT_STEER) ||
                    (id == BACK_RIGHT_STEER))
                {

                    std::cout << "[NAV_motors_debugging_node]: motor steer " << id << " : position " << motor->get_position_is() << std::endl;
                    motor->set_position_ref(0);
                }
                else if (id == FRONT_LEFT_STEER)
                {
                    encoder_resolution = motor->get_encoder_pulse();
                    std::cout << "[NAV_motors_debugging_node]: motor steer " << id << " : encoder pulse" << motor->get_encoder_pulse() << std::endl;
                }
                else if (((id == FRONT_LEFT_DRIVE) || (id == BACK_LEFT_DRIVE) || (id == FRONT_RIGHT_DRIVE) || (id == BACK_RIGHT_DRIVE)))
                {
                    std::cout << "[NAV_motors_debugging_node]: motor drive " << id << " : position " << motor->get_position_is() << std::endl;
                }
            }
            else
            {
                return false;
            }
        }

        return true;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msg::msg::Motorcmds>::SharedPtr sub_motors_displacement;
    rclcpp::Publisher<custom_msg::msg::MotorStatus>::SharedPtr pub_motor_nav_status;

    size_t count_;
    bool homing;

private:
    void disconnect_motors()
    {
        for (auto motor = motors.begin(); motor != motors.end(); motor++)
        {
            if (motor->connected())
                motor->set_output_state(false);
        }
        RCLCPP_INFO_ONCE(get_logger(), "Disconnect motors called");

        close_gateway(gateway);
        motors.clear();
    }

    std::vector<bool> current_faulty_motors;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_nav_motors_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_home_nav_motors_service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::executors::MultiThreadedExecutor exe;
        std::shared_ptr<MotorCmdsLifecycle> motor_cmds_node =
            std::make_shared<MotorCmdsLifecycle>();

        exe.add_node(motor_cmds_node->get_node_base_interface());
        exe.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(rclcpp::get_logger("NAV_motor_cmds"), "WARNING: node MotorCmds ended", 4);
    }

    rclcpp::shutdown();
    return 0;
}