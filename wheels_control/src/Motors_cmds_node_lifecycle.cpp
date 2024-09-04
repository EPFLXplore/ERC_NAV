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
#include "custom_msg/msg/motorstatus.hpp"

using namespace std::chrono_literals;

const int nb_wheels(4);
const _Float64 dist_wheels(1);
const _Float64 limit_angle(0);
const _Float64 dt(0);
const _Float64 limit_variation_angle(0);
const _Float64 limit_variation_drive(0);

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

class MotorCmdsLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
    MotorCmdsLifecycle()
        : rclcpp_lifecycle::LifecycleNode("NAV_motor_cmds"), count_(0)
    {
        // add all motors
        int i = 0;
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

        pub_absolute_encoders = this->create_publisher<custom_msg::msg::Wheelstatus>(
            "/NAV/absolute_encoders", 1);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&MotorCmdsLifecycle::motors_param_callback, this));

        sub_motors_displacement = this->create_subscription<custom_msg::msg::Motorcmds>(
            "NAV/displacement", 1, std::bind(&MotorCmdsLifecycle::motor_cmds_callback, this, std::placeholders::_1));

        pub_motor_status = this->create_publisher<custom_msg::msg::Motorstatus>(
            "/NAV/motor_status", 1);

        RCLCPP_INFO(get_logger(), "END CONNEXION", 4);

        // Get the size of the vector
        std::size_t size = motors.size();
        RCLCPP_INFO(get_logger(), "The size of the motors vector is:'%d'", size);
        RCLCPP_INFO(get_logger(), "Nav motors configured");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
        pub_absolute_encoders.reset();
        timer_.reset();
        sub_motors_displacement.reset();
        sub_cmds_shutdown.reset();
        destroy_sub_.reset();

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
        pub_absolute_encoders.reset();
        timer_.reset();
        sub_motors_displacement.reset();
        sub_cmds_shutdown.reset();
        destroy_sub_.reset();
        disconnect_motors();

        RCLCPP_WARN(get_logger(), "Navigation Motors SHUTDOWN");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        RCLCPP_INFO(get_logger(), "on_shutdown() called");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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
                            // RCLCPP_INFO(get_logger(), "FRONT_LEFT_DRIVE MOTOR : '%f'",motors_cmds[id-1]);

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

    void motors_param_callback()
    {
        auto message = custom_msg::msg::Wheelstatus();
        auto motor_status = custom_msg::msg::Motorstatus();
        for (auto motor = motors.begin(); motor != motors.end(); motor++)
        {
            int id = motor->get_id();
            if (motor->connected())
            {
                unsigned int error_code = 0;
                message.state[id - 1] = motor->fault_state(&error_code);
                message.current[id - 1] = motor->get_current_is();

                if (error_code != 0)
                {
                    RCLCPP_ERROR(get_logger(), "Error Navigation Motors Detected");
                    this->cleanup();
                    return;
                }

                if (id > 4)
                {
                    message.data[id - 5] = motor->get_position_is();
                }
                else
                {
                    motor_status.driving_vel[id - 1] = motor->get_velocity_is();
                    motor_status.driving_curr[id - 1] = motor->get_current_is();
                }
            }
        }

        pub_absolute_encoders->publish(message);
        pub_motor_status->publish(motor_status);
    }

    static void interrupt_handler(int s)
    {
        // RCLCPP_ERROR(get_logger(), "NAV_motor_cmds: Caught signal %d\n", s);
        // rclcpp::shutdown();
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
    rclcpp::Publisher<custom_msg::msg::Wheelstatus>::SharedPtr pub_absolute_encoders;
    rclcpp::Publisher<custom_msg::msg::Motorstatus>::SharedPtr pub_motor_status;
    rclcpp::Subscription<custom_msg::msg::Motorcmds>::SharedPtr sub_motors_displacement;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmds_shutdown;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr destroy_sub_;

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
        close_gateway(gateway);
        motors.clear();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::executors::SingleThreadedExecutor exe;

        std::shared_ptr<MotorCmdsLifecycle> motor_cmds_node =
            std::make_shared<MotorCmdsLifecycle>();

        exe.add_node(motor_cmds_node->get_node_base_interface());

        exe.spin();
        // rclcpp::spin(std::make_shared<MotorCmdsLifecycle>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(rclcpp::get_logger("NAV_motor_cmds"), "WARNING: node MotorCmds ended", 4);
    }

    rclcpp::shutdown();
    return 0;
}