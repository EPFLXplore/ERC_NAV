/*
pkg:    wheels_control
node:   NAV_motor_cmds
topics:
        publish:    /NAV/motor_nav_status
        subscribe:  /NAV/displacement
services:
        /CS/ResetNavMotors
        /CS/ResetHomeNavMotors
description:    Lifecycle node that connects and monitors the navigation motors.
                Publishes motor state, position, velocity, current, and fault status.
                Applies displacement commands to each motor:
                - steering motors are controlled in position
                - drive motors are controlled in velocity
                Detects faults, attempts recovery, and skips faulty motors and pairs.
*/

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
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

std::string mode_deplacement = "";

_Float64 motors_cmds[8];
bool safemode = true;

std::vector<NAV_Motor> motors;
struct gateway_struct *gateway;

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

        // Timer gets its own group; subscription and services share a second group.
        // The can_mutex_ serializes actual CAN bus access between the two groups.
        timer_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_group_    = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&MotorCmdsLifecycle::motors_param_callback, this),
            timer_group_
        );

        auto qos = rclcpp::QoS{rclcpp::KeepLast{1}}.best_effort();

        pub_motor_nav_status = this->create_publisher<custom_msg::msg::MotorStatus>(
            "/NAV/motor_nav_status", qos);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_group_;
        sub_motors_displacement = this->create_subscription<custom_msg::msg::Motorcmds>(
            "/NAV/displacement", qos,
            std::bind(&MotorCmdsLifecycle::motor_cmds_callback, this, std::placeholders::_1),
            sub_opts
        );

        reset_nav_motors_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/CS/ResetNavMotors",
            std::bind(&MotorCmdsLifecycle::handle_reset_nav_motors, this,
                        std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            sub_group_
        );

        reset_home_nav_motors_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/CS/ResetHomeNavMotors",
            std::bind(&MotorCmdsLifecycle::handle_reset_home_nav_motors, this,
                        std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            sub_group_
        );
            
        // Get the size of the vector
        std::size_t size = motors.size();
        RCLCPP_INFO(get_logger(), "The size of the motors vector is:'%zu'", size);
        RCLCPP_INFO(get_logger(), "----> NAV MOTORS CONFIGURED <----");

        current_faulty_motors.resize(motors.size(), false);
        fault_retry_counts.resize(motors.size(), 0);
        last_fault_log_times.resize(motors.size());

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
        std::lock_guard<std::mutex> lock(can_mutex_);

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
                const MotorLayout* layout = motor_layout_from_can_id(id);
                if (layout == nullptr) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "Unknown CAN node ID %d in reset service", id);
                    continue;
                }
                int idx = internal_can_index(*layout);

                if(current_faulty_motors[idx]){
                    bool cleared = motor->clear_fault();
                    if (!cleared || motor->is_faulty(false) || !motor->set_output_state(true)){
                        RCLCPP_ERROR(get_logger(), "ERR : could not reset fault for motor %d", id);
                    }else{
                        current_faulty_motors[idx] = false;
                        fault_retry_counts[idx] = 0;
                        RCLCPP_INFO(get_logger(), "Motor %d: fault cleared via service", id);
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
        std::lock_guard<std::mutex> lock(can_mutex_);

        if(request->data){
            for (auto motor = motors.begin(); motor != motors.end(); motor++){
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

    void motors_param_callback(){
        std::lock_guard<std::mutex> lock(can_mutex_);
        static unsigned int counter = 0;
        counter++;
        bool check_faults = (counter >= 20);
        if(check_faults){
            counter = 0;
        }

        auto message_nav = custom_msg::msg::MotorStatus();

        for (auto motor = motors.begin(); motor != motors.end(); motor++){
            
            int id = motor->get_id(); // gets the CAN node ID of the motor
            const MotorLayout* layout = motor_layout_from_can_id(id);
            if (layout == nullptr) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Unknown CAN node ID %d in motors_param_callback", id);
                continue;
            }
            const int can_idx = internal_can_index(*layout);
            const int status_idx = motor_status_index(*layout);
            bool debug_verbose = false;
            double measured_velocity = 0.0;
            unsigned int velocity_read_error = 0;
            message_nav.fault_state[status_idx] = current_faulty_motors[can_idx];

            if (motor->connected()){
                //Put the motor current callback at a lower requency because it severly diminishes the publishing rate
                message_nav.state[status_idx] = true;
                
                if(check_faults){
                    //message_nav.current[status_idx] = (double)motor->get_current_is();
                    message_nav.average_current[status_idx] = (double)motor->get_current_is_averaged(); // goes from 16hz to 12hz
                }
                if (is_steer_motor(*layout)){
                    message_nav.position[layout->corner] = (double)motor->get_position_is(); //takes max 6ms
                }
                else{
                    //this returns 1800 (approx) which is correct for the max speed.
                    //to get the actual value we need to divide by the gear ration 1:53
                    //which gives us again the 33.3 rpm
                    measured_velocity = motor->get_velocity_is(&velocity_read_error); //takes max 6ms
                    message_nav.velocity[layout->corner] = measured_velocity;
                }
            }else{
                message_nav.state[status_idx] = false;
            }

            if(check_faults){
                //RCLCPP_INFO(get_logger(), "checking for faults");
                bool has_fault = motor->is_faulty(debug_verbose);
                message_nav.fault_state[status_idx] = has_fault;
                current_faulty_motors[can_idx] = has_fault;
                if (has_fault) {
                    log_motor_fault_throttled(*motor, *layout, "fault reported during periodic status check");
                }
            }

            // if (layout->can_id == FRONT_RIGHT_DRIVE)
            // {
            //     const bool velocity_read_ok = motor->connected() && velocity_read_error == 0;
            //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            //         "FRONT_RIGHT_DRIVE status | can_id=%d | connected=%s | fault=%s | retry=%d/%d | command_ref=%.2f | measured_velocity=%.2f | velocity_read_ok=%s | velocity_read_error=0x%X",
            //         layout->can_id,
            //         motor->connected() ? "true" : "false",
            //         message_nav.fault_state[status_idx] ? "true" : "false",
            //         fault_retry_counts[can_idx],
            //         MAX_FAULT_RETRIES,
            //         motors_cmds[can_idx],
            //         measured_velocity,
            //         velocity_read_ok ? "true" : "false",
            //         velocity_read_error);
            // }
        }

        pub_motor_nav_status->publish(message_nav);
    }

    void motor_cmds_callback(const custom_msg::msg::Motorcmds::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(can_mutex_);

        mode_deplacement = msg->modedeplacement;
        //at max speed it sends in abs value 1800 --> gear ration 1:53 --> 1800/53=33.3rpm
        //which is what we actually measure in real life with a timer hence it is correct

        for (const auto& layout : MOTOR_LAYOUT) {
            const int can_idx = internal_can_index(layout);
            motors_cmds[can_idx] = is_drive_motor(layout)
                ? msg->drive[layout.corner]
                : msg->steer[layout.corner];
        }


        // --- Per-motor fault detection and recovery ---
        for (auto motor = motors.begin(); motor != motors.end(); motor++)
        {
            int id = motor->get_id();
            const MotorLayout* layout = motor_layout_from_can_id(id);
            if (layout == nullptr) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Unknown CAN node ID %d in fault recovery loop", id);
                continue;
            }
            int idx = internal_can_index(*layout);

            if (current_faulty_motors[idx])
            {
                bool restored = false;
                if (!motor->connected())
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "Motor %d: marked faulty and disconnected; attempting reconnect", id);
                    motor->reconnect();
                }

                unsigned int recovery_error_code = 0;
                const bool still_in_fault = motor->connected()
                    ? motor->fault_state(&recovery_error_code)
                    : true;

                if (recovery_error_code != 0)
                {
                    if (motor->reconnect())
                    {
                        recovery_error_code = 0;
                        const bool fault_after_reconnect = motor->fault_state(&recovery_error_code);
                        if (recovery_error_code == 0 && (!fault_after_reconnect || motor->clear_fault()))
                        {
                            restored = !motor->is_faulty(false) && motor->set_output_state(true);
                        }
                    }
                }
                else if (!still_in_fault || motor->clear_fault())
                {
                    restored = !motor->is_faulty(false) && motor->set_output_state(true);
                }

                if (restored)
                {
                    current_faulty_motors[idx] = false;
                    fault_retry_counts[idx] = 0;
                    RCLCPP_WARN(get_logger(), "Motor %d: fault cleared, reconnected, resuming", id);
                }
                else
                {
                    if (fault_retry_counts[idx] < MAX_FAULT_RETRIES)
                    {
                        fault_retry_counts[idx]++;
                    }
                    log_motor_fault_throttled(
                        *motor,
                        *layout,
                        "fault recovery still active; clear/reconnect attempt failed",
                        recovery_error_code);
                }
                continue;
            }

            unsigned int error_code = 0;
            bool is_fault = motor->fault_state(&error_code);

            if (error_code != 0)
            {
                log_motor_fault_throttled(
                    *motor,
                    *layout,
                    "fault-state read failed; skipping command cycle",
                    error_code);
                continue;
            }

            if (is_fault)
            {
                current_faulty_motors[idx] = true;
                fault_retry_counts[idx] = 0;
                log_motor_fault_throttled(
                    *motor,
                    *layout,
                    "FAULT detected; attempting recovery",
                    0,
                    true);

                bool cleared = motor->clear_fault();
                if (cleared && !motor->is_faulty(false) && motor->set_output_state(true))
                {
                    current_faulty_motors[idx] = false;
                    RCLCPP_WARN(get_logger(), "Motor %d: fault cleared immediately", id);
                }
                else
                {
                    fault_retry_counts[idx]++;
                }
            }
        }

        // Escalate: if too many motors are permanently faulted, shut down
        int unrecoverable_count = 0;
        for (size_t i = 0; i < motors.size(); i++)
        {
            if (current_faulty_motors[i] && fault_retry_counts[i] >= MAX_FAULT_RETRIES)
                unrecoverable_count++;
        }
        if (unrecoverable_count > 2)
        {
            RCLCPP_FATAL(get_logger(), "%d motors unrecoverable, shutting down", unrecoverable_count);
            this->cleanup();
            return;
        }

        // --- Send commands, skipping faulty motors and their wheel pair ---
        for (auto motor = motors.begin(); motor != motors.end(); motor++)
        {
            int id = motor->get_id();
            const MotorLayout* layout = motor_layout_from_can_id(id);
            if (layout == nullptr) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Unknown CAN node ID %d in command send loop", id);
                continue;
            }
            int idx = internal_can_index(*layout);

            // if (!motor->connected() || current_faulty_motors[idx])
            // {
            //     if (layout->can_id == FRONT_RIGHT_DRIVE)
            //     {
            //         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
            //             "FRONT_RIGHT_DRIVE command skipped | can_id=%d | connected=%s | fault=%s | retry=%d/%d | command_ref=%.2f",
            //             layout->can_id,
            //             motor->connected() ? "true" : "false",
            //             current_faulty_motors[idx] ? "true" : "false",
            //             fault_retry_counts[idx],
            //             MAX_FAULT_RETRIES,
            //             motors_cmds[idx]);
            //     }
            //     continue;
            // }

            // If this motor's wheel-pair partner is faulty, don't command it either
            int pair_idx = paired_motor_idx(id);
            if (pair_idx >= 0 && current_faulty_motors[pair_idx])
            {
                if (is_drive_motor(*layout))
                    motor->set_velocity_ref(0);
                // if (layout->can_id == FRONT_RIGHT_DRIVE)
                // {
                //     const MotorLayout* paired_layout = paired_motor_layout(*layout);
                //     RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                //         "FRONT_RIGHT_DRIVE command skipped because paired motor is faulty | can_id=%d | paired_motor=%s | command_ref=%.2f",
                //         layout->can_id,
                //         paired_layout != nullptr ? paired_layout->name : "unknown",
                //         motors_cmds[idx]);
                // }
                continue;
            }

            if (is_steer_motor(*layout))
                motor->set_position_ref(motors_cmds[idx]);
            else if (is_drive_motor(*layout))
                motor->set_velocity_ref(motors_cmds[idx]);
        }
    }

    static void interrupt_handler(int /*s*/)
    {
        exit(1);
    }

    bool connect_motors(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, bool homing) const
    {
        const size_t nb_expected = sizeof(MOTOR_LAYOUT) / sizeof(MOTOR_LAYOUT[0]);
        RCLCPP_INFO(logger, "==== NAV motors : connection sequence start (homing=%s, %zu motors expected) ====",
                    homing ? "true" : "false", nb_expected);

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
        int retries = 0;
        while (safemode && !gateway && rclcpp::ok() && MAX_TRY--)
        {
            retries++;
            RCLCPP_WARN(logger, "CAN network gateway opening has failed ! retry %d/30 in 2 s", retries);
            reconnect_rate.sleep();
            gateway = open_gateway();
        }

        // If the gateway is still not open, return false
        if (!gateway)
        {
            RCLCPP_ERROR(logger, "CAN network gateway could not be opened after %d attempts, aborting configuration",
                         retries + 1);
            return false;
        }
        if (retries > 0)
        {
            RCLCPP_INFO(logger, "CAN network gateway opened after %d retries", retries);
        }

        RCLCPP_INFO(logger, "START MOTOR DETECTION");
        size_t nb_connected = 0;
        std::string unresponsive_motors;
        for (const auto& layout : MOTOR_LAYOUT)
        {
            const bool drive_motor = is_drive_motor(layout);
            motors.push_back(NAV_Motor(
                gateway,
                layout.can_id,
                drive_motor ? MT_EC_BLOCK_COMMUTATED_MOTOR : MT_EC_SINUS_COMMUTATED_MOTOR,
                drive_motor ? OMD_PROFILE_VELOCITY_MODE : OMD_PROFILE_POSITION_MODE,
                drive_motor ? false : homing));

            if (motors.back().connected())
            {
                nb_connected++;
                RCLCPP_INFO(logger, "  %-18s (node %d, %s) : CONNECTED",
                            layout.name, layout.can_id, drive_motor ? "drive" : "steering");
            }
            else
            {
                if (!unresponsive_motors.empty())
                    unresponsive_motors += ", ";
                unresponsive_motors += layout.name;
                RCLCPP_ERROR(logger, "  %-18s (node %d, %s) : NOT CONNECTED",
                             layout.name, layout.can_id, drive_motor ? "drive" : "steering");
            }
        }
        RCLCPP_INFO(logger, "END MOTOR DETECTION : %zu/%zu motors answered", nb_connected, nb_expected);
        if (!unresponsive_motors.empty())
        {
            RCLCPP_ERROR(logger, "unresponsive motors : %s", unresponsive_motors.c_str());
        }

        for (auto motor = motors.begin(); motor != motors.end(); motor++)
        {
            int id = motor->get_id();
            const MotorLayout* layout = motor_layout_from_can_id(id);
            if (layout == nullptr) {
                RCLCPP_WARN_THROTTLE(get_logger(), *clock, 1000,
                    "Unknown CAN node ID %d after motor detection", id);
                return false;
            }
            if (motor->connected())
            {
                if (!motor->set_output_state(true))
                {
                    RCLCPP_ERROR(logger, "%s (node %d) : FAILED to enable the power stage",
                                 layout->name, id);
                }
                else
                {
                    RCLCPP_INFO(logger, "%s (node %d) : power stage enabled", layout->name, id);
                }

                if (is_steer_motor(*layout))
                {
                    RCLCPP_INFO(logger, "%s (node %d) : steering, position %d, sending position ref 0",
                                layout->name, id, motor->get_position_is());
                    if (!motor->set_position_ref(0))
                    {
                        RCLCPP_ERROR(logger, "%s (node %d) : FAILED to send the initial position reference",
                                     layout->name, id);
                    }
                }
                else if (is_drive_motor(*layout))
                {
                    RCLCPP_INFO(logger, "%s (node %d) : drive, position %d",
                                layout->name, id, motor->get_position_is());
                }
            }
            else
            {
                RCLCPP_ERROR(logger,
                    "%s (node %d) is unresponsive : aborting the connection sequence, the whole node stays unconfigured",
                    layout->name, id);
                return false;
            }
        }

        RCLCPP_INFO(logger, "==== NAV motors : %zu/%zu motors connected and enabled ====",
                    nb_connected, nb_expected);
        return true;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msg::msg::Motorcmds>::SharedPtr sub_motors_displacement;
    rclcpp::Publisher<custom_msg::msg::MotorStatus>::SharedPtr pub_motor_nav_status;

    size_t count_;
    bool homing;

private:
    std::mutex can_mutex_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::CallbackGroup::SharedPtr sub_group_;

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
        last_fault_log_times.clear();
    }

    static constexpr int MAX_FAULT_RETRIES = 3;
    std::vector<bool> current_faulty_motors;
    std::vector<int> fault_retry_counts;
    std::vector<std::chrono::steady_clock::time_point> last_fault_log_times;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_nav_motors_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_home_nav_motors_service_;

    static const char* corner_name(int corner)
    {
        switch (corner)
        {
            case FRONT_LEFT:
                return "front_left";
            case FRONT_RIGHT:
                return "front_right";
            case BACK_RIGHT:
                return "back_right";
            case BACK_LEFT:
                return "back_left";
            default:
                return "unknown_corner";
        }
    }

    static const char* motor_kind_name(const MotorLayout& layout)
    {
        return is_drive_motor(layout) ? "drive" : "steering";
    }

    bool should_log_motor_fault(int can_idx)
    {
        if (can_idx < 0 || static_cast<size_t>(can_idx) >= last_fault_log_times.size())
            return true;

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = now - last_fault_log_times[can_idx];
        if (last_fault_log_times[can_idx].time_since_epoch().count() == 0 ||
            elapsed >= std::chrono::seconds(1))
        {
            last_fault_log_times[can_idx] = now;
            return true;
        }

        return false;
    }

    std::string describe_motor_fault(
        NAV_Motor& motor,
        const MotorLayout& layout,
        const std::string& event,
        unsigned int epos_api_error_code = 0)
    {
        const int can_idx = internal_can_index(layout);
        const int status_idx = motor_status_index(layout);
        const int pair_idx = paired_motor_idx(layout.can_id);
        const MotorLayout* paired_layout = paired_motor_layout(layout);

        std::ostringstream details;
        details << event
                << " | motor=" << layout.name
                << " | type=" << motor_kind_name(layout)
                << " | corner=" << corner_name(layout.corner)
                << " | can_id=" << layout.can_id
                << " | internal_can_idx=" << can_idx
                << " | motor_status_idx=" << status_idx
                << " | connected=" << (motor.connected() ? "true" : "false")
                << " | command_ref=" << motors_cmds[can_idx]
                << " | retry=" << fault_retry_counts[can_idx] << "/" << MAX_FAULT_RETRIES;

        if (paired_layout != nullptr)
        {
            details << " | paired_motor=" << paired_layout->name
                    << " | paired_type=" << motor_kind_name(*paired_layout)
                    << " | paired_can_id=" << paired_layout->can_id;
            if (pair_idx >= 0 && static_cast<size_t>(pair_idx) < current_faulty_motors.size())
            {
                details << " | paired_fault=" << (current_faulty_motors[pair_idx] ? "true" : "false");
            }
        }

        if (epos_api_error_code != 0)
        {
            details << " | epos_api_error=0x" << std::hex << epos_api_error_code << std::dec;
        }

        const auto device_errors = motor.get_device_error_messages();
        details << " | device_errors=";
        for (size_t i = 0; i < device_errors.size(); ++i)
        {
            if (i > 0)
                details << "; ";
            details << device_errors[i];
        }

        return details.str();
    }

    void log_motor_fault_throttled(
        NAV_Motor& motor,
        const MotorLayout& layout,
        const std::string& event,
        unsigned int epos_api_error_code = 0,
        bool as_error = false)
    {
        const int can_idx = internal_can_index(layout);
        if (!should_log_motor_fault(can_idx))
            return;

        const std::string details = describe_motor_fault(motor, layout, event, epos_api_error_code);
        if (as_error)
            RCLCPP_ERROR(get_logger(), "%s", details.c_str());
        else
            RCLCPP_WARN(get_logger(), "%s", details.c_str());
    }

    // Returns the private CAN-indexed fault array slot for the same wheel's paired motor.
    int paired_motor_idx(int id) const
    {
        const MotorLayout* layout = motor_layout_from_can_id(id);
        if (layout == nullptr)
            return -1;

        const MotorLayout* paired = paired_motor_layout(*layout);
        if (paired == nullptr)
            return -1;

        return internal_can_index(*paired);
    }
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
        RCLCPP_INFO(rclcpp::get_logger("NAV_motor_cmds"), "WARNING: node MotorCmds ended");
    }

    rclcpp::shutdown();
    return 0;
}