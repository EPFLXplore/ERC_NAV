#include "wheels_control/motors.hpp"
#include <cstdio>

//use the ros logger instead of cout from iostream
#include "rclcpp/rclcpp.hpp"
#include "wheels_control/definition.hpp"
#include <chrono>
#include <string>

void log_info(const std::string &msg){
    RCLCPP_INFO(rclcpp::get_logger("motors"), "%s", msg.c_str());
}

void log_warn(const std::string &msg){
    RCLCPP_WARN(rclcpp::get_logger("motors"), "%s", msg.c_str());
}

void log_error(const std::string &msg){
    RCLCPP_ERROR(rclcpp::get_logger("motors"), "%s", msg.c_str());
}

// human readable "0x... : message" for any EPOS API error code
std::string vcs_error_string(unsigned int error_code)
{
    if (!error_code)
        return "0x0 : no error";

    char error_msg[100] = {0};
    if (!VCS_GetErrorInfo(error_code, error_msg, 100))
        std::snprintf(error_msg, sizeof(error_msg), "unknown EPOS error code");

    char buffer[160];
    std::snprintf(buffer, sizeof(buffer), "0x%X : %s", error_code, error_msg);
    return buffer;
}

// name of a CAN node as declared in definition.hpp, "UNKNOWN_NODE" otherwise
std::string node_name(unsigned short id)
{
    const MotorLayout *layout = motor_layout_from_can_id((int)id);
    return (layout != nullptr) ? layout->name : "UNKNOWN_NODE";
}

// "node 4 (FRONT_RIGHT_DRIVE)" prefix shared by every per motor log line
std::string node_tag(unsigned short id)
{
    return "node " + std::to_string((int)id) + " (" + node_name(id) + ")";
}

static const char *operation_mode_name(signed char mode)
{
    switch (mode)
    {
    case OMD_PROFILE_POSITION_MODE:
        return "profile position";
    case OMD_PROFILE_VELOCITY_MODE:
        return "profile velocity";
    case OMD_VELOCITY_MODE:
        return "velocity";
    case 0:
        return "none";
    default:
        return "other";
    }
}

static const char *motor_type_name(unsigned short type)
{
    return (type == MT_DC_MOTOR)                    ? "DC motor"
           : (type == MT_EC_SINUS_COMMUTATED_MOTOR) ? "EC sine motor"
           : (type == MT_EC_BLOCK_COMMUTATED_MOTOR) ? "EC block motor"
                                                    : "unknown motor type";
}

// ---------- namespaces ----------

using namespace std;



#define CONNECTION_CHECK                                                          \
    if (!is_connected)                                                            \
    {                                                                             \
        static rclcpp::Clock connection_check_clock(RCL_STEADY_TIME);             \
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("motors"), connection_check_clock,\
                             1000, "%s is not connected", node_tag(id).c_str());  \
        return false;                                                             \
    }

// private functions
void print_VCS_error(unsigned int error_code, const char *func)
{
    if (error_code)
    {
        log_error("ERR " + vcs_error_string(error_code) + " in " + func + "()");
    }
}

// public functions
bool get_device_name_selection(bool start_of_selection, std::string &device_name, int &end_of_selection)
{
    const unsigned short MAX_STR_SIZE = 256;
    char name_buffer[MAX_STR_SIZE] = {0};

    unsigned int error_code = 0;
    int local_end_of_select = 0;

    // Pass the start flag to the EPOS API function.
    // The API is expected to have a signature similar to:
    // BOOL VCS_GetDeviceNameSelection(BOOL StartOfSelection, char* pDeviceNameSel,
    //                                 WORD MaxStrSize, int* pEndOfSelection, DWORD* pErrorCode);
    bool success = (VCS_GetDeviceNameSelection(
                        start_of_selection ? 1 : 0,  // Use the passed start flag
                        name_buffer,
                        MAX_STR_SIZE,
                        &local_end_of_select,
                        &error_code) != 0); // assuming nonzero means success

    print_VCS_error(error_code, __FUNCTION__);

    if (!success){
        log_warn("VCS_GetDeviceNameSelection failed (" + vcs_error_string(error_code) + ")");
        return false;
    }

    device_name = name_buffer;
    end_of_selection = (local_end_of_select != 0) ? 1 : 0;

    return true;
}

bool get_protocol_stack_name_selection(std::string &device_name,
    std::string &protocol_stack_name_sel,
    bool start_of_selection,
    unsigned short max_str_size,
    int &end_of_selection,
    unsigned int &error_code)
{
    // Set a local buffer size; you can also use max_str_size if you are sure it is appropriate.
    static const unsigned short LOCAL_MAX_STR_SIZE = 256; 
    // We'll use the smaller of the provided max_str_size and our local constant.
    unsigned short buffer_size = (max_str_size < LOCAL_MAX_STR_SIZE) ? max_str_size : LOCAL_MAX_STR_SIZE;
    char protocol_buffer[LOCAL_MAX_STR_SIZE] = {0};

    int local_end_of_sel = 0;  // This will be filled by the EPOS function

    // Call the EPOS API.
    // VCS_GetProtocolStackNameSelection expects:
    // (char* DeviceName, int StartOfSelection, char* pProtocolStackNameSel, unsigned short MaxStrSize, int* pEndOfSelection, unsigned int* pErrorCode);
    bool success = (VCS_GetProtocolStackNameSelection(
        (char*)device_name.c_str(),             // DeviceName as a C string
        start_of_selection ? 1 : 0,              // Convert bool to int (1 for true, 0 for false)
        protocol_buffer,                         // Buffer for the protocol stack name
        buffer_size,                             // Maximum length for the name
        &local_end_of_sel,                       // Pass pointer to local_end_of_sel
        &error_code                              // Pass pointer to error_code
    ) != 0);  // assuming nonzero means success in the EPOS API

    print_VCS_error(error_code, __FUNCTION__);
    if (!success){
        log_warn("VCS_GetProtocolStackNameSelection failed (" + vcs_error_string(error_code) + ")");
        return false;
    }

    // Save the returned string to the std::string output parameter.
    protocol_stack_name_sel = protocol_buffer;

    // Set the caller’s end_of_selection flag (nonzero means end reached).
    end_of_selection = (local_end_of_sel != 0) ? 1 : 0;

    return true;
}

bool get_interface_name_selection(const std::string &device_name,
    const std::string &protocol_stack_name,
    bool start_of_selection,
    std::string &interface_name,
    unsigned short max_str_size,
    bool &end_of_selection,
    unsigned int &error_code)
{
    // Clamp the buffer length to a local constant
    static const unsigned short LOCAL_MAX_STR_SIZE = 256;
    unsigned short buffer_len = (max_str_size < LOCAL_MAX_STR_SIZE) ? max_str_size : LOCAL_MAX_STR_SIZE;

    // Local buffer for the interface name returned by the EPOS API
    char interface_buffer[LOCAL_MAX_STR_SIZE] = {0};

    // The EPOS function expects an int* for end-of-selection, not a bool*
    int local_end_of_sel = 0;

    // Convert the start flag to the type expected by the API (typically int for BOOL)
    bool bStartOfSelection = start_of_selection ? true : false;

    // Call EPOS API: VCS_GetInterfaceNameSelection prototype:
    // int VCS_GetInterfaceNameSelection(char* DeviceName, char* ProtocolStackName, int StartOfSelection, 
    //     char* pInterfaceNameSel, unsigned short MaxStrSize, int* pEndOfSelection, unsigned int* pErrorCode);
    bool success = (VCS_GetInterfaceNameSelection(
                        (char*)device_name.c_str(),
                        (char*)protocol_stack_name.c_str(),
                        bStartOfSelection,
                        interface_buffer,
                        buffer_len,
                        &local_end_of_sel,    // Pass an int pointer, not a bool pointer
                        &error_code
                    ) != 0);  // Assuming nonzero indicates success

    // Print error if any
    print_VCS_error(error_code, __FUNCTION__);
    if (!success) {
        log_warn("VCS_GetInterfaceNameSelection failed (" + vcs_error_string(error_code) + ")");
        return false;
    }

    // Convert the C-style buffer result to std::string
    interface_name = interface_buffer;

    // Convert the returned int to a bool for the output parameter
    end_of_selection = (local_end_of_sel != 0);

    return true;
}


bool get_port_name_selection(const std::string &device_name,
                            const std::string &protocol_stack_name,
                            const std::string &interface_name,
                            bool start_of_selection,
                            std::string &port_name,
                            unsigned short max_str_size,
                            int &end_of_selection,
                            unsigned int &error_code)
{
    // The EPOS library typically defines WORD as unsigned short, BOOL as int, and DWORD as unsigned int
    static const unsigned short LOCAL_MAX_STR_SIZE = 256;
    unsigned short buffer_len = (max_str_size < LOCAL_MAX_STR_SIZE) ? max_str_size : LOCAL_MAX_STR_SIZE;

    // EPOS requires a C-style buffer for the port name
    char port_buffer[LOCAL_MAX_STR_SIZE] = {0};

    // Convert bool to EPOS BOOL (which is often an int: 1 = TRUE, 0 = FALSE)
    int bStartOfSelection = start_of_selection ? 1 : 0;

    // EPOS also returns a BOOL for end_of_selection
    int local_end_of_sel = 0;
    // VCS_GetPortNameSelection signature:
    // BOOL VCS_GetPortNameSelection(
    //   char* DeviceName, char* ProtocolStackName, char* InterfaceName, BOOL StartOfSelection,
    //   char* pPortSel, WORD MaxStrSize, BOOL* pEndOfSelection, DWORD* pErrorCode);
    //
    // Note: We cast error_code's address to DWORD*, since that’s what the function expects.
    bool success = (VCS_GetPortNameSelection(
        (char*)device_name.c_str(),         // DeviceName
        (char*)protocol_stack_name.c_str(), // ProtocolStackName
        (char*)interface_name.c_str(),      // InterfaceName
        bStartOfSelection,                  // StartOfSelection flag
        port_buffer,                        // Output buffer for port name
        buffer_len,                         // Maximum string size
        &local_end_of_sel,                  // Pointer to local_end_of_sel (int*)
        &error_code                         // Pointer to error_code (unsigned int*)
    ) != 0);  // Assuming a nonzero return indicates success


    // Check for errors
    print_VCS_error(error_code, __FUNCTION__);
    if (!success){
        log_warn("VCS_GetPortNameSelection failed (" + vcs_error_string(error_code) + ")");
        return false;
    }

    // If the call succeeded, store the returned port name into our std::string
    port_name = port_buffer;

    // If local_end_of_sel is true, it means no more port names are available
    //end_of_selection = (local_end_of_sel != 0);
    if(local_end_of_sel != 0){
        end_of_selection = 1;
    }else{
        end_of_selection = 0;
    }

    return true;
}


void *open_gateway(void)
{
    unsigned int error_code = 0;
    //WATCH OUT : YOU NEED THE UDEV RULES IN AND OUT OF THE DOCKER.
    int end_of_selection = 0;
    std::string current_device;
    std::string current_protocol_stack;
    std::string device_name = "EPOS4";
    std::string protocol_stack = "MAXON SERIAL V2";
    std::string interface_name = "USB";
    std::string port_name_wanted = "USB0";
    std::string current_interface;
    const unsigned short max_size = 100;
    bool start_of_selection = true;

    log_info("===== EPOS gateway : opening =====");
    log_info("requested : device='" + device_name + "' protocol='" + protocol_stack +
             "' interface='" + interface_name + "' port='" + port_name_wanted + "'");

    // ---- 1/4 : device names known by the EPOS command library ----
    unsigned int nb_devices = 0;
    bool device_found = false;
    while (get_device_name_selection(start_of_selection, current_device, end_of_selection))
    {
        nb_devices++;
        log_info("  [1/4] available device    : " + current_device);
        if (current_device == device_name)
            device_found = true;
        if (end_of_selection)
            break;
        start_of_selection = false;
    }
    if (nb_devices == 0)
        log_error("  [1/4] no device name returned : the EPOS command library is not answering");
    else if (!device_found)
        log_warn("  [1/4] '" + device_name + "' is NOT in the list of available devices");

    // ---- 2/4 : protocol stacks available for that device ----
    start_of_selection = true;
    end_of_selection = 0;
    unsigned int nb_protocols = 0;
    bool protocol_found = false;
    while (get_protocol_stack_name_selection(device_name,
                                             current_protocol_stack,
                                             start_of_selection,
                                             max_size,
                                             end_of_selection,
                                             error_code))
    {
        nb_protocols++;
        log_info("  [2/4] available protocol  : " + current_protocol_stack);
        if (current_protocol_stack == protocol_stack)
            protocol_found = true;
        if (end_of_selection)
            break;
        start_of_selection = false;
    }
    if (nb_protocols == 0)
        log_error("  [2/4] no protocol stack returned for device '" + device_name + "'");
    else if (!protocol_found)
        log_warn("  [2/4] '" + protocol_stack + "' is NOT in the list of available protocol stacks");

    // ---- 3/4 : interfaces available for that protocol stack ----
    start_of_selection = true;
    bool interface_end = false;
    unsigned int nb_interfaces = 0;
    bool interface_found = false;
    while (get_interface_name_selection(device_name,
                                        protocol_stack,
                                        start_of_selection,
                                        current_interface,
                                        max_size,
                                        interface_end,
                                        error_code))
    {
        nb_interfaces++;
        log_info("  [3/4] available interface : " + current_interface);
        if (current_interface == interface_name)
            interface_found = true;
        if (interface_end)
            break;
        start_of_selection = false;
    }
    if (nb_interfaces == 0)
        log_error("  [3/4] no interface returned for '" + device_name + "' / '" + protocol_stack + "'");
    else if (!interface_found)
        log_warn("  [3/4] '" + interface_name + "' is NOT in the list of available interfaces");

    // ---- 4/4 : ports. A missing cable, an unpowered gateway or a missing udev
    //            rule shows up here as an empty list.
    start_of_selection = true;
    end_of_selection = 0;
    std::string port_name;
    unsigned int nb_ports = 0;
    bool port_found = false;
    while (get_port_name_selection(device_name,
                                   protocol_stack,
                                   interface_name,
                                   start_of_selection,
                                   port_name,
                                   max_size,
                                   end_of_selection,
                                   error_code))
    {
        nb_ports++;
        log_info("  [4/4] available port      : " + port_name);
        if (port_name == port_name_wanted)
            port_found = true;
        if (end_of_selection)
            break;
        start_of_selection = false;
    }
    if (nb_ports == 0)
    {
        log_error("  [4/4] no " + interface_name + " port found : the EPOS4 gateway is not visible from this process");
        log_error("        check : USB cable plugged, gateway powered, 'lsusb' shows the maxon device,");
        log_error("        and the udev rules are installed INSIDE and OUTSIDE the docker container");
    }
    else if (!port_found)
    {
        log_warn("  [4/4] '" + port_name_wanted + "' was not enumerated, trying to open it anyway");
    }

    // ---- open the device ----
    error_code = 0;
    const auto open_start = std::chrono::steady_clock::now();
    void *gateway = VCS_OpenDevice((char *)device_name.c_str(),
                                   (char *)protocol_stack.c_str(),
                                   (char *)interface_name.c_str(),
                                   (char *)port_name_wanted.c_str(),
                                   &error_code);
    const double open_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - open_start)
                               .count();
    print_VCS_error(error_code, __FUNCTION__);

    char summary[256];
    if (gateway == nullptr)
    {
        std::snprintf(summary, sizeof(summary),
                      "VCS_OpenDevice FAILED after %.0f ms on %s/%s (%s)",
                      open_ms, interface_name.c_str(), port_name_wanted.c_str(),
                      vcs_error_string(error_code).c_str());
        log_error(summary);
        log_error("===== EPOS gateway : NOT opened, no motor can be reached =====");
        return gateway;
    }

    if (error_code)
        log_warn("VCS_OpenDevice returned a valid handle but reported " + vcs_error_string(error_code));

    std::snprintf(summary, sizeof(summary),
                  "VCS_OpenDevice OK in %.0f ms : handle=%p on %s/%s",
                  open_ms, gateway, interface_name.c_str(), port_name_wanted.c_str());
    log_info(summary);
    log_info("===== EPOS gateway : opened =====");

    // VCS_ClearFault(gateway, 0, &error_code);
    // print_VCS_error(error_code);
    return gateway;
}

void close_gateway(void *gateway)
{
    unsigned int error_code = 0;
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "closing EPOS gateway (handle=%p)", gateway);
    log_info(buffer);

    VCS_CloseDevice(gateway, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    if (!error_code)
        log_info("EPOS gateway closed");
}

// CONSTRUCTORS
NAV_Motor::NAV_Motor(void *KeyHandle, unsigned short node_id, unsigned short exp_type, signed char mode, bool homing)
{
    unsigned int error_code = 0;
    gateway = KeyHandle;
    id = node_id;
    pos_ref = 0;
    op_mode = 0;

    const std::string tag = node_tag(id);
    char buffer[256];

    std::snprintf(buffer, sizeof(buffer),
                  "%s : probing | expected_type=%s requested_mode=%s homing=%s",
                  tag.c_str(), motor_type_name(exp_type), operation_mode_name(mode),
                  homing ? "true" : "false");
    log_info(buffer);

    const auto probe_start = std::chrono::steady_clock::now();
    VCS_GetMotorType(gateway, id, &motor_type, &error_code);
    const double probe_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - probe_start)
                                .count();

    is_connected = (error_code) ? false : true; // SDO TIMEOUT  == 0x5040000

    if (is_connected)
    {
        std::snprintf(buffer, sizeof(buffer),
                      "%s : ANSWERED in %.0f ms | motor_type=%d (%s)",
                      tag.c_str(), probe_ms, (int)motor_type, motor_type_name(motor_type));
        log_info(buffer);

        if (exp_type && (motor_type != exp_type))
        {
            std::snprintf(buffer, sizeof(buffer),
                          "%s : motor type MISMATCH, expected %s but the controller reports %s",
                          tag.c_str(), motor_type_name(exp_type), motor_type_name(motor_type));
            log_warn(buffer);
            // throw 0;
        }

        // state of the controller as found at connection time
        unsigned int fault_error_code = 0;
        int in_fault = false;
        VCS_GetFaultState(gateway, id, &in_fault, &fault_error_code);
        if (fault_error_code)
        {
            log_warn(tag + " : could not read the fault state (" + vcs_error_string(fault_error_code) + ")");
        }
        else if (in_fault)
        {
            log_warn(tag + " : controller is in FAULT state at connection time");
            const std::vector<std::string> device_errors = this->get_device_error_messages();
            for (size_t i = 0; i < device_errors.size(); i++)
                log_warn(tag + " : device error : " + device_errors[i]);
        }
        else
        {
            log_info(tag + " : no fault reported at connection time");
        }

        if (homing && (mode == OMD_PROFILE_POSITION_MODE))
        {
            int pos = 0;
            error_code = 0;
            VCS_GetPositionIs(gateway, id, &pos, &error_code);
            VCS_ActivateHomingMode(gateway, id, &error_code);
            VCS_DefinePosition(gateway, id, pos, &error_code);
            VCS_StopHoming(gateway, id, &error_code);
            if (error_code)
            {
                print_VCS_error(error_code, __FUNCTION__);
                log_error(tag + " : homing FAILED (" + vcs_error_string(error_code) + ")");
            }
            else
            {
                log_info(tag + " : homed, current position defined as " + std::to_string(pos));
            }
        }

        if (mode)
        {
            error_code = 0;
            VCS_SetOperationMode(gateway, id, mode, &error_code);
            if (error_code)
            {
                print_VCS_error(error_code, __FUNCTION__);
                log_error(tag + " : could not set the operation mode to " + operation_mode_name(mode));
                op_mode = 0;
            }
            else
            {
                op_mode = mode;
                log_info(tag + " : operation mode set to " + operation_mode_name(mode));
            }

            error_code = 0;
            if (mode == OMD_PROFILE_POSITION_MODE) // OMD_PROFILE_POSITION_MODE
            {
                VCS_SetPositionProfile(gateway, id, MAX_STEER_VEL, MAX_STEER_ACCEL, MAX_STEER_ACCEL, &error_code);
                std::snprintf(buffer, sizeof(buffer),
                              "%s : position profile vel=%d rpm accel=decel=%d rpm/s -> %s",
                              tag.c_str(), MAX_STEER_VEL, MAX_STEER_ACCEL, error_code ? "FAILED" : "ok");
            }
            else if (mode == OMD_PROFILE_VELOCITY_MODE)
            {
                VCS_SetVelocityProfile(gateway, id, MAX_DRIVE_ACCEL, MAX_DRIVE_DECEL, &error_code);
                std::snprintf(buffer, sizeof(buffer),
                              "%s : velocity profile accel=%d rpm/s decel=%d rpm/s -> %s",
                              tag.c_str(), MAX_DRIVE_ACCEL, MAX_DRIVE_DECEL, error_code ? "FAILED" : "ok");
            }
            else
            {
                std::snprintf(buffer, sizeof(buffer), "%s : no motion profile written for this mode", tag.c_str());
            }

            if (error_code)
            {
                log_error(buffer);
                print_VCS_error(error_code, __FUNCTION__);
            }
            else
            {
                log_info(buffer);
            }
        }
        else
        {
            log_info(tag + " : no operation mode requested, controller left as is");
        }
    }
    else
    {
        std::snprintf(buffer, sizeof(buffer),
                      "%s : UNRESPONSIVE after %.0f ms (%s)",
                      tag.c_str(), probe_ms, vcs_error_string(error_code).c_str());
        log_error(buffer);

        if (error_code == 0x5040000)
        {
            log_error(tag + " : SDO timeout, the controller did not answer on the CAN bus.");
            log_error(tag + " : check the node ID switches, the CAN wiring and termination, and the 24V/48V power of that controller");
        }
        else
        {
            print_VCS_error(error_code, __FUNCTION__);
        }
        motor_type = 0;
    }
}

// UTILITY FUNCTIONS
unsigned short NAV_Motor::get_id()
{
    return id;
}

bool NAV_Motor::connected()
{
    return is_connected;
}

void NAV_Motor::disconnect()
{
    is_connected = false;
}

bool NAV_Motor::reconnect()
{
    unsigned int error_code = 0;
    unsigned short detected_motor_type = 0;

    VCS_GetMotorType(gateway, id, &detected_motor_type, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    if (error_code)
    {
        is_connected = false;
        return false;
    }

    motor_type = detected_motor_type;
    is_connected = true;

    if (op_mode && !set_operational_mode(op_mode))
    {
        is_connected = false;
        return false;
    }

    if (op_mode == OMD_PROFILE_POSITION_MODE)
        VCS_SetPositionProfile(gateway, id, MAX_STEER_VEL, MAX_STEER_ACCEL, MAX_STEER_ACCEL, &error_code);
    else if (op_mode == OMD_PROFILE_VELOCITY_MODE)
        VCS_SetVelocityProfile(gateway, id, MAX_DRIVE_ACCEL, MAX_DRIVE_DECEL, &error_code);

    print_VCS_error(error_code, __FUNCTION__);
    if (error_code)
    {
        is_connected = false;
        return false;
    }

    return true;
}

int NAV_Motor::get_encoder_pulse()
{
    unsigned int encoder_pulse_nb;
    int InvertedPolarity;
    unsigned int ErrorCode;
    VCS_GetIncEncoderParameter(gateway, id, &encoder_pulse_nb, &InvertedPolarity, &ErrorCode);
    print_VCS_error(ErrorCode, __FUNCTION__);
    return encoder_pulse_nb;
}

bool NAV_Motor::set_operational_mode(signed char mode)
{
    CONNECTION_CHECK;

    unsigned int error_code = 0;
    VCS_SetOperationMode(gateway, id, mode, &error_code);
    if (error_code)
    {
        print_VCS_error(error_code, __FUNCTION__);
        op_mode = 0;
        return false;
    }
    op_mode = mode;
    return true;
}

bool NAV_Motor::set_output_state(bool output_active)
{
    CONNECTION_CHECK;

    unsigned int error_code = 0;
    VCS_ClearFault(gateway, id, &error_code); // clear fault just in case
    print_VCS_error(error_code, __FUNCTION__);

    if (output_active)
        VCS_SetEnableState(gateway, id, &error_code);
    else
        VCS_SetDisableState(gateway, id, &error_code);
    print_VCS_error(error_code, __FUNCTION__);

    return !error_code;
}

bool NAV_Motor::fault_state()
{
    unsigned int error_code = 0;
    int fault = false;
    VCS_GetFaultState(gateway, id, &fault, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    return fault;
}

bool NAV_Motor::fault_state(unsigned int *error_code)
{
    int fault = false;
    VCS_GetFaultState(gateway, id, &fault, error_code);
    if (*error_code)
        is_connected = false;
    return fault;
}

bool NAV_Motor::is_faulty(bool verbose)
{
    if (!is_connected)
        return true;

    unsigned int error_code = 0;
    int in_fault = false;
    VCS_GetFaultState(gateway, id, &in_fault, &error_code);
    print_VCS_error(error_code, __FUNCTION__);

    if (in_fault && !error_code && verbose)
    {
        unsigned int device_error = 0;
        unsigned char nb_err = 0;
        VCS_GetNbOfDeviceError(gateway, id, &nb_err, &error_code);
        print_VCS_error(error_code, __FUNCTION__);
        if (nb_err && !error_code)
        {
            static char error_msg[100];
            for (unsigned char i = 1; i <= nb_err; i++)
            {
                VCS_GetDeviceErrorCode(gateway, id, i, &device_error, &error_code);
                VCS_GetErrorInfo(device_error, error_msg, 100);
                log_warn("FAULT : " + node_tag(id) + " : " + error_msg);
            }
        }
    }

    return (in_fault || error_code);
}

std::vector<std::string> NAV_Motor::get_device_error_messages()
{
    std::vector<std::string> errors;
    if (!is_connected)
    {
        errors.push_back("motor is not connected");
        return errors;
    }

    unsigned int error_code = 0;
    unsigned char nb_err = 0;
    VCS_GetNbOfDeviceError(gateway, id, &nb_err, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    if (error_code)
    {
        char buffer[80];
        std::snprintf(buffer, sizeof(buffer), "failed to read device errors, EPOS API error 0x%X", error_code);
        errors.push_back(buffer);
        return errors;
    }

    static char error_msg[100];
    for (unsigned char i = 1; i <= nb_err; i++)
    {
        unsigned int device_error = 0;
        VCS_GetDeviceErrorCode(gateway, id, i, &device_error, &error_code);
        if (error_code)
        {
            char buffer[80];
            std::snprintf(buffer, sizeof(buffer), "failed to read device error %u/%u, EPOS API error 0x%X", i, nb_err, error_code);
            errors.push_back(buffer);
            continue;
        }

        VCS_GetErrorInfo(device_error, error_msg, 100);
        char buffer[160];
        std::snprintf(buffer, sizeof(buffer), "0x%X: %s", device_error, error_msg);
        errors.push_back(buffer);
    }

    if (errors.empty())
    {
        errors.push_back("controller reports fault state but no device error entries");
    }

    return errors;
}

bool NAV_Motor::clear_fault()
{
    //VCS_ClearFault changes the device state from “fault” to “disable”.
    CONNECTION_CHECK;
    unsigned int error_code = 0;
    VCS_ClearFault(gateway, id, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    return !error_code;
}

bool NAV_Motor::set_position_ref(long pos)
{
    CONNECTION_CHECK;

    unsigned int error_code = 0;
    if ((op_mode != OMD_PROFILE_POSITION_MODE) &&
        !this->set_operational_mode(OMD_PROFILE_POSITION_MODE))
        return false;

    VCS_MoveToPosition(gateway, id, pos, true, true, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    return !error_code;
}

bool NAV_Motor::homing()
{
    int pos;
    unsigned int error_code = 0;
    VCS_GetPositionIs(gateway, id, &pos, &error_code);
    VCS_ActivateHomingMode(gateway, id, &error_code);
    bool succesfull_homing = VCS_DefinePosition(gateway, id, pos, &error_code);
    VCS_StopHoming(gateway, id, &error_code);
    return succesfull_homing;
}
bool NAV_Motor::reset_position_counter()
{
    CONNECTION_CHECK;

    unsigned int error_code = 0;
    int pos;
    VCS_GetPositionIs(gateway, id, &pos, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    pos_ref = pos;

    // VCS_ResetPositionMarkerCounter(gateway, id, &error_code); // not available ???
    // print_VCS_error(error_code, __FUNCTION__);
    return !error_code;
}

int NAV_Motor::get_position_is()
{
    CONNECTION_CHECK;

    unsigned int error_code = 0;
    int pos;
    VCS_GetPositionIs(gateway, id, &pos, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    return pos;
}

bool NAV_Motor::has_reached()
{
    unsigned int error_code = 0;
    int reached;
    VCS_GetMovementState(gateway, id, &reached, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    return reached;
}

// bool NAV_Motor::fcity_ref(long vel) {
//     CONNECTION_CHECK;

//     unsigned int    error_code  = 0;
//     if ((op_mode != OMD_VELOCITY_MODE) &&
//         !this->set_operational_mode(OMD_VELOCITY_MODE))
//         return false;

//     VCS_SetVelocityMust(gateway, id, vel, &error_code);
//     print_VCS_error(error_code, __FUNCTION__);
//     return !error_code;
// }

bool NAV_Motor::set_velocity_ref(long vel)
{
    unsigned int error_code = 0;
    return set_velocity_ref(vel, &error_code);
}

bool NAV_Motor::set_velocity_ref(long vel, unsigned int *error_code)
{
    if (!is_connected)
    {
        *error_code = 0;
        return false;
    }

    // The drive motors are configured in OMD_PROFILE_VELOCITY_MODE, so that is what
    // op_mode holds. Comparing against OMD_VELOCITY_MODE made this test always true and
    // forced a redundant VCS_SetOperationMode write before every single velocity command.
    if ((op_mode != OMD_PROFILE_VELOCITY_MODE) &&
        !this->set_operational_mode(OMD_PROFILE_VELOCITY_MODE))
    {
        *error_code = 0;
        return false;
    }

    VCS_MoveWithVelocity(gateway, id, vel, error_code);
    print_VCS_error(*error_code, __FUNCTION__);
    if (*error_code)
        is_connected = false;
    return !*error_code;
}

int NAV_Motor::get_velocity_is()
{
    unsigned int error_code = 0;
    return get_velocity_is(&error_code);
}

int NAV_Motor::get_velocity_is(unsigned int *error_code)
{
    if (!is_connected)
    {
        *error_code = 0;
        return 0;
    }

    int vel = 0;
    VCS_GetVelocityIs(gateway, id, &vel, error_code);
    print_VCS_error(*error_code, __FUNCTION__);
    if (*error_code)
        is_connected = false;
    return vel;
}

int NAV_Motor::get_current_is()
{
    CONNECTION_CHECK;

    unsigned int error_code = 0;
    int cur;
    VCS_GetCurrentIsEx(gateway, id, &cur, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    return cur;
}

int NAV_Motor::get_current_is_averaged()
{
    CONNECTION_CHECK;

    unsigned int error_code = 0;
    int current_averaged;
    VCS_GetCurrentIsAveragedEx(gateway, id, &current_averaged, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    return current_averaged;
}

int NAV_Motor::get_efficiency()
{
    CONNECTION_CHECK;

    float vel = (float)this->get_velocity_is();
    float cur = (float)this->get_current_is();

    return (int)(100. / (1. + fabs((WINDING_RES * cur) / (SPEED_CONSTANT * vel))));
}
