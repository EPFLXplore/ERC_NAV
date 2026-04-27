#include "wheels_control/motors.hpp"
#include <cstdio>

//use the ros logger instead of cout from iostream
#include "rclcpp/rclcpp.hpp"
void log_info(const std::string &msg){
    RCLCPP_INFO(rclcpp::get_logger("motors"), "%s", msg.c_str());
}

// ---------- namespaces ----------

using namespace std;

#define WINDING_RES 0.573  // [Ohm]                                        // TODO : keep updated // TODO2: make this less stupid
#define SPEED_CONSTANT 100 // [rpm/V] origin value 236                    // TODO : keep updated // TODO2: make this less stupid

#define MAX_DRIVE_ACCEL 900   // [rpm/S] // default :   900
#define MAX_DRIVE_DECEL 3000  // [rpm/s] // default :  3000
#define MAX_STEER_VEL 7   // [rpm]   // max 10
#define MAX_STEER_ACCEL 20 // [rpm/s] // max 28

#ifndef ROSCPP_ROS_H
#define CONNECTION_CHECK                                                   \
    if (!is_connected)                                                     \
    {                                                                      \
        cerr << "[motors.cpp] Node " << id << " is not connected" << endl; \
        return false;                                                      \
    }
#else
#define CONNECTION_CHECK                                        \
    if (!is_connected)                                          \
    {                                                           \
        ROS_ERROR_STREAM("Node " << id << " is not connected"); \
        return false;                                           \
    }
#endif

// private functions
void print_VCS_error(unsigned int error_code, const char *func)
{
    static char error_msg[100];
    if (error_code)
    {
        VCS_GetErrorInfo(error_code, error_msg, 100);
#ifdef ROSCPP_ROS_H
        ROS_ERROR_STREAM("ERR 0x" << hex << error_code << dec << " : " << error_msg << " in " << func << "()");
#else
        cerr << "[motors.cpp] ERR 0x" << hex << error_code << dec << " : " << error_msg << " in " << func << "()" << endl;
#endif
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
    std::string current_interface;
    const unsigned short max_size = 100;
    bool start_of_selection = true;

    while(get_device_name_selection(start_of_selection, current_device, end_of_selection)){
        //std::cout << "Found device name: " << current_device << std::endl;
        log_info("Found device name: "+current_device);
        if(end_of_selection)
            break;
        start_of_selection = false;
    }
    start_of_selection = true;
    end_of_selection = 0;

    while(get_protocol_stack_name_selection(device_name,
        current_protocol_stack,
        start_of_selection,  // pass the start flag here
        max_size,
        end_of_selection,
        error_code)){

        //std::cout << "Available protocol stack: " << current_protocol_stack << std::endl;
        log_info("Available protocol stack: "+current_protocol_stack);

        if(end_of_selection)
            break;
        start_of_selection = false;  // subsequent calls use false
    }

    end_of_selection = 0;
    bool interface_end = false;
    start_of_selection = true;

    while(get_interface_name_selection(
        device_name,             // device name
        current_protocol_stack, // protocol stack name
        start_of_selection,     // TRUE for the first call
        current_interface,      // output: the interface name
        max_size,               // maximum string size
        interface_end,           // output: set when no more interfaces
        error_code))            // output: error code
    {
        std::cout << "Available interface name: " << current_interface << std::endl;
        log_info("Available interface name: "+current_interface);

        if(interface_end)
            break;
        start_of_selection = false;  // subsequent calls: get the next interface name
    }

    end_of_selection = 0;
    start_of_selection = true;
    std::string port_name;

    // Loop until all port names have been enumerated.
    while (get_port_name_selection(device_name,
                                   protocol_stack,
                                   interface_name,
                                   start_of_selection,
                                   port_name,
                                   max_size,
                                   end_of_selection,
                                   error_code))
    {
        std::cout << "Available port name: " << port_name << std::endl;

        // If no more port names are available, exit the loop.
        if (end_of_selection)
            break;

        // For subsequent calls, set start_of_selection to false.
        start_of_selection = false;
    }


    void *gateway = VCS_OpenDevice((char *)"EPOS4", (char *)"MAXON SERIAL V2", (char *)"USB", (char *)"USB0", &error_code);
    print_VCS_error(error_code, __FUNCTION__);
    // VCS_ClearFault(gateway, 0, &error_code);
    // print_VCS_error(error_code);
    return gateway;
}

void close_gateway(void *gateway)
{

    unsigned int error_code = 0;
    VCS_CloseDevice(gateway, &error_code);
    print_VCS_error(error_code, __FUNCTION__);
}

// CONSTRUCTORS
NAV_Motor::NAV_Motor(void *KeyHandle, unsigned short node_id, unsigned short exp_type, signed char mode, bool homing)
{
    unsigned int error_code = 0;
    gateway = KeyHandle;
    id = node_id;
    pos_ref = 0;
    op_mode = 0;

    VCS_GetMotorType(gateway, id, &motor_type, &error_code);
    cout << "motors.cpp] motor id : " << id << endl;
    is_connected = (error_code) ? false : true; // SDO TIMEOUT  == 0x5040000

    if (is_connected)
    {
        if (exp_type && (motor_type != exp_type))
        {
#ifdef ROSCPP_ROS_H
            ROS_WARN_STREAM("NAV_Motor " << id << " : " << ((motor_type == MT_DC_MOTOR) ? "DC motor" : (motor_type == MT_EC_SINUS_COMMUTATED_MOTOR) ? "EC sine motor"
                                                                                                                                                    : "EC block motor"));
#else
            cerr << "[motors.cpp] NAV_Motor " << id << " : " << ((motor_type == MT_DC_MOTOR) ? "DC motor" : (motor_type == MT_EC_SINUS_COMMUTATED_MOTOR) ? "EC sine motor"
                                                                                                                                                         : "EC block motor")
                 << endl;
#endif
            // throw 0;
        }
        else
        {
#ifdef ROSCPP_ROS_H
            ROS_DEBUG_STREAM("NAV_Motor " << id << " : " << ((motor_type == MT_DC_MOTOR) ? "DC motor" : (motor_type == MT_EC_SINUS_COMMUTATED_MOTOR) ? "EC sine motor"
                                                                                                                                                     : "EC block motor"));
#else
            cout << "[motors.cpp] NAV_Motor " << id << " : " << ((motor_type == MT_DC_MOTOR) ? "DC motor" : (motor_type == MT_EC_SINUS_COMMUTATED_MOTOR) ? "EC sine motor"
                                                                                                                                                         : "EC block motor")
                 << endl;
#endif
        }
        if (homing && (mode == OMD_PROFILE_POSITION_MODE))
        {
            int pos;
            VCS_GetPositionIs(gateway, id, &pos, &error_code);
            VCS_ActivateHomingMode(gateway, id, &error_code);
            VCS_DefinePosition(gateway, id, pos, &error_code);
            VCS_StopHoming(gateway, id, &error_code);
        }

        if (mode)
        {
            VCS_SetOperationMode(gateway, id, mode, &error_code);
            if (error_code)
            {
                print_VCS_error(error_code, __FUNCTION__);
                op_mode = 0;
            }
            else
                op_mode = mode;
            if (mode == OMD_PROFILE_POSITION_MODE) // OMD_PROFILE_POSITION_MODE
                VCS_SetPositionProfile(gateway, id, MAX_STEER_VEL, MAX_STEER_ACCEL, MAX_STEER_ACCEL, &error_code);
            else if (mode == OMD_PROFILE_VELOCITY_MODE)
                VCS_SetVelocityProfile(gateway, id, MAX_DRIVE_ACCEL, MAX_DRIVE_DECEL, &error_code);
            if (error_code)
            {
                print_VCS_error(error_code, __FUNCTION__);
            }
        }
    }
    else
    {
#ifdef ROSCPP_ROS_HSet
        ROS_WARN_STREAM("NAV_Motor " << id << " : unresponsive");
#else
        cerr << "[motors.cpp] NAV_Motor " << id << " : unresponsive" << endl;
#endif

        if (error_code != 0x5040000)
        {
            print_VCS_error(error_code, __FUNCTION__);
            // throw 0;
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
#ifdef ROSCPP_ROS_H
                ROS_WARN_STREAM("FAULT : Node " << id << " : " << error_msg);
#else
                cerr << "[motors.cpp] FAULT : Node " << id << " : " << error_msg << endl;
#endif
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

    if ((op_mode != OMD_VELOCITY_MODE) &&
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
    VCS_GetVelocityIsAveraged(gateway, id, &current_averaged, &error_code);
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
