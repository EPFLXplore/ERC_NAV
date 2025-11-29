#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <iostream>
#include <math.h>
#include <tuple>
/*
#ifndef TESTING
#include "ros/ros.h"
#endif

*/
#include "EposCmd.h"
// #include "thermal_modeling.hpp"

class NAV_Motor
{
private:
    void *gateway;
    unsigned short id;
    bool is_connected;
    unsigned short motor_type;
    signed char op_mode;
    long pos_ref;

public:
    /* NAV_Motor
     * brief :  class of objects to represent a NAV motor
     * param    KeyHandle   handle to the USB gateway
     * param    node_id     CAN Node ID of the controller
     * param    exp_type    expected motor type (0 for no expected type)
     */
    NAV_Motor(void *KeyHandle, unsigned short node_id, unsigned short exp_type, signed char mode, bool homing);
    NAV_Motor(void *KeyHandle, unsigned short node_id, unsigned short exp_type) : NAV_Motor(KeyHandle, node_id, exp_type, 0, false) {};

    /* get_id
     * brief :  return the CAN id of the node
     */
    unsigned short get_id();

    /* get_id
     * brief :  return wheather the node is connected
     */
    bool connected();

    /* disconnect
     * brief :  manually disconnect a motor
     */
    void disconnect();

    /* set_operational_mode
     * brief :  set the operational mode of the controller (position / velocity, etc)
     */
    bool set_operational_mode(signed char mode);

    /* set_output_state
     * brief :  set the output state to active / inactive
     */
    bool set_output_state(bool output_active);

    bool fault_state(unsigned int *error_code);

    bool fault_state();

    bool homing();

    int get_encoder_pulse();
    /* is_faulty
     * brief :  check if the device is currently in a fault state
     * param :  verbose     ouput the faults onto the terminal
     * return : in_fault    if the device is faulty
     */
    bool is_faulty(bool verbose);

    bool clear_fault();

    /* set_position_ref
     * brief :  set the position reference for the motor to track
     *          if the motor is not yet in the correct mode, this function
     *          automatically sets the controller into the right mode
     */

    bool reset_device();
    bool set_position_ref(long pos);

    /* reset_position_counter
     * brief :  reset the internal position reference
     */
    bool reset_position_counter();

    /* get_position_is
     * brief :  get the current position
     */
    int get_position_is();

    bool has_reached();

    /* set_velocity_ref
     * brief :  set the position reference for the motor to track
     *          if the motor is not yet in the correct mode, this function
     *          automatically sets the controller into the right mode
     */
    bool set_velocity_ref(long vel);

    /* get_velocity_is
     * brief :  get the current velocity
     */
    int get_velocity_is();

    /* get_current_is
     * brief :  get the current current in [mA]
     */
    int get_current_is();

    /* get_current_is_averaged
     * brief :  get the averaged current current in [mA]
     */
    int get_current_is_averaged();

    /* get_efficiency
     * brief :  get the current efficiency in %
     */
    int get_efficiency();


};

/* get_device_name_selection
    * brief :  VCS_GetDeviceNameSelection returns all available device names.
    You pass start_of_selection = true if you want the very first device name in the list
    */

// - device_name: the device name (as a std::string)
// - protocol_stack_name_sel: an output std::string for the protocol stack name
// - start_of_selection: a bool flag (true for the first call, false for subsequent calls)
// - max_str_size: maximum allowed size for the string (your buffer size)
// - end_of_selection: an output parameter indicating whether there are no more strings (nonzero means done)
// - error_code: an output error code from the EPOS API
bool get_device_name_selection(bool start_of_selection, std::string &device_name, int &end_of_selection);


/* 
get_protocol_stack_name_selection
    * brief :  VCS_GetProtocolStackNameSelection returns all available protocol stack names.
*/
bool get_protocol_stack_name_selection(std::string &device_name,
    std::string &protocol_stack_name_sel,
    bool start_of_selection,
    unsigned short max_str_size,
    int &end_of_selection,
    unsigned int &error_code);

/* get_interface_name_selection
 * brief :  VCS_GetInterfaceNameSelection returns all available interface names
 * param :  device_name          The device name you are querying
 * param :  protocol_stack_name  The protocol stack name you are using
 * param :  start_of_selection   True to get the first interface name, false to get the next
 * param :  interface_name       [out] Will hold the interface name returned
 * param :  max_str_size         The maximum length for the interface name
 * param :  end_of_selection     [out] True if no more names are available
 * param :  error_code           [out] EPOS library error code
 * return : true if the call succeeded, false otherwise
 */
 bool get_interface_name_selection(const std::string &device_name,
    const std::string &protocol_stack_name,
    bool start_of_selection,
    std::string &interface_name,
    unsigned short max_str_size,
    bool &end_of_selection,
    unsigned int &error_code);

/**
 * brief Retrieves the next available port name for a given device, protocol stack, and interface
 *        from the EPOS library.
 *
 * param device_name         [in]  The device name (e.g. "EPOS4").
 * param protocol_stack_name [in]  The protocol stack name (e.g. "MAXON SERIAL V2").
 * param interface_name      [in]  The interface name (e.g. "USB").
 * param start_of_selection  [in]  Pass true for the first call, false for subsequent calls.
 * param port_name           [out] The returned port name, if any.
 * param max_str_size        [in]  The maximum allowed length of the port name string.
 * param end_of_selection    [out] Set to true if there are no more port names left to read.
 * param error_code          [out] Stores an EPOS library error code if the call fails.
 * 
 * return true if the call succeeded; false otherwise.
 */
 bool get_port_name_selection(const std::string &device_name,
                              const std::string &protocol_stack_name,
                              const std::string &interface_name,
                              bool start_of_selection,
                              std::string &port_name,
                              unsigned short max_str_size,
                              int &end_of_selection,
                              unsigned int &error_code);



/* open_gateway
 * brief :  open a gateway through which to adress a maxon CANopen bus
CS_OpenDevice opens the port to send and receive commands. Ports can be RS232, USB, and CANopen interfaces.
For correct designations on DeviceName, ProtocolStackName, InterfaceName, and PortName, use the
functions Get Device Name Selection, Get Protocol Stack Name Selection, Get Interface Name
Selection, and Get Port Name Selection.
 */
void *open_gateway(void);

/* close_gateway
 * brief :  safely close a gateway which was used to adress a maxon CANopen bus
 */
void close_gateway(void *gateway);

#endif