import os, launch, time, rclpy, yaml
from custom_msg.srv import ChangeModeSystem
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

"""
INTERFACING NAV-CS 2024
-----------------
Author: Giovanni Ranieri
Year: 2024
Description: Node handling service requests from the CS to change the mode of navigation.
Going from Off or Manual to Auto enables nav2 stack and deactivate it else.
"""

Mode = {
    0: "Off",
    1: "Ackermann",
    2: "Omni",
    3: "Auto"
}

Mode_to_CS = {
    "Off": 0,
    "Ackermann": 1,
    "Omni": 2,
    "Auto": 3
}

class NavCSInterface(Node):

    def __init__(self):
        super().__init__("NavCSInterfacing")

        # Declare and get simulation mode parameter
        self.declare_parameter('sim_mode', False)
        self.sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value

        with open('/home/xplore/dev_ws/src/custom_msg/config/nav_interface_names.yaml', 'r') as file:
            self.nav_names = yaml.safe_load(file)["/**"]["ros__parameters"]

        with open('/home/xplore/dev_ws/src/custom_msg/config/rover_interface_names.yaml', 'r') as file:
            self.rover_names = yaml.safe_load(file)["/**"]["ros__parameters"]

        self.mode = 'Off'
        self.transitioning_state = False

        group = ReentrantCallbackGroup()
        group2 = ReentrantCallbackGroup()

        # Change Mode
        self.cs_request = self.create_service(ChangeModeSystem, self.rover_names['rover_change_nav_mode'], self.execute_service, callback_group=group2)
        self.mode_publisher = self.create_publisher(String, self.nav_names['system_status'], 1)
        # self.timer_mode = self.create_timer(2.0, self.pub_state)

        # Motor Health - only create in real robot mode
        self.state_motor_control = State.PRIMARY_STATE_INACTIVE if self.sim_mode else State.PRIMARY_STATE_UNCONFIGURED
        
        if not self.sim_mode:
            self.check_motor_health = self.create_timer(2.0, self.get_state_motor_control)
            self.motor_change_service = self.create_client(ChangeState, 
                                                          '/NAV_motor_cmds/change_state', callback_group=group2)
            self.motor_check_service = self.create_client(GetState, 
                                                          '/NAV_motor_cmds/get_state')
        else:
            self.get_logger().info("Running in simulation mode - skipping motor health checks")

    def pub_state(self):
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.get_logger().info(f"DEBUG [{timestamp}]: pub_state called - publishing mode: '{self.mode}'")

        m = String()
        m.data = self.mode
        self.mode_publisher.publish(m)
    
    # ----------------------------------------------------------
    # SERVICE

    def execute_service(self, request, response):
        mode = request.mode
       
        if self.mode == Mode[mode]:
            response.new_mode = mode
            response.error_type = 1
            response.error_message = "already in that state"
            return response

        self.get_logger().info(f"DEBUG: Service request received - mode: {mode}, Mode[mode]: {Mode[mode]}")
        self.get_logger().info(f"DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDdd: Current mode: {self.mode}, sim_mode: {self.sim_mode}")

        
        # Transition to Manual or Omni mode from Off
        # Warning, don't transition directly to Auto!
        if self.mode == 'Off' and (Mode[mode] == 'Ackermann' or Mode[mode] == 'Omni'):
            self.transitioning_state = True
            self.transition_state(Transition.TRANSITION_CONFIGURE, "configure", 
                                  State.PRIMARY_STATE_INACTIVE, 
                                  self.default_transition_check_callback,
                                  mode,
                                  "NAV lifecycle change state successfully",
                                  response)
            
            while(self.transitioning_state):
                time.sleep(1)

            return response 
        
        # Change of modes for manual control
        if (self.mode != "Off" and Mode[mode] != 'Off'):
            self.change_mode(mode, 0, response, "no errors")
            return response 

        
        # Transition to Off mode from any state
        if (self.mode != 'Off') and Mode[mode] == 'Off':
            self.get_logger().info(f"in transition from X to Off")
            self.transitioning_state = True
            self.transition_state(Transition.TRANSITION_CLEANUP, "cleanup", 
                                  State.PRIMARY_STATE_UNCONFIGURED, 
                                  self.default_transition_check_callback,
                                  mode,
                                  "NAV lifecycle change state successfully",
                                  response)
            
            while(self.transitioning_state):
                time.sleep(1)

            return response
    
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    # GET STATE MOTOR

    def get_state_motor_control(self):
        self.get_logger().info(f"DEBUG: get_state_motor_control called - sim_mode: {self.sim_mode} dddddddddddddddddddddd")

        # Skip motor state checking in simulation
        if self.sim_mode:
            self.get_logger().info("DEBUG: Skipping motor control check due to simulation mode")
            return
            
        request = GetState.Request()
        future = self.motor_check_service.call_async(request)
        future.add_done_callback(self.get_state_callback)

    def get_state_callback(self, future):
        self.get_logger().info(f"DEBUG: get_state_callback called - sim_mode: {self.sim_mode} AAAAAAAAAAAAAAAAAAAAAAAa")

        # Skip motor state callbacks in simulation
        if self.sim_mode:
            self.get_logger().info("DEBUG: Skipping state callback due to simulation mode")
            return
            
        try:
            self.state_motor_control = future.result().current_state.id
            
            # If for some reasons the motors go off while being in Manual mode
            if self.state_motor_control == State.PRIMARY_STATE_UNCONFIGURED and self.mode != 'Off':
                self.mode = 'Off'
                self.get_logger().warn("Motor control lost - reverting to Off mode")
                return
        except Exception as e:
            self.get_logger().error(f"Motor state check failed: {e}")
    
    # ------------------------------------------------------------------------

    def transition_state(self, state, label, next_state, callback_transition, mode, message_response, response):
        # In simulation mode, simulate successful transition
        
        self.get_logger().info(f"DEBUG: transition_state called - mode: {mode}, Mode[mode]: {Mode[mode]}, sim_mode: {self.sim_mode}")

        
        if self.sim_mode:
            self.get_logger().info(f"Simulating transition to {Mode[mode]}")
            self.state_motor_control = next_state
            self.change_mode(mode, 0, response, message_response)
            self.transitioning_state = False
            return
        
        # Real robot mode - use actual motor service
        self.get_logger().info(f"DEBUG: Taking real robot path (not simulation)")
        motor_request = ChangeState.Request()
        motor_request.transition.id = state
        motor_request.transition.label = label
        self.get_logger().info(f"Transitioning motor state: {label}")

        future = self.motor_change_service.call_async(motor_request)
        future.add_done_callback(lambda f: self.default_transition_check_callback(f, next_state, mode, message_response, response))

    def default_transition_check_callback(self, future, next_state, mode, message_response, response):
        self.get_logger().info(f"Transition callback executed")
        
        if future.result().success:
            self.get_logger().info(f"Transition successful")
            self.state_motor_control = next_state
            self.change_mode(mode, 0, response, message_response)
            self.get_logger().info(message_response)
        else:
            self.change_mode(Mode_to_CS[self.mode], 1, response, "NAV lifecycle change state failed")
            self.get_logger().error("NAV lifecycle change state failed")
        
        self.transitioning_state = False
    
    def change_mode(self, mode, error_type, response, message_response):
        self.get_logger().info(f"DEBUG: change_mode called - mode: {mode}, Mode[mode]: {Mode[mode]}")
        self.mode = Mode[mode]
        #self.pub_state()
        response.new_mode = mode
        response.error_type = error_type
        response.error_message = message_response

def main(args=None):
    rclpy.init(args=args)

    nav_cs_interface = NavCSInterface()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nav_cs_interface)

    executor.spin()

    nav_cs_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    
# def generate_launch_description(self):
        
#         pkg_name = "path_planning"

#         pkg_share_dir = get_package_share_directory(pkg_name)
#         nav2_ros_share_dir = get_package_share_directory("nav2_bringup")

#         map_server_params_config_path = os.path.join(
#             pkg_share_dir, "config", "map_server_params.yaml"
#         )
#         nav2_params_config_path = os.path.join(pkg_share_dir, "config", "nav2_params_real.yaml")

#         # ------------- Launch Commands -------------
#         self.start_nav2_cmd = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(nav2_ros_share_dir, "launch", "bringup_launch.py")
#             ),
#             launch_arguments={
#                 "use_sim_time": "false",
#                 "autostart": "true",
#                 "params_file": nav2_params_config_path,
#                 "map": map_server_params_config_path,
#             }.items(),
#         )

#     def launch_nav2_cmd(self):
#         if self.launch_service is not None:
#             return False
        
#         self.launch_service = launch.LaunchService()
#         self.generate_launch_description()
#         self.launch_service.include_launch_description(self.start_nav2_cmd)
#         self.launch_service.run()

#         for i in range(20):
#             if '/nav2_container' not in self.get_node_names():
#                 time.sleep(1)
#                 print("Checking Nav2 cmd stack has started...")
#                 continue

#             print("Nav2 cmd stack has started!")
#             return True

#         return False

#     def stop_nav2_cmd(self):
#         if self.launch_service is not None:
#             self.launch_service.shutdown()
#             self.launch_service = None
#             self.start_nav2_cmd = None

#         for i in range(20):
#             if '/nav2_container' in self.get_node_names():
#                 time.sleep(1)
#                 self.get_logger().warning("Nav2 cmd stack is still ON...")
#                 continue

#             self.get_logger().info("Nav2 cmd stack has finished correctly!")
#             return True

#         self.get_logger().error("Nav2 cmd stack HAS NOT FINISHED!")
#         return False

# Transition to Auto mode from Off
'''
if self.mode == 'Off' and Mode[mode] == 'Auto':
    self.transition_state(Transition.TRANSITION_CONFIGURE, "configure", 
                            State.PRIMARY_STATE_INACTIVE, self.default_transition_check_callback)
    
    if self.state_motor_control == State.PRIMARY_STATE_INACTIVE:
        has_started = self.launch_nav2_cmd()

        if has_started:
            response.new_mode = Mode[mode]
            response.error_type = 0
            response.error_message = "Transition to Auto mode successful"
            self.mode = Mode[mode]
            return response
        else:
            self.transition_state(Transition.TRANSITION_CLEANUP, "cleanup", 
                                    State.PRIMARY_STATE_UNCONFIGURED, self.default_transition_check_callback)
            response.new_mode = Mode_to_CS[self.mode]
            response.error_type = 1
            response.error_message = "error can't launch nav2"
            return response
        
    else:
        response.new_mode = Mode_to_CS[self.mode]
        response.error_type = 1
        response.error_message = "NAV lifecycle change state failed"
        return response

# Transition to Off mode from Auto
if self.mode == 'Auto' and Mode[mode] == 'Off':
    self.transition_state(Transition.TRANSITION_CLEANUP, "cleanup", 
                            State.PRIMARY_STATE_UNCONFIGURED, self.default_transition_check_callback)
    
    if self.state_motor_control == State.PRIMARY_STATE_UNCONFIGURED:
        
        has_stopped = self.stop_nav2_cmd()

        if has_stopped:
            response.new_mode = Mode[mode]
            response.error_type = 0
            response.error_message = "Transition to Off mode successful"
            self.mode = Mode[mode]
            return response
        else:
            response.new_mode = Mode_to_CS[self.mode]
            response.error_type = 1
            response.error_message = "error can't stop nav2"
            return response
        
    else:
        response.new_mode = Mode_to_CS[self.mode]
        response.error_type = 1
        response.error_message = "NAV lifecycle change state failed"
        return response
'''
