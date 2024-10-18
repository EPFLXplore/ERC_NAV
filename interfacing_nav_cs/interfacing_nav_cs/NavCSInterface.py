import os, launch, time, rclpy
from custom_msg.srv import ChangeModeSystem
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from std_msgs.msg import String
from sensor_msgs.msg import Joy

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
    1: "Manual",
    2: "Auto"
}

class NavCSInterface(Node):

    def __init__(self):
        super().__init__("NavCSInterfacing")

        self.mode = 'Off'

        # Change Mode
        self.cs_request = self.create_service(ChangeModeSystem, '/ROVER/change_NAV_mode', self.execute_service)
        self.mode_publisher = self.create_publisher(String, '/ROVER/NAV_mode', 1)

        # Nav 2
        self.path_nav2_launch_file = '/dev_ws/src/path_planning/launch/nav2_real.launch.py'
        self.ros2_node_name = ''
        self.start_nav2_cmd = None
        self.launch_service = None

        # Motor Health
        self.state_motor_control = State.PRIMARY_STATE_UNCONFIGURE
        self.check_motor_health = self.create_timer(0.1, self.get_state_motor_control)
        self.motor_change_service = self.create_client(ChangeState, 
                                                      '/NAV_motor_cmds/change_state')
        self.motor_check_service = self.create_client(GetState, 
                                                      '/NAV_motor_cmds/get_state')

    # ----------------------------------------------------------
    # SERVICE

    def execute_service(self, request, response):
        mode = request.mode

        if self.mode == 'Off' and mode != 'Off':
            print("ACTIVATE MANUAL OR AUTO")
            self.transition_state(Transition.CONFIGURE, "configure", self.default_transition_check_callback)

            if self.state_motor_control == State.PRIMARY_STATE_INACTIVE:
                has_started = self.launch_nav2_cmd()

                if has_started:
                    response.systems_state = Mode[mode]
                    response.error_type = 0
                    response.error_message = ""
                    self.mode = Mode[mode]
                    return response
                else:
                    response.systems_state = self.mode
                    response.error_type = 1
                    response.error_message = "error can't launch nav2"
                    return response
            
            response.systems_state = self.mode
            response.error_type = 1
            response.error_message = "error on changing the state of motors"
            return response

        if self.mode != 'Off' and mode == 'Off':
            self.transition_state(Transition.TRANSITION_CLEANUP, "cleanup", 
                                  self.default_transition_check_callback)
            
            if self.state_motor_control == State.PRIMARY_STATE_CLEANUP:
                has_stopped = self.stop_nav2_cmd()

                if has_stopped:
                    response.systems_state = Mode[mode]
                    response.error_type = 0
                    response.error_message = ""
                    self.mode = Mode[mode]
                    return response
                else:
                    response.systems_state = self.mode
                    response.error_type = 1
                    response.error_message = "error can't stop nav2"
                    return response
            
            response.systems_state = self.mode
            response.error_type = 1
            response.error_message = "error on changing the state of motors"
            return response

    # ------------------------------------------------------------------------
    # NAV2 COMMANDS STACK
    
    def generate_launch_description(self):
        
        pkg_name = "path_planning"

        pkg_share_dir = get_package_share_directory(pkg_name)
        nav2_ros_share_dir = get_package_share_directory("nav2_bringup")

        map_server_params_config_path = os.path.join(
            pkg_share_dir, "config", "map_server_params.yaml"
        )
        nav2_params_config_path = os.path.join(pkg_share_dir, "config", "nav2_params_real.yaml")

        # ------------- Launch Commands -------------
        self.start_nav2_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_ros_share_dir, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "false",
                "autostart": "true",
                "params_file": nav2_params_config_path,
                "map": map_server_params_config_path,
            }.items(),
        )

    def launch_nav2_cmd(self):
        if self.launch_service is not None:
            return False
        
        self.launch_service = launch.LaunchService()
        self.generate_launch_description()
        self.launch_service.include_launch_description(self.start_nav2_cmd)
        self.launch_service.run()

        for i in range(20):
            if '/nav2_container' not in self.get_node_names():
                time.sleep(1)
                print("Checking Nav2 cmd stack has started...")
                continue

            print("Nav2 cmd stack has started!")
            return True

        return False

    def stop_nav2_cmd(self):
        if self.launch_service is not None:
            self.launch_service.shutdown()
            self.launch_service = None
            self.start_nav2_cmd = None

        for i in range(20):
            if '/nav2_container' in self.get_node_names():
                time.sleep(1)
                self.get_logger().warning("Nav2 cmd stack is still ON...")
                continue

            self.get_logger().info("Nav2 cmd stack has finished correctly!")
            return True

        self.get_logger().error("Nav2 cmd stack HAS NOT FINISHED!")
        return False
    
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    # GET STATE MOTOR

    def get_state_motor_control(self):
        
        request = GetState.Request()
        future = self.motor_check_service.call_async(request)
        future.add_done_callback(self.get_state_callback)

    def get_state_callback(self, future):

        self.state_motor_control = future.result().current_state.id

        if self.state_motor_control == State.PRIMARY_STATE_UNCONFIGURED and self.mode != 'Off':
            # kill nav2
            has_stopped = self.stop_nav2_cmd()

            if has_stopped:
                # send information to cs and update state
                self.mode = 'Off'
                self.action = False
                mode = String()
                mode.data = "Off" 
                #self.mode_publisher.publish(mode)
                return
            else:
                self.get_logger().error("nav2 has not been shutdown")
                return
        
        if self.state_motor_control == State.PRIMARY_STATE_INACTIVE and self.mode == Mode.Off:
            self.transition_state(Transition.TRANSITION_CLEANUP, "cleanup", 
                                  self.default_transition_check_callback)

            self.action = False
            self.mode = 'Off'
    
    # ------------------------------------------------------------------------

    def transition_state(self, state, label, callback_transition):
        motor_request = ChangeState.Request()
        motor_request.transition.id = state
        motor_request.transition.label = label

        future = self.motor_change_service.call_async(motor_request)
        future.add_done_callback(lambda f: callback_transition(f, state))
    
    def default_transition_check_callback(self, future, state):
        if future.result().success:
            self.state_motor_control = state
            self.get_logger().error("NAV lifecycle change state successfull")
    
        else:
            self.get_logger().error("NAV lifecycle change state failed")

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = NavCSInterface()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        

