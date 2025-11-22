"""
INTERFACING NAV-CS 2024
-----------------------
Author: Giovanni Ranieri
Year: 2024
Description: Node handling service requests from the CS to change the mode of navigation.
             Going from Off or Manual to Auto enables nav2 stack and deactivates it otherwise.
"""

# Standard library imports
import json
import os
import time

# Third-party imports
import launch
import rclpy
import yaml

# ROS2 imports
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool

# Custom message imports
from custom_msg.msg import MotorStatus, ServoRequest
from custom_msg.srv import ChangeModeCamera, ChangeModeSystem


# ============================================================================
# Constants
# ============================================================================

# Mode integer to string mapping
MODE = {
    0: "Off",
    1: "Ackermann",
    2: "Omni",
    3: "Auto"
}

# Mode string to integer mapping (for CS responses)
MODE_TO_CS = {
    "Off": 0,
    "Ackermann": 1,
    "Omni": 2,
    "Auto": 3
}


# ============================================================================
# Main Node Class
# ============================================================================

class NavCSInterface(Node):
    """
    Interface node between Control Station (CS) and Navigation subsystem.
    
    Handles:
    - Mode transitions (Off, Ackermann, Omni, Auto)
    - Gamepad command forwarding
    - Motor status monitoring and aggregation
    - Camera control and configuration
    - Full state publishing for CS
    """

    def __init__(self):
        super().__init__("NavCSInterfacing")

        # ====================================================================
        # Load Configuration Files
        # ====================================================================
        self.nav_names = self._load_config('/home/xplore/dev_ws/src/custom_msg/config/nav_interface_names.yaml')
        self.rover_names = self._load_config('/home/xplore/dev_ws/src/custom_msg/config/rover_interface_names.yaml')
        self.el_names = self._load_config('/home/xplore/dev_ws/src/custom_msg/config/el_interface_names.yaml')
        self.cs_names = self._load_config('/home/xplore/dev_ws/src/custom_msg/config/cs_interface_names.yaml')

        # ====================================================================
        # State Variables
        # ====================================================================
        self.mode = 'Off'
        self.transitioning_state = False
        self.state_motor_control = State.PRIMARY_STATE_UNCONFIGURED
        self.last_increment = 0  # Camera servo state

        # ====================================================================
        # Callback Groups
        # ====================================================================
        group = ReentrantCallbackGroup()
        group2 = ReentrantCallbackGroup()

        # ====================================================================
        # QoS Profile (Best Effort for real-time data)
        # ====================================================================
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ====================================================================
        # Gamepad Interface
        # ====================================================================
        # Subscribe to gamepad commands from CS
        self.gamepad_sub = self.create_subscription(
            Joy,
            '/CS/GamepadCmdsNav',
            self.handle_gamepad,
            qos_profile=self.qos_profile
        )

        # Forward gamepad to motor controller
        self.gamepad_pub = self.create_publisher(
            Joy,
            self.rover_names['rover_pubsub_nav_gamepad'],
            qos_profile=self.qos_profile
        )

        # ====================================================================
        # Camera Servo Control
        # ====================================================================
        self.cam_cmd_pub = self.create_publisher(
            ServoRequest, 
            self.el_names["SERVO_REQ_TOPIC"], 
            1
        )

        # ====================================================================
        # Speed Control Interface
        # ====================================================================
        # Subscribe to speed changes from CS
        self.speed_sub = self.create_subscription(
            Float32,
            self.cs_names['cs_pubsub_speed_rover'],
            self.handle_speed_change,
            10
        )

        # Forward speed commands to motor controller
        self.speed_pub = self.create_publisher(
            Float32,
            self.rover_names["rover_change_nav_speed"],
            1
        )

        # ====================================================================
        # Motor Status Monitoring
        # ====================================================================
        # Subscribe to motor status for state aggregation
        self.motor_status_sub = self.create_subscription(
            MotorStatus,
            self.nav_names['nav_motors_status'],
            self.handle_motor_status,
            qos_profile=self.qos_profile
        )
    
        # Storage for motor data
        self.motor_data = {
            'wheels': {
                'front_left': {},
                'front_right': {},
                'rear_left': {},
                'rear_right': {}
            }
        }

        # Hardware constants (for motor calculations)
        self.wheels_radius = 0.1325  # meters
        self.gear_ratio = 1.0 / 53.0

        # ====================================================================
        # Full State Publishing
        # ====================================================================
        # Publish comprehensive state for CS (replaces ROVER aggregation)
        self.state_publisher = self.create_publisher(
            String, 
            '/NAV/State', 
            qos_profile=self.qos_profile
        )
        self.timer_full_state = self.create_timer(2.0, self.publish_full_state)

        # Initialize full state dictionary
        self.nav_full_state = {
            "state": {
                "mode": "Off",
                "lifecycle_state": "unconfigured"
            },
            "wheels": {
                "front_left": {},
                "front_right": {},
                "rear_left": {},
                "rear_right": {}
            },
            "localization": {
                "position": {"x": 0.0, "y": 0.0}
            },
            "software": {
                "nodes": {}
            },
            "cameras": {},
            "hardware": {
                "stats_nav": {}
            }
        }

        # ====================================================================
        # Mode Change Service
        # ====================================================================
        self.cs_request = self.create_service(
            ChangeModeSystem, 
            self.rover_names['rover_change_nav_mode'], 
            self.execute_service, 
            callback_group=group2
        )
        
        # Mode status publisher
        self.mode_publisher = self.create_publisher(
            String, 
            self.nav_names['system_status'], 
            1
        )
        self.timer_mode = self.create_timer(2.0, self.pub_state)

        # ====================================================================
        # Motor Lifecycle Management
        # ====================================================================
        # Timer to check motor health
        self.check_motor_health = self.create_timer(2.0, self.get_state_motor_control)
        
        # Service clients for motor lifecycle control
        self.motor_change_service = self.create_client(
            ChangeState, 
            '/NAV_motor_cmds/change_state', 
            callback_group=group2
        )
        self.motor_check_service = self.create_client(
            GetState, 
            '/NAV_motor_cmds/get_state'
        )
        
        # ====================================================================
        # Camera Services
        # ====================================================================
        # Camera mode change service
        self.camera_mode_srv = self.create_service(
            ChangeModeCamera,
            '/NAV/ChangeModeCamera',
            self.handle_camera_mode_change,
            callback_group=group2
        )

        # RGBD depth mode service
        self.rgbd_mode_srv = self.create_service(
            SetBool,
            '/NAV/ChangeModeRGB',
            self.handle_rgbd_mode,
            callback_group=group2
        )

        # Screenshot service (subscription for trigger)
        self.screenshot_srv = self.create_subscription(
            bool,
            '/NAV/ScreenshotAllCameras',
            self.handle_screenshot,
            qos_profile=self.qos_profile
        )

        # Camera control clients
        self.camera_nav_0 = self.create_client(SetBool, '/NAV/req_camera_nav_0')
        self.camera_nav_1 = self.create_client(SetBool, '/NAV/req_camera_nav_1')
        self.camera_nav_2 = self.create_client(SetBool, '/NAV/req_camera_nav_2')
        
        # RGBD and screenshot clients
        self.rgbd_client = self.create_client(SetBool, '/NAV/depth_req_camera_nav_0')
        self.screenshot_client = self.create_client(SetBool, '/NAV/screenshot_camera_nav_0')

    # ========================================================================
    # Utility Methods
    # ========================================================================

    def _load_config(self, filepath):
        """Load YAML configuration file and return parameters."""
        with open(filepath, 'r') as file:
            return yaml.safe_load(file)["/**"]["ros__parameters"]

    def pub_state(self):
        """Publish current mode to system status topic."""
        m = String()
        m.data = self.mode
        self.mode_publisher.publish(m)

    # ========================================================================
    # Camera Management Handlers
    # ========================================================================

    def handle_camera_mode_change(self, request, response):
        """
        Forward camera activation commands to specific camera nodes.
        
        Args:
            request: ChangeModeCamera.Request with camera_name and activate
            response: ChangeModeCamera.Response with error_type and message
        """
        camera_name = request.camera_name
        activate = request.activate
        
        # Map camera names to clients
        camera_clients = {
            'camera_nav_0': self.camera_nav_0,
            'camera_nav_1': self.camera_nav_1,
            'camera_nav_2': self.camera_nav_2,
        }
        
        if camera_name in camera_clients:
            req = SetBool.Request(data=activate)
            future = camera_clients[camera_name].call_async(req)
            response.error_type = 0
            response.error_message = f"Camera {camera_name} command sent"
        else:
            response.error_type = 1
            response.error_message = f"Unknown camera: {camera_name}"
        
        return response
    
    def handle_rgbd_mode(self, request, response):
        """
        Toggle RGBD depth mode for the RGBD camera.
        
        Args:
            request: SetBool.Request with data (True/False)
            response: SetBool.Response with success status
        """
        future = self.rgbd_client.call_async(request)
        response.success = True
        response.message = "RGBD mode toggle requested"
        return response
    
    def handle_screenshot(self, msg):
        """
        Trigger screenshot capture for NAV cameras.
        
        Args:
            msg: Bool message, if True triggers screenshot
        """
        if msg.data:
            self.screenshot_client.call_async(SetBool.Request(data=True))

    # ========================================================================
    # Gamepad and Manual Control Handlers
    # ========================================================================

    def handle_gamepad(self, msg):
        """
        Process and forward gamepad commands when in manual mode.
        
        Performs security checks before forwarding:
        - Verifies message is for navigation subsystem
        - Checks mode is manual (Ackermann or Omni)
        - Ensures motors are in ready state
        
        Args:
            msg: Joy message from CS with axes and buttons
        """
        # Security check: verify message is for navigation
        if int(msg.buttons[0]) != 1:
            return
        
        # Only forward if in manual mode (Ackermann or Omni)
        if self.mode not in ['Ackermann', 'Omni']:
            return
        
        # Safety: verify motors are ready
        if self.state_motor_control != State.PRIMARY_STATE_INACTIVE:
            self.get_logger().warning("Motors not ready, ignoring gamepad")
            return
        
        # Handle camera servo controls (buttons[2] and buttons[3])
        self.handle_camera_servo(msg)
        
        # Forward gamepad to motor controller
        self.gamepad_pub.publish(msg)

    def handle_speed_change(self, msg):
        """
        Forward speed change to motor controller with validation.
        
        Validates speed is within safe operating range before forwarding.
        
        Args:
            msg: Float32 message with speed value [0.5, 2.31]
        """
        # Validate speed range
        if msg.data <= 0.5 or msg.data >= 2.31:
            self.get_logger().warning(f"Speed {msg.data} out of range [0.5, 2.31]")
            return
        
        self.get_logger().info(f"Speed change requested: {msg.data}")
        self.speed_pub.publish(msg)

    def handle_camera_servo(self, msg):
        """
        Handle front camera servo angle adjustment from gamepad buttons.
        
        Uses buttons[2] (up) and buttons[3] (down) for servo control.
        Implements debouncing to prevent rapid repeated commands.
        
        Args:
            msg: Joy message with button states
        """
        # Arrow up/down buttons
        increase = msg.buttons[2]  # +1
        decrease = msg.buttons[3]  # -1
        
        # Reset debounce state when buttons released
        if increase == 0 and decrease == 0:
            self.last_increment = 0
            return
        
        # Debounce: ignore if already processing
        if self.last_increment == 1 and (increase == 1 or decrease == 1):
            return
        
        # Create servo request
        angle = ServoRequest()
        angle.id = 1
        angle.zero_in = False
        
        if increase == 1:
            angle.increment = 20
            self.cam_cmd_pub.publish(angle)
            self.last_increment = 1
        elif decrease == 1:
            angle.increment = -20
            self.cam_cmd_pub.publish(angle)
            self.last_increment = 1

    # ========================================================================
    # Motor Status Monitoring
    # ========================================================================

    def handle_motor_status(self, msg):
        """
        Parse motor status messages and update internal state.
        
        Extracts and processes data for all 4 wheels (drive + steer motors).
        Converts raw motor data to human-readable units:
        - Velocity (RPS) -> Speed (m/s)
        - Position (encoder counts) -> Steering angle (degrees)
        
        Motor mapping:
        - Front Left: drive=0, steer=4
        - Front Right: drive=1, steer=5
        - Rear Right: drive=2, steer=6
        - Rear Left: drive=3, steer=7
        
        Args:
            msg: MotorStatus message with arrays of motor data
        """
        # Only process if not Off
        if self.mode == 'Off':
            return
        
        # Conversion factor: RPS to m/s
        rps_to_ms = 2 * 3.1415 * self.wheels_radius / 60.0
        
        # Extract data per wheel
        # Front Left (drive=0, steer=4)
        self.motor_data['wheels']['front_left'] = {
            'current_driving': abs(msg.average_current[0]),
            'current_steering': abs(msg.average_current[4]),
            'speed': abs(round(msg.velocity[0] * rps_to_ms * self.gear_ratio, 1)),
            'steering_angle': int(float(msg.position[0]) / 65536 * 360),
            'driving_motor_state': msg.state[0],
            'steering_motor_state': msg.state[4],
            'driving_fault': msg.fault_state[0],
            'steering_fault': msg.fault_state[4]
        }

        # Front Right (drive=1, steer=5)
        self.motor_data['wheels']['front_right'] = {
            'current_driving': abs(msg.average_current[1]),
            'current_steering': abs(msg.average_current[5]),
            'speed': abs(round(msg.velocity[1] * rps_to_ms * self.gear_ratio, 1)),
            'steering_angle': int(float(msg.position[1]) / 65536 * 360),
            'driving_motor_state': msg.state[1],
            'steering_motor_state': msg.state[5],
            'driving_fault': msg.fault_state[1],
            'steering_fault': msg.fault_state[5]
        }

        # Rear right (drive=2, steer=6)
        self.motor_data['wheels']['rear_right'] = {
            'current_driving': abs(msg.average_current[2]),
            'current_steering': abs(msg.average_current[6]),
            'speed': abs(round(msg.velocity[2] * rps_to_ms * self.gear_ratio, 1)),
            'steering_angle': int(float(msg.position[2]) / 65536 * 360),
            'driving_motor_state': msg.state[2],
            'steering_motor_state': msg.state[6],
            'driving_fault': msg.fault_state[2],
            'steering_fault': msg.fault_state[6]
        }

        # Rear left (drive=3, steer=7)
        self.motor_data['wheels']['rear_left'] = {
            'current_driving': abs(msg.average_current[3]),
            'current_steering': abs(msg.average_current[7]),
            'speed': abs(round(msg.velocity[3] * rps_to_ms * self.gear_ratio, 1)),
            'steering_angle': int(float(msg.position[3]) / 65536 * 360),
            'driving_motor_state': msg.state[3],
            'steering_motor_state': msg.state[7],
            'driving_fault': msg.fault_state[3],
            'steering_fault': msg.fault_state[7]
        }

    # ========================================================================
    # State Publishing
    # ========================================================================

    def publish_full_state(self):
        """
        Publish comprehensive NAV state for CS.
        
        Aggregates all NAV subsystem data into a single JSON message:
        - Mode and lifecycle state
        - Wheel/motor status
        - Active node information
        - Localization data
        - Camera status
        - Hardware stats
        """
        
        # Update mode
        self.nav_full_state["state"]["mode"] = self.mode
        self.nav_full_state["state"]["lifecycle_state"] = self.get_lifecycle_string()
        
        # Update motor/wheel data
        self.nav_full_state["wheels"] = self.motor_data.get('wheels', {})
        
        # Update active nodes (check which NAV nodes are running)
        self.nav_full_state["software"]["nodes"] = self.check_nav_nodes()
        
        # Publish as JSON string
        msg = String()
        msg.data = json.dumps(self.nav_full_state)
        self.state_publisher.publish(msg)
    
    def get_lifecycle_string(self):
        """
        Convert lifecycle state ID to human-readable string.
        
        Returns:
            str: Lifecycle state name or "unknown"
        """
        states = {
            State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
            State.PRIMARY_STATE_INACTIVE: "inactive",
            State.PRIMARY_STATE_ACTIVE: "active"
        }
        return states.get(self.state_motor_control, "unknown")
    
    def check_nav_nodes(self):
        """
        Query and identify active NAV-related nodes in the system.
        
        Returns:
            dict: Dictionary of NAV node names and their status
        """
        nodes = {}
        node_names = self.get_node_names()  # ROS2 API
        
        for node_name in node_names:
            if 'NAV' in node_name or 'nav' in node_name.lower():
                nodes[node_name] = {
                    "name": node_name,
                    "status": True
                }
        
        return nodes

    # ========================================================================
    # Mode Change Service Handler
    # ========================================================================

    def execute_service(self, request, response):
        """
        Main service handler for mode changes from CS.
        
        Handles three types of transitions:
        1. Off -> Manual (Ackermann/Omni): Configure motors
        2. Manual <-> Manual: Direct mode change
        3. Any -> Off: Cleanup motors
        
        Args:
            request: ChangeModeSystem.Request with mode integer
            response: ChangeModeSystem.Response with result
            
        Returns:
            response: Populated response with new mode and status
        """
        mode = request.mode

        # Check if already in requested mode
        if self.mode == MODE[mode]:
            response.new_mode = mode
            response.error_type = 1
            response.error_message = "already in that state"
            return response

        # Case 1: Transition from Off to Manual mode (Ackermann or Omni)
        # This requires motor lifecycle configuration
        if self.mode == 'Off' and (MODE[mode] == 'Ackermann' or MODE[mode] == 'Omni'):
            self.transitioning_state = True
            self.transition_state(
                Transition.TRANSITION_CONFIGURE, 
                "configure", 
                State.PRIMARY_STATE_INACTIVE, 
                self.default_transition_check_callback,
                mode,
                "NAV lifecycle change state successfully",
                response
            )
            
            # Wait for transition to complete
            while self.transitioning_state:
                time.sleep(1)

            return response 
        
        # Case 2: Change between manual modes (no lifecycle change needed)
        if self.mode != "Off" and MODE[mode] != 'Off':
            self.change_mode(mode, 0, response, "no errors")
            return response 

        # Case 3: Transition to Off mode from any active state
        # This requires motor lifecycle cleanup
        if self.mode != 'Off' and MODE[mode] == 'Off':
            self.get_logger().info("Transitioning to Off mode")
            self.transitioning_state = True
            self.transition_state(
                Transition.TRANSITION_CLEANUP, 
                "cleanup", 
                State.PRIMARY_STATE_UNCONFIGURED, 
                self.default_transition_check_callback,
                mode,
                "NAV lifecycle change state successfully",
                response
            )
            
            # Wait for transition to complete
            while self.transitioning_state:
                time.sleep(1)

            return response

    # ========================================================================
    # Motor Lifecycle Management
    # ========================================================================

    def get_state_motor_control(self):
        """
        Periodically check motor controller lifecycle state.
        
        Called by timer to monitor motor health. If motors unexpectedly
        transition to unconfigured while in manual mode, forces mode to Off.
        """
        request = GetState.Request()
        future = self.motor_check_service.call_async(request)
        future.add_done_callback(self.get_state_callback)

    def get_state_callback(self, future):
        """
        Process motor state query response.
        
        Safety check: If motors become unconfigured while in manual mode,
        automatically transition system to Off mode.
        
        Args:
            future: Future containing GetState.Response
        """
        self.state_motor_control = future.result().current_state.id
        
        # Safety check: if motors go off while in Manual mode, force Off
        if self.state_motor_control == State.PRIMARY_STATE_UNCONFIGURED and self.mode != 'Off':
            self.mode = 'Off'
            return

    def transition_state(self, state, label, next_state, callback_transition, mode, message_response, response):
        """
        Execute motor lifecycle state transition.
        
        Sends lifecycle transition command to motor controller and sets up
        callback to handle the result.
        
        Args:
            state: Transition ID (CONFIGURE/CLEANUP)
            label: Transition label string
            next_state: Expected state after transition
            callback_transition: Callback function (unused)
            mode: Target mode integer
            message_response: Success message
            response: Service response object to populate
        """
        motor_request = ChangeState.Request()
        motor_request.transition.id = state
        motor_request.transition.label = label

        future = self.motor_change_service.call_async(motor_request)
        future.add_done_callback(
            lambda f: self.default_transition_check_callback(
                f, next_state, mode, message_response, response
            )
        )
    
    def default_transition_check_callback(self, future, next_state, mode, message_response, response):
        """
        Handle result of lifecycle transition.
        
        On success: Updates state and changes mode
        On failure: Reverts to current mode with error
        
        Args:
            future: Future containing ChangeState.Response
            next_state: Expected lifecycle state after transition
            mode: Target mode integer
            message_response: Success message
            response: Service response object to populate
        """
        if future.result().success:
            self.state_motor_control = next_state
            self.change_mode(mode, 0, response, message_response)
            self.get_logger().info(message_response)
        else:
            self.change_mode(MODE_TO_CS[self.mode], 1, response, "NAV lifecycle change state failed")
            self.get_logger().error("NAV lifecycle change state failed")
        
        self.transitioning_state = False
    
    def change_mode(self, mode, error_type, response, message_response):
        """
        Update internal mode and populate service response.
        
        Args:
            mode: New mode integer
            error_type: 0 for success, 1 for error
            response: Service response to populate
            message_response: Status message
        """
        self.mode = MODE[mode]
        response.new_mode = mode
        response.error_type = error_type
        response.error_message = message_response

# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):

    rclpy.init(args=args)

    nav_cs_interface = NavCSInterface()

    # Use multi-threaded executor for concurrent service handling
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nav_cs_interface)

    try:
        executor.spin()
    finally:
        nav_cs_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()