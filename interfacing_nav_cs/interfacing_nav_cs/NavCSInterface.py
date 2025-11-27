"""
INTERFACING NAV-CS 2024
-----------------------
Initial Author: Giovanni Ranieri
Modifying Author: Arno Laurie
Year: 2025
Description: Node handling service requests from the CS to change the mode of navigation.
             Going from Off or Manual to Auto enables nav2 stack and deactivates it otherwise.


What this file does:
- Gamepad forwarding to nav
- Front servo angle publisher
- speed change subscriber and publisher
- subscribes to all info from nav and publishes it as a single summarizing message at 2hz
- camera rgb activation service clients
- camera depth activation service clients
- camera screenshot topic subscribing

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
from std_msgs.msg import Bool, Float32, String
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

# Configuration file paths
CONFIG_BASE_PATH = '/home/xplore/dev_ws/src/custom_msg/config'
NAV_CONFIG_FILE = f'{CONFIG_BASE_PATH}/nav_interface_names.yaml'
ROVER_CONFIG_FILE = f'{CONFIG_BASE_PATH}/rover_interface_names.yaml'
EL_CONFIG_FILE = f'{CONFIG_BASE_PATH}/el_interface_names.yaml'
CS_CONFIG_FILE = f'{CONFIG_BASE_PATH}/cs_interface_names.yaml'

# Topic names
CS_GAMEPAD_TOPIC = '/CS/GamepadCmdsNav'
NAV_STATE_TOPIC = '/NAV/State'
CAMERA_MODE_SERVICE = '/NAV/ChangeModeCamera'
RGBD_MODE_SERVICE = '/NAV/ChangeModeRGB'
SCREENSHOT_TOPIC = '/NAV/ScreenshotAllCameras'

# Camera service names
CAMERA_NAV_0_SERVICE = '/NAV/req_camera_nav_0'
CAMERA_NAV_1_SERVICE = '/NAV/req_camera_nav_1'
CAMERA_NAV_2_SERVICE = '/NAV/req_camera_nav_2'
RGBD_DEPTH_SERVICE = '/NAV/depth_req_camera_nav_0'
SCREENSHOT_SERVICE = '/NAV/screenshot_camera_nav_0'

# Camera status topics
CAMERA_0_STATUS_TOPIC = '/NAV/status_camera_nav_0'
CAMERA_1_STATUS_TOPIC = '/NAV/status_camera_nav_1'
CAMERA_2_STATUS_TOPIC = '/NAV/status_camera_nav_2'
DEPTH_STATUS_TOPIC = '/NAV/state_depth_camera_nav_0'

# Camera identifiers
CAMERA_NAV_0 = 'camera_nav_0'
CAMERA_NAV_1 = 'camera_nav_1'
CAMERA_NAV_2 = 'camera_nav_2'

# Camera display names
CAMERA_FRONT = 'Front'
CAMERA_UP1 = 'Up1'
CAMERA_UP2 = 'Up2'

# Motor lifecycle service paths
MOTOR_CHANGE_STATE_SERVICE = '/NAV_motor_cmds/change_state'
MOTOR_GET_STATE_SERVICE = '/NAV_motor_cmds/get_state'

# Hardware constants
WHEELS_RADIUS_M = 0.1325  # meters
GEAR_RATIO = 1.0 / 53.0
MIN_ROVER_SPEED_MS = 0.5  # m/s
MAX_ROVER_SPEED_MS = 2.0  # m/s

# Motor conversion constants
PI = 3.1415
ENCODER_COUNTS_PER_REV = 65536
DEGREES_PER_CIRCLE = 360
SECONDS_PER_MINUTE = 60.0

# Motor indices (for MotorStatus arrays)
MOTOR_DRIVE_FL = 0  # Front Left Drive
MOTOR_DRIVE_FR = 1  # Front Right Drive
MOTOR_DRIVE_RR = 2  # Rear Right Drive
MOTOR_DRIVE_RL = 3  # Rear Left Drive
MOTOR_STEER_FL = 4  # Front Left Steer
MOTOR_STEER_FR = 5  # Front Right Steer
MOTOR_STEER_RR = 6  # Rear Right Steer
MOTOR_STEER_RL = 7  # Rear Left Steer

# Gamepad button indices
GAMEPAD_NAV_SELECT_BUTTON = 0  # Button to confirm gamepad is for NAV
GAMEPAD_CAMERA_UP_BUTTON = 2   # Button to tilt camera up
GAMEPAD_CAMERA_DOWN_BUTTON = 3 # Button to tilt camera down

# Camera servo settings
CAMERA_SERVO_ID = 1
CAMERA_SERVO_INCREMENT_DEG = 20

# Timer periods (seconds)
FULL_STATE_PUBLISH_PERIOD = 2.0
MODE_PUBLISH_PERIOD = 2.0
MOTOR_HEALTH_CHECK_PERIOD = 2.0

# QoS settings
QOS_DEPTH = 1

# Service response codes
SERVICE_SUCCESS = 0
SERVICE_ERROR = 1

# Transition timing
TRANSITION_WAIT_TIME = 1  # seconds


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
        self.nav_names = self._load_config(NAV_CONFIG_FILE)
        self.rover_names = self._load_config(ROVER_CONFIG_FILE)
        self.el_names = self._load_config(EL_CONFIG_FILE)
        self.cs_names = self._load_config(CS_CONFIG_FILE)

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
            depth=QOS_DEPTH,
        )

        # ====================================================================
        # Gamepad Interface
        # ====================================================================
        # Subscribe to gamepad commands from CS
        self.gamepad_sub = self.create_subscription(
            Joy,
            CS_GAMEPAD_TOPIC,
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
        self.front_cam_servo_pub = self.create_publisher(
            ServoRequest, 
            self.el_names["SERVO_REQ_TOPIC"], 
            QOS_DEPTH
        )

        # ====================================================================
        # Speed Control Interface
        # ====================================================================
        # Subscribe to speed changes from CS
        self.speed_sub = self.create_subscription(
            Float32,
            self.cs_names['cs_pubsub_speed_rover'],
            self.handle_speed_change,
            QOS_DEPTH
        )

        # Forward speed commands to motor controller
        self.speed_pub = self.create_publisher(
            Float32,
            self.rover_names["rover_change_nav_speed"],
            QOS_DEPTH
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
        self.wheels_radius = WHEELS_RADIUS_M
        self.gear_ratio = GEAR_RATIO
        self.min_rover_speed = MIN_ROVER_SPEED_MS
        self.max_rover_speed = MAX_ROVER_SPEED_MS

        # ====================================================================
        # Full State Publishing
        # ====================================================================
        # Publish comprehensive state for CS (replaces ROVER aggregation)
        self.state_publisher = self.create_publisher(
            String, 
            NAV_STATE_TOPIC, 
            qos_profile=self.qos_profile
        )
        self.timer_full_state = self.create_timer(FULL_STATE_PUBLISH_PERIOD, self.publish_full_state)

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
            QOS_DEPTH
        )
        self.timer_mode = self.create_timer(MODE_PUBLISH_PERIOD, self.pub_state)

        # ====================================================================
        # Motor Lifecycle Management
        # ====================================================================
        # Timer to check motor health
        self.check_motor_health = self.create_timer(MOTOR_HEALTH_CHECK_PERIOD, self.get_state_motor_control)
        
        # Service clients for motor lifecycle control
        self.motor_change_service = self.create_client(
            ChangeState, 
            MOTOR_CHANGE_STATE_SERVICE, 
            callback_group=group2
        )
        self.motor_check_service = self.create_client(
            GetState, 
            MOTOR_GET_STATE_SERVICE
        )
        
        # ====================================================================
        # Camera Services
        # ====================================================================
        # Camera mode change service
        self.camera_mode_srv = self.create_service(
            ChangeModeCamera,
            CAMERA_MODE_SERVICE,
            self.handle_camera_mode_change,
            callback_group=group2
        )

        # RGBD depth mode service
        self.rgbd_mode_srv = self.create_service(
            SetBool,
            RGBD_MODE_SERVICE,
            self.handle_rgbd_mode,
            callback_group=group2
        )

        # Screenshot topic subscriber (CS publishes Bool messages to trigger)
        self.screenshot_sub = self.create_subscription(
            Bool,
            SCREENSHOT_TOPIC,
            self.handle_screenshot,
            QOS_DEPTH
        )

        # Camera control clients
        self.camera_nav_0 = self.create_client(SetBool, CAMERA_NAV_0_SERVICE)
        self.camera_nav_1 = self.create_client(SetBool, CAMERA_NAV_1_SERVICE)
        self.camera_nav_2 = self.create_client(SetBool, CAMERA_NAV_2_SERVICE)
        
        # RGBD and screenshot clients
        self.rgbd_client = self.create_client(SetBool, RGBD_DEPTH_SERVICE)
        self.screenshot_client = self.create_client(SetBool, SCREENSHOT_SERVICE)
        # ====================================================================
        # Camera Status Monitoring
        # ====================================================================
        # Initialize camera state tracking
        self.camera_states = {
            CAMERA_FRONT: {
                "name": CAMERA_NAV_0,
                "status": False,
                "node": False,
                "data_rate": "0",
                "depth": False
            },
            CAMERA_UP1: {
                "name": CAMERA_NAV_1, 
                "status": False,
                "node": False,
                "data_rate": "0"
            },
            CAMERA_UP2: {
                "name": CAMERA_NAV_2,
                "status": False,
                "node": False,
                "data_rate": "0"
            }
        }

        # Subscribe to camera status topics (if they exist)
        # You'll need to create these topics in your camera nodes
        self.camera_0_status_sub = self.create_subscription(
            Bool,
            CAMERA_0_STATUS_TOPIC,
            lambda msg: self.update_camera_status(CAMERA_FRONT, msg.data),
            QOS_DEPTH
        )

        self.camera_1_status_sub = self.create_subscription(
            Bool,
            CAMERA_1_STATUS_TOPIC,
            lambda msg: self.update_camera_status(CAMERA_UP1, msg.data),
            QOS_DEPTH
        )

        self.camera_2_status_sub = self.create_subscription(
            Bool,
            CAMERA_2_STATUS_TOPIC,
            lambda msg: self.update_camera_status(CAMERA_UP2, msg.data),
            QOS_DEPTH
        )

        # Subscribe to RGBD depth status
        self.depth_status_sub = self.create_subscription(
            Bool,
            DEPTH_STATUS_TOPIC,
            lambda msg: self.update_depth_status(CAMERA_FRONT, msg.data),
            QOS_DEPTH
        )


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

        # Map frontend camera names (enum values) to actual topic names
        camera_name_map = {
            CAMERA_FRONT: CAMERA_NAV_0,
            CAMERA_UP1: CAMERA_NAV_1,
            CAMERA_UP2: CAMERA_NAV_2,
            # Also support direct topic names for backwards compatibility
            CAMERA_NAV_0: CAMERA_NAV_0,
            CAMERA_NAV_1: CAMERA_NAV_1,
            CAMERA_NAV_2: CAMERA_NAV_2,
        }
        # Translate frontend name to topic name
        topic_camera_name = camera_name_map.get(camera_name, camera_name)
        
        # Map camera names to clients
        camera_clients = {
            CAMERA_NAV_0: self.camera_nav_0,
            CAMERA_NAV_1: self.camera_nav_1,
            CAMERA_NAV_2: self.camera_nav_2,
        }
        
        if topic_camera_name in camera_clients:
            req = SetBool.Request(data=activate)
            future = camera_clients[topic_camera_name].call_async(req)
            
            # Update local camera state
            if camera_name in self.camera_states:
                self.camera_states[camera_name]["status"] = activate
            
            response.error_type = SERVICE_SUCCESS
            response.error_message = f"Camera {camera_name} command sent"
        else:
            response.error_type = SERVICE_ERROR
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

    def update_camera_status(self, camera_name, is_active):
        """Update camera status when camera publishes state."""
        if camera_name in self.camera_states:
            self.camera_states[camera_name]["status"] = is_active
            self.camera_states[camera_name]["node"] = True  # Node is running if we're receiving messages

    def update_depth_status(self, camera_name, depth_active):
        """Update RGBD depth mode status."""
        if camera_name in self.camera_states:
            self.camera_states[camera_name]["depth"] = depth_active

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
        if int(msg.buttons[GAMEPAD_NAV_SELECT_BUTTON]) != 1:
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
            msg: Float32 message with safe speed value
        """
        # Validate speed range
        if msg.data <= self.min_rover_speed or msg.data >= self.max_rover_speed:
            self.get_logger().warning(f"Speed {msg.data} out of range [{self.min_rover_speed}, {self.max_rover_speed}]")
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
        increase = msg.buttons[GAMEPAD_CAMERA_UP_BUTTON]  # +1
        decrease = msg.buttons[GAMEPAD_CAMERA_DOWN_BUTTON]  # -1
        
        # Reset debounce state when buttons released
        if increase == 0 and decrease == 0:
            self.last_increment = 0
            return
        
        # Debounce: ignore if already processing
        if self.last_increment == 1 and (increase == 1 or decrease == 1):
            return
        
        # Create servo request
        angle = ServoRequest()
        angle.id = CAMERA_SERVO_ID
        angle.zero_in = False
        
        if increase == 1:
            angle.increment = CAMERA_SERVO_INCREMENT_DEG
            self.front_cam_servo_pub.publish(angle)
            self.last_increment = 1
        elif decrease == 1:
            angle.increment = -CAMERA_SERVO_INCREMENT_DEG
            self.front_cam_servo_pub.publish(angle)
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
        rps_to_ms = 2 * PI * self.wheels_radius / SECONDS_PER_MINUTE
        
        # Extract data per wheel
        # Front Left (drive=0, steer=4)
        self.motor_data['wheels']['front_left'] = {
            'current_driving': abs(msg.average_current[MOTOR_DRIVE_FL]),
            'current_steering': abs(msg.average_current[MOTOR_STEER_FL]),
            'speed': abs(round(msg.velocity[MOTOR_DRIVE_FL] * rps_to_ms * self.gear_ratio, 1)),
            'steering_angle': int(float(msg.position[MOTOR_DRIVE_FL]) / ENCODER_COUNTS_PER_REV * DEGREES_PER_CIRCLE),
            'driving_motor_state': msg.state[MOTOR_DRIVE_FL],
            'steering_motor_state': msg.state[MOTOR_STEER_FL],
            'driving_fault': msg.fault_state[MOTOR_DRIVE_FL],
            'steering_fault': msg.fault_state[MOTOR_STEER_FL]
        }

        # Front Right (drive=1, steer=5)
        self.motor_data['wheels']['front_right'] = {
            'current_driving': abs(msg.average_current[MOTOR_DRIVE_FR]),
            'current_steering': abs(msg.average_current[MOTOR_STEER_FR]),
            'speed': abs(round(msg.velocity[MOTOR_DRIVE_FR] * rps_to_ms * self.gear_ratio, 1)),
            'steering_angle': int(float(msg.position[MOTOR_DRIVE_FR]) / ENCODER_COUNTS_PER_REV * DEGREES_PER_CIRCLE),
            'driving_motor_state': msg.state[MOTOR_DRIVE_FR],
            'steering_motor_state': msg.state[MOTOR_STEER_FR],
            'driving_fault': msg.fault_state[MOTOR_DRIVE_FR],
            'steering_fault': msg.fault_state[MOTOR_STEER_FR]
        }

        # Rear right (drive=2, steer=6)
        self.motor_data['wheels']['rear_right'] = {
            'current_driving': abs(msg.average_current[MOTOR_DRIVE_RR]),
            'current_steering': abs(msg.average_current[MOTOR_STEER_RR]),
            'speed': abs(round(msg.velocity[MOTOR_DRIVE_RR] * rps_to_ms * self.gear_ratio, 1)),
            'steering_angle': int(float(msg.position[MOTOR_DRIVE_RR]) / ENCODER_COUNTS_PER_REV * DEGREES_PER_CIRCLE),
            'driving_motor_state': msg.state[MOTOR_DRIVE_RR],
            'steering_motor_state': msg.state[MOTOR_STEER_RR],
            'driving_fault': msg.fault_state[MOTOR_DRIVE_RR],
            'steering_fault': msg.fault_state[MOTOR_STEER_RR]
        }

        # Rear left (drive=3, steer=7)
        self.motor_data['wheels']['rear_left'] = {
            'current_driving': abs(msg.average_current[MOTOR_DRIVE_RL]),
            'current_steering': abs(msg.average_current[MOTOR_STEER_RL]),
            'speed': abs(round(msg.velocity[MOTOR_DRIVE_RL] * rps_to_ms * self.gear_ratio, 1)),
            'steering_angle': int(float(msg.position[MOTOR_DRIVE_RL]) / ENCODER_COUNTS_PER_REV * DEGREES_PER_CIRCLE),
            'driving_motor_state': msg.state[MOTOR_DRIVE_RL],
            'steering_motor_state': msg.state[MOTOR_STEER_RL],
            'driving_fault': msg.fault_state[MOTOR_DRIVE_RL],
            'steering_fault': msg.fault_state[MOTOR_STEER_RL]
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

        # Update camera states
        self.nav_full_state["cameras"] = self.camera_states
        
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
            response.error_type = SERVICE_ERROR
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
                time.sleep(TRANSITION_WAIT_TIME)

            return response 
        
        # Case 2: Change between manual modes (no lifecycle change needed)
        if self.mode != "Off" and MODE[mode] != 'Off':
            self.change_mode(mode, SERVICE_SUCCESS, response, "no errors")
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
                time.sleep(TRANSITION_WAIT_TIME)

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
            self.change_mode(mode, SERVICE_SUCCESS, response, message_response)
            self.get_logger().info(message_response)
        else:
            self.change_mode(MODE_TO_CS[self.mode], SERVICE_ERROR, response, "NAV lifecycle change state failed")
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