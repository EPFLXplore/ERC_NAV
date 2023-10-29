# NAV_workspace_2023

Warning: the code is not finish and have compilation error


## Main Launch commands (need to be updated)
- ros2 launch nav.launch.xml
- ros2 launch src/wheels_control/wheels_control.launch.xml 


## Other Launch commands
- ros2 launch ros2_ouster ouster.launch.py
- ros2 launch astra_ros2_description urdf_launch.py
- ros2 launch scanmatcher lio.launch.py
- ros2 launch nav2_bringup navigation_launch.py
- ros2 launch nav2_bringup rviz_launch.py


## Manual Navigation
- colcon build --packages-select custom_msg  wheels_cmds
- ros2 launch wheels_cmds wheels_cmds_launch.xml fake_cs_gamepad:=true
- ros2 launch wheels_cmds wheels_cmds_launch.xml 
- ros2 run wheels_cmds fake_cs_gamepad.py 
- ros2 run wheels_cmds NAV_displacement_cmds
- ros2 run wheels_cmds NAV_motor_cmds

## Autonomous Navigation

### Set/reset lidar connection

If lidar is connected to eth0

    sudo ip addr flush dev eth0
    sudo ip link set eth0 down
    sudo ip addr show dev eth0 # need to see ...state DOWN...
    sudo ip addr add 10.5.5.1/24 dev eth0
    sudo ip link set eth0 up
    sudo ip addr show eth0
    sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i eth0 --bind-dynamic

If lidar is connected to eth1

    sudo ip addr flush dev eth1
    sudo ip link set eth1 down
    sudo ip addr show dev eth1 # need to see ...state DOWN...
    sudo ip addr add 10.5.5.1/24 dev eth1
    sudo ip link set eth1 up
    sudo ip addr show eth1
    sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i eth1 --bind-dynamic

### Launch lidar and Nav2 nodes

The following launch files run the nodes that receive the data from the sensors (lidar + depth_camera) and create the costmap. Furthermore, they also run the nodes related to autonomous navigation (Nav planner, Nav controller, etc).

    ros2 launch ros2_ouster ouster.launch.py
    ros2 launch astra_ros2_description urdf_launch.py
    ros2 launch scanmatcher lio.launch.py
    ros2 launch nav2_bringup navigation_launch.py
    ros2 launch nav2_bringup rviz_launch.py

### TODO

Using a custom package we can publish a waypoint that the Nav planner and Nav controller will receive. In turn, they will publish on the `/cmd_vel` topic (a Twist message). We then subscribe to this topic and convert the message to a /Nav/displacement message that the Motor node then uses to control the wheels.

Convert the Control Station gamepad so that it publishes cmd_vel topics



## Lidar stuff
### QUICK CHECK LIDAR WORKED
pip install ouster-sdk
simple-viz --sensor os-122140001125.local
http://os-122140001125.local











######## LE DEBUT DE TOUTES CHOSES 2



### Usefull lidar commands
nmcli connection show
avahi-browse -arlt
ping 10.5.5.87
ping 10.5.5.1
pinglidar(ping os-122140001125.local)
nc os-122140001125.local


### LINUX usefull commands
usefull linux commands
sudo dmesg -w
usb-devices
lsusb


### ROS2
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select
ros2 daemon stop  # to stop ros2 if strange behavior
ros2 daemon start # to start ros2



### Launch Navigation
ros2 launch ros2_ouster ouster.launch.py
ros2 launch astra_ros2_description urdf_launch.py
ros2 launch scanmatcher lio.launch.py
ros2 launch nav2_bringup navigation_launch.py
ros2 launch nav2_bringup rviz_launch.py

ros2 run cs_interface cs_interface_receive_goal
ros2 run cs_interface cs_interface_send_goal --ros-args -p:=2.5 y:=2.5 w:=6.2
ros2 launch src/wheels_control/src/wheels_cmds/launch/wheels_control_launch.xml



### lidar ipv4
sudo ip addr add 10.5.5.1/24 dev mtu 1500















