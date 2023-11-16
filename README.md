# NAV_workspace_2023

Warning: the code is not finish and have compilation error



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







## Lidar stuff
### QUICK CHECK LIDAR WORKED
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




### lidar ipv4
sudo ip addr add 10.5.5.1/24 dev mtu 1500















