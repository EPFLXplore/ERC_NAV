sudo tee /usr/local/bin/imu-usb-setup.sh > /dev/null << 'EOF'
#!/bin/bash
# 1) configure the USB-IMU link
ip addr flush dev enx0a76f8b03d57
ip addr add 169.254.56.222/24 dev enx0a76f8b03d57 noprefixroute
ip link set dev enx0a76f8b03d57 up

# 2) host-route to the IMU
ip route add 169.254.56.221/32 \
    dev enx0a76f8b03d57 \
    src 169.254.56.222 \
    metric 50

# 3) disable forwarding
sysctl -q -w net.ipv4.ip_forward=0


EOF

sudo chmod +x /usr/local/bin/imu-usb-setup.sh
