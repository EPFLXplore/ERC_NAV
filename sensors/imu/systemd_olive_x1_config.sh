sudo tee /etc/systemd/system/imu-usb-setup.service > /dev/null << 'EOF'
[Unit]
Description=Configure Olive IMU USB-Ethernet link route
# wait until the network stack is “online”...
Wants=network-online.target
After=network-online.target
# …and also wait for the USB-eth device itself
Requires=sys-subsystem-net-devices-enx0a76f8b03d57.device
After=sys-subsystem-net-devices-enx0a76f8b03d57.device

[Service]
Type=oneshot
ExecStart=/usr/local/bin/imu-usb-setup.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
