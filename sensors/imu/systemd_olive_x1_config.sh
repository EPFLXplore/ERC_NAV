sudo tee /etc/systemd/system/imu-usb-setup.service > /dev/null << 'EOF'
[Unit]
Description=Configure Olive IMU USB-Ethernet link route
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/imu-usb-setup.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
