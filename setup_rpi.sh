#!/bin/bash

# Kuzgun 2025 Servo Control Setup Script for Raspberry Pi
# Run this script to install dependencies and setup the servo control system

echo "=================================================="
echo "Kuzgun 2025 Servo Control Setup"
echo "=================================================="

# Update system
echo "Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install Python pip if not installed
echo "Installing Python pip..."
sudo apt install python3-pip -y

# Install required Python packages
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Install additional system dependencies for OpenCV
echo "Installing OpenCV system dependencies..."
sudo apt install python3-opencv -y
sudo apt install libcamera-apps -y

# Enable GPIO and Camera (if not already enabled)
echo "Configuring Raspberry Pi..."

# Check if GPIO is enabled
if ! grep -q "dtparam=spi=on" /boot/config.txt; then
    echo "Enabling SPI..."
    echo "dtparam=spi=on" | sudo tee -a /boot/config.txt
fi

if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "Enabling I2C..."
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
fi

# Enable camera
if ! grep -q "camera_auto_detect=1" /boot/config.txt; then
    echo "Enabling camera..."
    echo "camera_auto_detect=1" | sudo tee -a /boot/config.txt
fi

# Create systemd service for auto-start (optional)
read -p "Do you want to create a systemd service for auto-start? (y/N): " create_service

if [[ $create_service =~ ^[Yy]$ ]]; then
    cat > kuzgun.service << EOF
[Unit]
Description=Kuzgun 2025 Payload Drop System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=$(pwd)
ExecStart=/usr/bin/python3 $(pwd)/main.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    sudo mv kuzgun.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable kuzgun.service
    echo "Systemd service created. Use 'sudo systemctl start kuzgun' to start."
fi

# Test servo connections
echo "=================================================="
echo "Setup complete!"
echo "=================================================="
echo "Next steps:"
echo "1. Connect your servos to GPIO pins 18 and 19"
echo "2. Connect servo power (usually 5V)"
echo "3. Test the setup with: python3 main.py"
echo "4. Use 'T' key during operation to test servos"
echo "5. Use 'R' key to reset servos to closed position"
echo "6. Use 'Q' key to quit"
echo ""
echo "Servo connections:"
echo "- Servo 1 (Payload 1): GPIO 18"
echo "- Servo 2 (Payload 2): GPIO 19"
echo "- Power: 5V (from Pi or external)"
echo "- Ground: Common ground"
echo ""
echo "Reboot recommended to apply all changes."