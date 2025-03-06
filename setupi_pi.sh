#!/bin/bash

set -e  # Exit immediately if a command fails

echo "Starting Raspberry Pi Setup..."

# Ensure the script is run as root
if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root. Please run it with sudo."
    exit 1
fi

# Update and upgrade system
echo "Updating system..."
apt update && apt upgrade -y

# Install necessary packages
echo "Installing required packages..."
apt install -y git vim python3-pip docker.io curl locales software-properties-common

# Enable SSH
echo "Enabling SSH..."
systemctl enable ssh
systemctl start ssh

# Configure system locale for ROS 2
echo "Setting up locale..."
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
echo "Adding ROS 2 repository..."
add-apt-repository universe -y
apt update
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
apt update && apt upgrade -y

# Install ROS 2
echo "Installing ROS 2..."
apt install -y ros-humble-desktop ros-dev-tools

# Source ROS 2 setup files automatically
echo "Sourcing ROS 2 setup..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
source ~/.bashrc

# Create ROS 2 workspace
ROS2_WS=~/ros2_ws
mkdir -p $ROS2_WS/src
cd $ROS2_WS/src

# Create motor_control package
#echo "Creating ROS 2 package..."
#ros2 pkg create --build-type ament_python motor_control

# Build ROS 2 workspace
#cd $ROS2_WS
#colcon build

# Source the new workspace setup
#echo "source $ROS2_WS/install/setup.bash" >> ~/.bashrc
#source install/setup.bash

echo "Raspberry Pi setup complete!"
