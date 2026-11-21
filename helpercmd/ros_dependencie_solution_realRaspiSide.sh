#!/bin/sh
echo "Try install ROS dependencie packages..."
set -x
sudo apt update -y
rosdep update
sudo apt install -y ros-melodic-tf2
sudo apt install -y ros-melodic-joy
sudo apt install -y ros-melodic-hector-sensors-description
sudo apt install -y ros-melodic-urdf
sudo apt install -y ros-melodic-joint-state-publisher
sudo apt install -y ros-melodic-robot-state-publisher
sudo apt install -y ros-melodic-ecl-threads
sudo apt install -y ros-melodic-joint-state-publisher-gui
sudo apt install -y ros-melodic-yocs-velocity-smoother
sudo apt install -y ros-melodic-rosserial
sudo apt install -y ros-melodic-robot-localization
set +x
echo "...done"

