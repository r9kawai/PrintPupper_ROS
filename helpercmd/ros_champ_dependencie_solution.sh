#!/bin/sh
echo "Try install CHAMP dependencie packages..."
set -x
sudo apt update -y
rosdep update
sudo apt install -y ros-melodic-ros-controllers
sudo apt install -y ros-melodic-pointcloud-to-laserscan
sudo apt install -y ros-melodic-velodyne-gazebo-plugins
sudo apt install -y ros-melodic-ecl-threads
sudo apt install -y ros-melodic-moveit
sudo apt install -y ros-melodic-amcl
sudo apt install -y ros-melodic-hector-sensors-description
sudo apt install -y ros-melodic-base-local-planner
sudo apt install -y ros-melodic-hector-gazebo-plugins
sudo apt install -y ros-melodic-joint-state-publisher-gui
sudo apt install -y ros-melodic-joy
sudo apt install -y ros-melodic-dwa-local-planner
sudo apt install -y ros-melodic-yocs-velocity-smoother
sudo apt install -y ros-melodic-navfn ros-melodic-rosserial
sudo apt install -y ros-melodic-global-planner
sudo apt install -y ros-melodic-robot-localization
sudo apt install -y ros-melodic-move-base ros-melodic-map-server
sudo apt install -y ros-melodic-octomap-server
sudo apt install -y ros-melodic-hector-mapping
sudo apt install -y ros-melodic-gmapping
sudo apt install -y ros-melodic-gazebo-plugins
sudo apt install -y ros-melodic-moveit-ros-planning-interface
sudo apt install -y ros-melodic-robot-state-publisher
sudo apt install -y ros-melodic-ekf-localization-node
sudo apt install -y ros-melodic-gazebo-ros-control
sudo apt install -y ros-melodic-gazebo-ros-pkgs
sudo apt install -y python3-jinja2
sudo apt install -y python-jinja2
set +x
echo "...done"
