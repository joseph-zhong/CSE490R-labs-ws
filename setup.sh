#!/usr/bin/env bash
#
# setup.sh
# ---
#   This helps setup the development environment for the labs.
#

echo "Installing vesc..."
sudo apt install ros-kinetic-serial

echo "Installing map-server..."
sudo apt install ros-kinetic-map-server

echo "Building packages in the standard catkin_ws"
if [ ! -d "~/catkin_ws" ]; then
  mkdir -p ~/catkin_ws
  if [ ! -d "~/catkin_ws/racecar_base" ]; then
    git clone https://gitlab.cs.washington.edu/cse490r_18wi/racecar_base.git ~/catkin_ws
  fi
fi

echo "Updating packages from racecar_base..."
cd ~/catkin_ws/racecar_base/
git pull
cp -r ~/catkin_ws/racecar_base/* ~/catkin_ws/src/
cd ~/catkin_ws/src/; catkin_make

echo "Make sure to run the following, and add them to your ~/.bashrc"
echo
echo "
   export ROS_MASTER_URI=http://10.42.0.1:11311
   export ROS_PACKAGE_PATH

   source /opt/ros/kinetic/setup.bash
   source ~/catkin_ws/devel/setup.bash
"
