#!/usr/bin/env bash
#
# setup.sh
# ---
#   This helps setup the development environment for the labs.
#

echo "Installing vesc dependencies..."
sudo apt install ros-kinetic-serial

echo "Building packages in the standard catkin_ws"
if [ ! -d "~/catkin_ws" ]; then
  mkdir -p ~/catkin_ws
fi

git clone https://gitlab.cs.washington.edu/cse490r_18wi/racecar_base.git ~/catkin_ws
cp -r ~/catkin_ws/racecar_base/* ~/catkin_ws/src/
