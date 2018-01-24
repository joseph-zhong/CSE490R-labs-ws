#!/usr/bin/env bash
#
# setup.sh
# ---
#   This helps setup the development environment for the labs.
#

echo ""
echo " -- Installing vesc..."
echo ""
sudo apt install ros-kinetic-serial

echo ""
echo " -- Installing map-server..."
echo ""
sudo apt install ros-kinetic-map-server

echo ""
echo " -- Building packages in the standard catkin_ws"
echo ""
if [ ! -d "~/catkin_ws" ]; then
  mkdir -p ~/catkin_ws
  if [ ! -d "~/catkin_ws/racecar_base" ]; then
    git clone https://gitlab.cs.washington.edu/cse490r_18wi/racecar_base.git ~/catkin_ws
  fi
fi

echo ""
echo " -- Updating packages from racecar_base..."
echo ""
cd ~/catkin_ws/racecar_base/
git pull
cp -r ~/catkin_ws/racecar_base/* ~/catkin_ws/src/
cd ~/catkin_ws/; catkin_make

echo ""
echo " -- Done updating packages from racecar_base..."
echo ""

echo ""
echo " -- Installing range_libc"
echo "    See https://github.com/kctess5/range_libc#python-wrappers "
echo ""
pip install --user Cython

cd
git clone https://github.com/kctess5/range_libc
cd range_libc/pywrapper
sudo python setup.py install
python test.py

echo ""
echo " -- Done installing range_libc"
echo ""

echo ""
echo "Make sure to run the following, and add them to your ~/.bashrc"
echo
echo "
   export ROS_MASTER_URI=http://10.42.0.1:11311
   export ROS_PACKAGE_PATH

   source /opt/ros/kinetic/setup.bash
   source ~/catkin_ws/devel/setup.bash
"
