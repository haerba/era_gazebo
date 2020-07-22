#!/bin/bash
source devel/setup.bash
cd ~/catkin_ws/src/dsrc/gr-ros_interface/
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
cd ~/catkin_ws/src/dsrc/gr-foo/
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
cd ~/catkin_ws/src/dsrc/gr-ieee802-11/
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
cd ~/catkin_ws/src/dsrc/gr-ieee802-11/examples
grcc ./wifi_transceiver.grc -d .
cd ~/catkin_ws
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
