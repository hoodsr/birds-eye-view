#!/bin/bash

#This file will install turtle bot components on a fesh install of Ubuntu 14.04
# This file DOES NOT install pangolin or ORB SLAM. Once i figure out how to install it, i wil add it


#########################
#Check version of ubuntu#
#########################
version=$(lsb_release -r)
if [ "Release:	14.04" = "$version" ]; then
	echo Running Ubuntu 14.04, good version
else
	echo Bad Version
	exit
fi

#######################
#Installing ROS Indigo#
#######################

#install sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#setup keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

#update the stuff
sudo apt-get update

#install ros
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update

#add ros to bashrc
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

#get rosinstall
sudo apt-get install python-rosinstall




###############
#Install lidar#
###############

#decide which lidar type
echo "Are you using a lidar connected by ethernet or usb? Check if there is an addaptor from ethernet to usb. [ethernet/usb]"

read lidar

while [[ ( "$lidar" != "ethernet" ) && ( "$lidar" != "usb" ) ]]; do
	echo "Please enter either ethernet or usb: "
	read lidar
done

if [ "$lidar" = "ethernet" ]; then
	sudo apt-get install ros-indigo-urg-node #install ethernet version
else
	sudo apt-get install ros-indigo-hokuyo-node #install usb version
fi

#######################################################################
#                            Install OpenCV                           #
#Borrowed from https://gist.github.com/robinkraft/4d1807fb8f9c246b2d21#
#######################################################################
 cd ~/
# install dependencies
sudo apt-get update
sudo apt-get install -y build-essential
sudo apt-get install -y cmake
sudo apt-get install -y libgtk2.0-dev
sudo apt-get install -y pkg-config
sudo apt-get install -y python-numpy python-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libjpeg-dev libpng-dev libtiff-dev libjasper-dev

sudo apt-get -qq install libopencv-dev build-essential checkinstall cmake pkg-config yasm libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python-dev python-numpy libtbb-dev libqt4-dev libgtk2.0-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils

# download opencv-2.4.10
wget http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.4.10/opencv-2.4.10.zip
unzip opencv-2.4.10.zip
cd opencv-2.4.10

# compile and install
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON .
make -j2 # 2 cores
sudo make install

####################################################
#skipped ORBSLAM instqall due to it being difficult#
####################################################


#AR Track ALvar
sudo apt-get install ros-indigo-ar-track-alvar

#install joystick teleop
sudo apt-get install ros-indigo-joy

#install a few last prerequisites for catkkin_make
sudo sudo apt-get install -y --force-yes libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libavresample-dev

mkdir -p ~/turtle_ws/src
cd ~/turtle_ws/
sudo rm -r build devel
cd src
git clone "https://github.com/hoodsr/birds-eye-view.git"
cd ..
catkin_make


################################################33
#Lets get some of that turtlebot
##################################

sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs

sudo apt-get install python-rosdep python-wstool ros-indigo-ros

rosdep update

mkdir ~/rocon
cd ~/rocon
wstool init -j5 src https://raw.github.com/robotics-in-concert/rocon/release/indigo/rocon.rosinstall
source /opt/ros/indigo/setup.bash
rosdep install --from-paths src -i -y
catkin_make

mkdir ~/kobuki
cd ~/kobuki
wstool init src -j5 https://raw.github.com/yujinrobot/yujin_tools/master/rosinstalls/indigo/kobuki.rosinstall
source ~/rocon/devel/setup.bash
rosdep install --from-paths src -i -y
catkin_make

mkdir ~/turtlebot
cd ~/turtlebot
wstool init src -j5 https://raw.github.com/yujinrobot/yujin_tools/master/rosinstalls/indigo/turtlebot.rosinstall
source ~/kobuki/devel/setup.bash
rosdep install --from-paths src -i -y
catkin_make


#############################
#Lets finish up and explain a thing
############################
echo "You need to run ls /dev/input to find the the jsX number. then run 'sudo chmod a+r /dev/input/jsX' and 'jstest /dev/input/jsX'"
echo "You also need to plug in the turtle bot and execute 'sudo chmod a+rw /dev/ttyACM0'"

