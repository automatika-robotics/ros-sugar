#!/bin/bash

# Your commands here
echo "Creating deb files"
apt-get update
apt-get -y install python3-pip python3-bloom
apt-get -y install fakeroot dpkg-dev debhelper

# location specified in container build script
cd /project
echo $PWD
lsb_release -sc

bloom-generate rosdebian --os-name ubuntu --os-version $(lsb_release -sc) --ros-distro $ROS_DISTRO

source /opt/ros/$ROS_DISTRO/setup.bash

fakeroot debian/rules binary


