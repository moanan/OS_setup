#!/bin/sh
# Run this to setup ROS2 (humble) and Moveit2 development environment on Ubuntu 22.04


if [ `whoami` == "root" ]; then 
  echo "Do not run this as root!"
  exit 1
fi

# https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html#id7
echo "********************************"
echo "********** Set locale **********"
echo "********************************"

sudo apt update && sudo apt -y install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "********************************"
echo "********* Setup source *********"
echo "********************************"

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "********************************"
echo "******* Install ROS Humble *******"
echo "********************************"
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

echo "********************************"
echo "******* Install MoveIt 2 *******"
echo "********************************"
# https://moveit.ros.org/install-moveit2/source/
source /opt/ros/humble/setup.zsh
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
sudo apt install python3-vcstool
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src
git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
vcs import < moveit2_tutorials/moveit2_tutorials.repos
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

echo "********************************"
echo "******* Build MoveIt 2 *******"
echo "********************************"
cd ~/ws_moveit2
colcon build --mixin release --parallel-workers 1

echo "********************************"
echo "******** All done baby! ********"
echo "********************************"