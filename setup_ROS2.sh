#!/bin/sh
# Run this to setup ROS2 (foxy) development environment on Ubuntu 20.04


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

sudo apt update && sudo apt -y install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "********************************"
echo "******* Install packages *******"
echo "********************************"
sudo apt update
sudo apt -y install ros-foxy-desktop
sudo apt -y install ros-foxy-ros-base
source /opt/ros/foxy/setup.bash         # source the setup script
sudo apt install -y python3-argcomplete # Install argcomplete (optional)

echo "source /opt/ros/foxy/setup.zsh" >> ~/.zshrc

sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

echo "********************************"
echo "******* Install MoveIt 2 *******"
echo "********************************"
# https://moveit.ros.org/install-moveit2/source/
source /opt/ros/foxy/setup.zsh
rosdep update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  clang-format-10 && \
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

export COLCON_WS=~/ws_moveit2/
mkdir -p $COLCON_WS/src
cd $COLCON_WS/src

wget https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos
vcs import < moveit2.repos
git clone https://github.com/ros-planning/moveit2.git
rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y

cd $COLCON_WS
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release


echo "********************************"
echo "****** Install Odrivetool ******"
echo "********************************"

cd
sudo pip3 install --upgrade odrive

echo "********************************"
echo "******** All done baby! ********"
echo "********************************"