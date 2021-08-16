#!/bin/sh
# Run this after fresh installation of Ubuntu 20.04.2.0
# chmod 755 setup_E320.sh


if [ `whoami` == "root" ]; then 
  echo "Do not run this as root!"
  exit 1
fi


echo "*** Update the OS ***"
sudo apt -y update
sudo apt -y upgrade


sudo apt -y install git
sudo apt -y install curl
sudo apt -y install htop

echo "*** System settings ***"
git config --global user.name "$An Mo(E320H)"
git config --global user.email "$moan1992@gmail.com"

enable SPI, SSH



echo "*** Install VS Code ***"
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt install code

echo "*** bcm2835 ***"
# https://www.airspayce.com/mikem/bcm2835/
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.68.tar.gz
tar zxvf bcm2835-1.68.tar.gz
rm bcm2835-1.68.tar.gz
cd bcm2835-1.68
./configure
make
sudo make check
sudo make install


echo "*** rt_preempt ***"

echo "*** Over clock ***"
# https://magpi.raspberrypi.org/articles/how-to-overclock-raspberry-pi-4
# https://qengineering.eu/overclocking-the-raspberry-pi-4.html
over_voltage=2
arm_freq=1750
force_turbo=1
# watch -n 1 cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
# cat /sys/class/thermal/thermal_zone0/temp


echo "***********************************************************************"
echo "*********** All done! Please do the following task manually ***********"
echo "***********************************************************************"
echo "Manually add the following lines to ~/.zshrc to enable the plugins"
echo "plugins=(zsh-autosuggestions"
echo "           zsh-syntax-highlighting)"
echo "-------------------------------------------------------------------"
echo "Add backup ssh key pairs: https://superuser.com/questions/332510/how-to-transfer-my-ssh-keys-to-another-machine"
echo "-------------------------------------------------------------------"
echo "Reboot the system to make installations work."
echo "-------------------------------------------------------------------"