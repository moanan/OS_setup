#!/bin/sh
# Run this after fresh installation of Ubuntu 22.04.3
# chmod 755 setup_E320.sh


if [ `whoami` == "root" ]; then 
  echo "Do not run this as root!"
  exit 1
fi

echo "*** Update the OS ***"
sudo apt -y update
sudo apt -y upgrade

sudo apt -y install git
sudo apt -y install wget gpg
sudo apt -y install curl
sudo apt -y install htop
sudo apt -y install exfat-fuse exfat-utils
sudo apt -y install tree
sudo apt -y install vim
sudo apt -y install openssh-server
sudo apt -y install psensor
sudo apt -y install psensor-server

echo "*** System settings ***"
git config --global user.name "amo"
git config --global user.email "moan1992@gmail.com"

echo "*** Install Chrome ***"
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo dpkg -i google-chrome-stable_current_amd64.deb

echo "*** Install VS Code ***"
sudo apt install software-properties-common apt-transport-https wget -y
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt install code

echo "*** Install oh-my-zsh ***"
# https://zellwk.com/blog/bash-zsh-fish/
sudo apt -y install zsh
sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
chsh -s $(which zsh) # it needs reboot to take effect

echo "*** Setup fish-like zsh ***"
# sudo apt -y install zsh-syntax-highlighting
# sudo apt -y install zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
# git clone https://github.com/zsh-users/zsh-autosuggestions ~/.zsh/zsh-autosuggestions
# git clone https://github.com/zsh-users/zsh-autosuggestions ~/.zsh/zsh-autosuggestions
# source ~/.zsh/zsh-autosuggestions/zsh-autosuggestions.zsh
# source ~/.zsh/zsh-autosuggestions/zsh-autosuggestions.zsh

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