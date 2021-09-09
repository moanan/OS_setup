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
sudo apt -y install exfat-fuse exfat-utils

echo "*** System settings ***"
git config --global user.name "$An Mo(E320H)"
git config --global user.email "$moan1992@gmail.com"


echo "*** Install Sublime ***"
# https://www.sublimemerge.com/docs/linux_repositories#apt
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
sudo apt-get install apt-transport-https
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-merge
sudo apt-get install sublime-text


echo "*** Install Chrome ***"
# chrome
wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'
sudo apt update
sudo apt install google-chrome-stable


echo "*** Install VS Code ***"
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt install code


echo "*** Install zsh ***"
# https://zellwk.com/blog/bash-zsh-fish/
sudo apt -y install zsh

echo "*** Install oh-my-zsh  ***"
sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

chsh -s $(which zsh) # it needs reboot to take effect

echo "*** Setup fish-like zsh ***"
# sudo apt -y install zsh-syntax-highlighting
# sudo apt -y install zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting


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