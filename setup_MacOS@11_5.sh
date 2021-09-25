#!/bin/bash
## A script to set up a new mac with standard brew stuff + lots of apps
## The above line tells shell to use bash
## do this to make the script executable for the user
# chmod u+x setup_MacOS.sh

# helpers
function echo_ok { echo -e '\033[1;32m'"$1"'\033[0m'; }
function echo_warn { echo -e '\033[1;33m'"$1"'\033[0m'; }
echo_ok "Install starting. You may be asked for your password (for sudo)."
cd ~

# install command-line tools from xcode (without installing xcode)
# xcode-select --install

echo "***********************************************************************"
echo "***************************** OS settings *****************************"
echo "***********************************************************************"

# Set up git settings
# On Macs, Finder creates in every folder a .DS_Store file. This can get annoying if you mistakenly add it to your git local repository with a `git add --all`, so let's add this to .gitignore_global
echo ".DS_Store" >> ~/.gitignore_global
echo "._.DS_Store" >> ~/.gitignore_global
echo "**/.DS_Store" >> ~/.gitignore_global
echo "**/._.DS_Store" >> ~/.gitignore_global
git config --global core.excludesfile ~/.gitignore_global
# Use diff3 format, for much easier conflict merging
# git config --global merge.conflictstyle diff3
# Use keychain to manage your passwords ### This should be done automatically by homebrew
# git config --global credential.helper osxkeychain
# use Sublime as standard git editor
# git config --global core.editor subl
# git config --global user.name and user.email

# read -p "Do you want to change settings too?" -n 1 -r
# echo
# if [[ $REPLY =~ ^[Yy]$ ]]
# then
  echo_warn "Disabling Photos from auto-starting when plugging in a camera."
  defaults -currentHost write com.apple.ImageCapture disableHotPlug -bool YES

  echo_warn "Expanding save and print dialogs by default"
  defaults write NSGlobalDomain PMPrintingExpandedStateForPrint -bool true
  defaults write NSGlobalDomain PMPrintingExpandedStateForPrint2 -bool true
  defaults write NSGlobalDomain NSNavPanelExpandedStateForSaveMode -bool true
  defaults write NSGlobalDomain NSNavPanelExpandedStateForSaveMode2 -bool true

  echo_warn "Always showing scroll bars"
  defaults write NSGlobalDomain AppleShowScrollBars -string "Always"

 # echo_warn "Using oldschool (unnatural) scrolling direction (for trackpad)"
 # defaults write NSGlobalDomain com.apple.swipescrolldirection -bool false

  echo_warn "Turning off auto-spell globally"
  defaults write NSGlobalDomain NSAutomaticSpellingCorrectionEnabled -bool false

  echo_warn "Cleaning up and customizing the dock."
  defaults delete com.apple.dock persistent-apps # unpin apps
  defaults delete com.apple.dock persistent-others # unpin more apps
  # defaults write com.apple.dock pinning -string start # place at left
  defaults write com.apple.dock mineffect scale # just scale minimizing windows instead of fancy minimize
  killall Dock # restart dock

  # echo_warn "Tell Chrome to use system dialog for printing"
  # defaults write com.google.Chrome DisablePrintPreview -boolean true
# fi

echo_ok "You will have to startup some apps manually once (like spectacle)"
echo_ok "You have to manually set which apps automatically start on startup, under 'preferences > users & groups > login items'"
echo_ok "You should set up your `git config --global user.name and user.email`"


echo "***********************************************************************"
echo "******************** preparation before installing ********************"
echo "***********************************************************************"

# Setup oh-my-zsh
sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" # shell will change after installation
chsh -s $(which zsh) # it needs reboot to take effect
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
echo_warn "add plugins to .zshrc manually: plugins=(    )"

# Setup homebrew
if hash brew &> /dev/null; then
  echo_ok "Homebrew already installed"
  brew update
else
  echo_warn "Installing homebrew..."
  /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
fi


echo "***********************************************************************"
echo "************************* uncomment below to brew *********************"
echo "***********************************************************************"

# echo_warn "Installing standard homebrew libraries"

# brew install git wget youtube-dl htop
# brew install ffmpeg imagemagick ghostscript python
# brew install pdfsandwich

# brew install --cask iterm2 # best terminal
# brew install --cask sublime-text
# brew install --cask sublime-merge
# brew install --cask visual-studio-code
# brew install --cask xquartz
# brew install --cask vlc
# brew install --cask mpv
# brew install --cask the-unarchiver
# brew install --cask mactex # tex, latex. This takes a long time, do separately
# brew install --cask spectacle
# brew install --cask inkscape
# brew install --cask gimp
# brew install --cask adobe-acrobat-reader
# brew install --cask flux
# brew install --cask firefox
# brew install --cask thunderbird
# brew install --cask google-chrome
# brew install --cask zoom
# brew install --cask webex
# brew install --cask wechat
# brew install --cask mattermost
# brew install --cask karabiner-elements
# brew install --cask rectangle
# brew install --cask microsoft-office
# brew install --cask mpv
# brew install --cask obs


echo "***********************************************************************"
echo "************************** manual installation ************************"
echo "***********************************************************************"

# chrome-remote desktop
# anaconda
# cisco anyconnect
# # matlab
# # endnote


# manual settings

# # matlab 
# https://github.com/scottclowe/matlab-schemer
# https://github.com/altmany/export_fig
# # sublime-text
# https://latextools.readthedocs.io/en/latest/install/
# # thunderbird
# https://support.mozilla.org/en-US/kb/moving-thunderbird-data-to-a-new-computer
# # endnote
# https://unimelb.libguides.com/c.php?g=403235&p=2744660
# # davinci-resolve

# # common drive access

# github and gitlab add ssh keys
# transfer files







