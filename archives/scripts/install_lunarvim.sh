#!/bin/bash
source utils.sh
echo '================================'
echo '          apt update            '
echo '================================'
ensure_sudo && sudo apt update && sudo apt install -y curl wget git unzip

echo '================================'
echo '       Install neovim           '
echo '================================'
# Official neovim repo: https://github.com/neovim/neovim/releases
#NVIM_VERSION="v0.8.3"
# wget https://github.com/neovim/neovim/releases/download/"$NVIM_VERSION"/nvim-linux64.deb

# NVIM_VERSION="nightly"
# https://github.com/neovim/neovim/releases/download/nightly/nvim-linux64.deb

#sudo apt install -y ./nvim-linux64.deb || exit_code=$?
# if [[ $exit_code -ne 0 ]]; then
#   echo "install neovim failed."
#   exit
# else
#   echo "install neovim done."
# fi

NVIM_VERSION="v0.9.1"
rm $HOME/nvim-linux64.tar.gz > /dev/null 2>&1
rm -rf $HOME/nvim-linux64 > /dev/null 2>&1
cd "$HOME" && wget "https://github.com/neovim/neovim/releases/download/${NVIM_VERSION}/nvim-linux64.tar.gz"
tar xvf $HOME/nvim-linux64.tar.gz
sudo rm /usr/bin/nvim > /dev/null 2>&1
sudo ln -s $HOME/nvim-linux64/bin/nvim /usr/bin/nvim

echo '================================'
echo '        Install python          '
echo '================================'
sudo apt install -y python3 python3-pip python3-venv

echo '================================'
echo '         Install cargo          '
echo '================================'
curl https://sh.rustup.rs -sSf | sh -s -- -y
. "$HOME/.cargo/env"

echo '================================'
echo '          Install npm           '
echo '================================'
# === ubuntu 20.04 ===
#curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.2/install.sh | bash
# === ubuntu 18.04 ===
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.33.11/install.sh | bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" # this loads nvm
[ -s "$NVM_DIR/bash_completion" ] && . "$NVM_DIR/bash_completion" # this loads nvm bash_completion
nvm ls-remote

# === ubuntu 20.04 ===
#nvm install v18.12.1
# === ubuntu 18.04 ===
nvm install v16.19.0

npm install -g npm

append_bashrc 'export NVM_DIR="$HOME/.nvm"'
append_bashrc '[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" # this loads nvm'
append_bashrc '[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"'

echo '==============================================='
echo '  Install LunarVim and follow the instructions '
echo '==============================================='
# Release version (neovim 0.8.0)
# LV_BRANCH='release-1.2/neovim-0.8' bash <(curl -s https://raw.githubusercontent.com/lunarvim/lunarvim/fc6873809934917b470bff1b072171879899a36b/utils/installer/install.sh)
#
# Nightly version (neovim 0.9.0)
bash <(curl -s https://raw.githubusercontent.com/lunarvim/lunarvim/master/utils/installer/install.sh) -y

echo '================================'
echo '     Modify and source bashrc   '
echo '================================'
echo "export PATH=$HOME/.local/bin:$PATH" >> "$HOME/.bashrc"
echo "export TERM=screen-256color" >> "$HOME/.bashrc"

echo '================================'
echo '     Add My LunarVim config     '
echo '================================'
rm -rf $HOME/dotfiles-lvim
git clone https://github.com/zealzel/dotfiles-lvim.git -b shared $HOME/dotfiles-lvim
rm -rf $HOME/.config/lvim
mkdir -p $HOME/.config
ln -sf "$HOME/dotfiles-lvim/.config/lvim" "$HOME/.config/lvim"
