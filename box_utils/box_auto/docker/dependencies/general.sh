# Exit if a single command fails
set -e

apt update -y

# Installing dependencies
apt install -y libyaml-cpp-dev libtool
apt install -y vim tmux tmuxp 
apt install -y figlet
apt install -y curl # if you haven't already installed curl
apt install -y lsb-release
apt install -y openssh-client
apt install -y git-all

apt update -y

# Install python packages
apt install -y python3-opencv
apt install -y python3-pip
pip3 install tqdm
pip3 install --upgrade pip
# pip install rerun-sdk
pip install numpy==1.2

mkdir -p /root/.ssh
ssh-keyscan github.com >> /root/.ssh/known_hosts
ssh-keyscan bitbucket.org >> /root/.ssh/known_hosts


custom_ps1="\[\e[0;34m\](box_auto) \[\e[0;32m\]\u@\h:\[\e[0;34m\]\W\$ \[\e[0m\]"$
echo "PS1='$custom_ps1'" >> /root/.bashrc
