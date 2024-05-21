# Install vim
sudo apt-get install vim tmux tmuxp -y


if [ -f ~/.vimrc ]; then
    cp ~/.vimrc ~/.vimrc_old
fi

cp ~/git/grand_tour_box/box_configuration/general/.vimrc ~/
mkdir -p ~/.vim/colors
cp ~/git/grand_tour_box/box_configuration/general/solarized.vim ~/.vim/colors/


if [ -f ~/.tmux.conf ]; then
    cp ~/.tmux.conf ~/.tmux.conf_old
fi
cp ~/git/grand_tour_box/box_configuration/general/.tmux.conf ~/



# add hosts
sudo -- sh -c -e "echo '' >> /etc/hosts";
sudo -- sh -c -e "echo '' >> /etc/hosts";
sudo -- sh -c -e "echo '192.168.2.51      jetson' >> /etc/hosts";
sudo -- sh -c -e "echo '192.168.2.56      nuc' >> /etc/hosts";
sudo -- sh -c -e "echo '192.168.2.57      pi' >> /etc/hosts";
sudo -- sh -c -e "echo '192.168.2.98      cpt7' >> /etc/hosts";
sudo -- sh -c -e "echo '192.168.2.154     opc' >> /etc/hosts";