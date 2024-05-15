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