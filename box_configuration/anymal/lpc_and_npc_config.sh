boxi push --lpc --npc

rm ~/.bashrc; ln -s /home/rsl/git/grand_tour_box/box_configuration/anymal/lpc/.bashrc ~/
rm ~/.bashrc; ln -s /home/rsl/git/grand_tour_box/box_configuration/anymal/npc/.bashrc ~/

# Only add required packages to catkin_ws
ln -s ~/git/grand_tour_box/box_utils/box_recording ~/catkin_ws/src/
ln -s ~/git/grand_tour_box/box_launch ~/catkin_ws/src/
ln -s ~/git/grand_tour_box/box_drivers/zed2i_recording_driver ~/catkin_ws/src/

# Add data folder
ln -s /data /home/rsl/git/grand_tour_box/box_utils/box_recording/

# Build workspace
catkin build launch_anymal


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



# Create system service to turn on powerline
sudo cp /home/rsl/git/grand_tour_box/box_configuration/anymal/lpc/powerline.service /etc/systemd/system/powerline.service
sudo systemctl daemon-reload
sudo systemctl enable powerline

