boxi push --lpc --npc

rm ~/.bashrc; ln -s /home/rsl/git/grand_tour_box/box_configuration/anymal/lpc/.bashrc ~/
rm ~/.bashrc; ln -s /home/rsl/git/grand_tour_box/box_configuration/anymal/npc/.bashrc ~/


ln -s /data /home/rsl/git/grand_tour_box/box_utils/box_recording/

catkin build launch_anymal