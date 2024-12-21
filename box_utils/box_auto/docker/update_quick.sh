# Execute in first terminal
/home/jonfrey/git/grand_tour_box/box_utils/box_auto/docker/run.sh --type=kleinkram
cd /home/catkin_ws/src/grand_tour_box; git fetch origin --depth=1 && git reset --hard origin/main

# Execute in second terminal
docker commit $(docker ps -n 1 -q) rslethz/grand_tour_box:kleinkram; docker push rslethz/grand_tour_box:kleinkram
