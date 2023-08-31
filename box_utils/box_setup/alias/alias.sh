#!/bin/bash

alias tk='tmux kill-server'
alias gs='git status'
alias recho='rostopic echo'
alias rlist='rostopic list'
alias rhz='rostopic hz'
alias rinfo='rostopic info'

alias l-nuc='tmuxp load $(rospack find box_launch)/tmux/box_nuc.yaml'
alias l-nuc-calibration='tmuxp load $(rospack find box_launch)/tmux/box_nuc_calibration.yaml'
alias l-jetson='tmuxp load $(rospack find box_launch)/tmux/box_jetson.yaml'
alias l-jetson-calibration='tmuxp load $(rospack find box_launch)/tmux/box_jetson_calibration.yaml'
alias l-opc='tmuxp load $(rospack find box_launch)/tmux/box_opc.yaml'
alias l-replay='tmuxp load $(rospack find box_launch)/tmux/box_replay.yaml'
alias ssh-nuc='ssh -X rsl@192.168.2.56'
alias ssh-jetson='ssh -X rsl@192.168.2.51'
alias ping-nuc='ping 192.168.2.56'
alias ping-jetson='ping 192.168.2.51'
alias attach-nuc='tmux attach-session -t box_nuc'
alias attach-jetson='tmux attach-session -t box_jetson'

restart-tmux(){
    echo "this will take 30 seconds..."
    sudo systemctl restart autostart_tmux

}

alias reboot-nuc='ssh -t rsl@192.168.2.56 "echo rsl | sudo -S reboot"'
alias reboot-jetson='ssh -t rsl@192.168.2.51 "echo rsl | sudo -S reboot"'
alias reboot-box='reboot-nuc; reboot-jetson'

alias shutdown-nuc='ssh -t rsl@192.168.2.56 "echo rsl | sudo -S shutdown -h now"'
alias shutdown-jetson='ssh -t rsl@192.168.2.51 "echo rsl | sudo -S shutdown -h now"'
alias shutdown-box='shutdown-nuc; shutdown-jetson'

restart-clocks-jetson(){
    ssh -t rsl@192.168.2.51 "echo rsl | sudo -S systemctl restart ptp4l_mgbe0.service"
    ssh -t rsl@192.168.2.51 "echo rsl | sudo -S systemctl restart ptp4l_mgbe1.service"

    ssh -t rsl@192.168.2.51 "echo rsl | sudo -S systemctl restart phc2sys_mgbe0.service"
    ssh -t rsl@192.168.2.51 "echo rsl | sudo -S systemctl restart phc2sys_mgbe1.service"
}

restart-clocks-nuc(){
    ssh -t rsl@192.168.2.56 "echo rsl | sudo -S systemctl restart ptp4l_enp45s0.service"
    ssh -t rsl@192.168.2.56 "echo rsl | sudo -S systemctl restart ptp4l_enp46s0.service"

    ssh -t rsl@192.168.2.56 "echo rsl | sudo -S systemctl restart phc2sys_system.service"
    ssh -t rsl@192.168.2.56 "echo rsl | sudo -S systemctl restart phc2sys_NIC.service"
}

restart-clocks-box(){
    restart-clocks-jetson
    restart-clocks-nuc
}

copy_data(){
    rosrun box_recording copy_mission_data_from_robot.sh rsl jetson .
    rosrun box_recording copy_mission_data_from_robot.sh rsl nuc .
}

start-recording(){
   rosservice call /gt_box/rosbag_record_coordinator/start_recording "yaml_file: ''" 
}

start-recording-calibration(){
    rosservice call /gt_box/rosbag_record_coordinator/start_recording "yaml_file: '/home/rsl/git/grand_tour_box/box_utils/box_recording/cfg/box_calibration.yaml'" 
}

start-recording-uncompressed(){
   rosservice call /gt_box/rosbag_record_coordinator/start_recording "yaml_file: '/home/beni/git/grand_tour_box/box_utils/box_recording/cfg/box_uncompressed.yaml'" 
}

stop-recording(){
    rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false" 
}