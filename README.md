<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="GrandTour">
<h1 style="text-align: center;">GrandTour - The Box Software</h1>

<p align="center">
  <a href="#citation">Citation</a> •
  <a href="#project-structure">Project Structure</a> •
  <a href="#getting-started">Getting Started</a> •
  <a href="#contributing">Contributing</a> •
  <a href="#credits">Credits</a>
    
  ![Formatting](https://github.com/leggedrobotics/wild_visual_navigation/actions/workflows/formatting.yml/badge.svg)
</p>

<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="Citation">
<h2 id="citation">
Credits
</h2>

```
@INPROCEEDINGS{RSL, 
    AUTHOR    = {TBD}, 
    TITLE     = {{Grand Tour}}, 
    BOOKTITLE = {Nature}, 
    YEAR      = {2024}, 
    ADDRESS   = {TBD}, 
    MONTH     = {TBD}, 
    DOI       = {TBD} 
} 
```

<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="GrandTour">
<h2 id="project-structure">
Project Structure
</h2>

The project is organized into the following directories:

- **box_bringup**: Used to launch the box. Contains simple launch files with all the parameters (Single overlay structure).

- **box_drivers**: Contains the external drivers, either as submodules/forked or directly included in a mono repo style. These packages should be mostly static.

- **box_messages**: Includes all ROS messages needed by the full box setup.

- **box_model**: Contains the URDF model and description of the box for GAZEBO.

- **box_utils**: Helper packages for the project.

  - **box_mission_report**: Provides post-processing tools for a logged mission.

  - **box_recording**: Offers recording tools for the box.

  - **box_rviz**: Provides a RVIZ interface for visualizing all sensor data and health monitoring.

  - **box_setup**: Contains scripts to initialize the box workspace and Docker containers.

- **box_core**: Runs the core algorithms needed for the box. Packages for which a separate Git package is not useful and tightly coupled to the hardware.

  - **core_total_station**: Contains the core algorithms for the total station.

  - **core_navigation**: Contains the core algorithms for navigation.

  - **core_localization**: Contains the core algorithms for localization.

  - **core_mapping**: Contains the core algorithms for mapping.

- **box_documentation**: Documentation

<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="GrandTour">
<h2 id="getting-started">
Getting Started
</h2>

### Most important workflows:
###### Record data normally:
- power on the box -> one the jetson and the nuc a detached tmux session starts, all the drivers are started and the topics start publishing.
- on opc, run the command: `l-opc` -> a tmux session is launched, and the visualization is started. Only the throtteled topics are visualized (currently 1 Hz)
- on opc via commandline, run `start-recording` , which uses the default yaml file which saves the compressed images and the hesai packets (instead of the uncompressed pointcloud)
- the status of each recording node is visible in the rviz
- on opc run `stop-recording` top stop the recording.
- close the tmux session on opc with the command `tk`
- copy it to the opc with the command `copy_data` which copies all bags in the data folders on the nuc and jetson to the opc
- delete the data on the jetson and nuc (else it will be copied again)
- merge the bags
###### Transform bag with compressed topics into uncompressed topics (not really tested, hacky)
- run `l-replay`, which opens another tmux session
- follow the instructions echoed on the terminal: first start recording with `start-recording-uncompressed`, then play bag with --clock flag, then stop recording

###### Record data fro cam-lidar calibration (1hz images):
- power on the box -> one the jetson and the nuc a detached tmux session starts, all the drivers are started and the topics are published.
- ssh into jetson (ssh jetson), kill the tmux session `tk`, then run l-jetson-calibration -> the drivers now wtart to publish uncompressed images at 1hz and full pointclouds
- run `l-opc` on opc (visualization won't work, because throtteled topics are selected)
- start recording with `start-recording-calibration` -> this records the topics to calibrate (right now the uncompressed front facing alphasense and the pointlcouds of the lidars)
- stop recording with `stop-recording`

###### Sync clocks opc to jetson
- ssh into jetson
- check status `systemctl status chrony.service`
- restart chrony `systemctl restart chrony.service `

##### Few pointers:
- many useful aliases are defined in grand_tour_box/box_utils/box_setup/alias/alias.sh
- clock problems? restart with `restart-clocks-box`, `restart-clocks-jetson` or `restart-clocks-nuc` from opc
- no images showing? right click on image, disable hide menu and then refresh image

### Using boxi
```shell
pip3 install --user -e ~/git/grand_tour_box/box_utils/boxi
```
Now you can run:
```shell
boxi push-code-to-box --jetson
boxi black
```
And many more to be implemented to easily use the box

### Launch Structure Overview:

---
1. **tmux configuration** explained
The software is started based on pre-defined tmux configuration.
    ```shell
    tmuxp load $(rospack find box_launch)/tmux/box_replay.yaml
    ```
    Each `tmux session` is started using the `box_launch/scripts/initalize_session.sh` scripts.
We then open a set of terminals within the tmux-session.
The `tmux/box_replay.yaml` tmux configuration specifies all the box_launch/launch files that will be launched by fkie.launch.
More explanations in point 2/3 below. 
The launch files should only point to launch files within `box_launch/launch` folder.
The `tmux/box_replay.yaml` mainly launches the `roslaunch box_launch fkie.launch`.

---

2. **box_launch fkie.launch** explained
Starts the fkie node manager, which is responsible to start all launch files.
The provided launch files should all live within **box_launch/launch files**. 

---

3. **box_launch/launch files** explained
The launch files within `box_launch/launch` solely add a `capability parameter` which is used by `fkie`.
Each launch files calls the respective `bringup_XXXXX` package launch file.

---

4. **bringup_XXXXX packages** explained
The bringup packages contain all the parameters and the define the nodes that are launched.

---



### Install dependencies

##### ROS
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full python3-catkin-tools
```


##### USB HDR Cam
```
sudo apt install libv4l-dev v4l-utils
# Find camera with:
# v4l2-ctl --list-devices
```

##### Hesai Lidar
```
sudo apt install libpcap-dev
```

##### Livox Lidar
```
cd ~/catkin_ws/src/grand_tour_box/box_drivers/Livox-SDK2
mkdir build
cd build
cmake .. && make -j
sudo make install
```

##### IMU over USB
```
sudo apt install python3-pip
pip3 install pyserial
```

##### FKIE Multimaster
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install -i --as-root pip:false --reinstall --from-paths ~/catkin_ws/src/grand_tour_box/box_drivers/multimaster_fkie

##### GPS
```
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src
./grand_tour_box/box_drivers/ethz_piksi_ros/piksi_multi_cpp/install/prepare-jenkins-slave.sh
git clone git@github.com:catkin/catkin_simple.git
git clone git@github.com:ethz-asl/libsbp_catkin.git
git clone git@github.com:ethz-asl/libserialport_catkin.git
```


##### Totalstaion
```
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/cuckoo_time_translator.git
```

##### Alphasense
```
# Install alphasense drivers
# Add the Sevensense PGP key to make this machine trust Sevensense's packages.
curl -Ls http://deb.7sr.ch/pubkey.gpg | sudo gpg --dearmor -o /usr/share/keyrings/deb-7sr-ch-keyring.gpg

# Add the Sevensense APT repository to the list of known sources.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/deb-7sr-ch-keyring.gpg] http://deb.7sr.ch/alphasense/stable $(lsb_release -cs) main" \
          | sudo tee /etc/apt/sources.list.d/sevensense.list

# Install the Alphasense driver.
sudo apt update
sudo apt install alphasense-driver-core alphasense-viewer alphasense-firmware ros-noetic-alphasense-driver-ros ros-noetic-alphasense-driver

# Set the maximum socket buffer size
sudo sysctl -w net.core.rmem_max=11145728
```

##### ADIS16475 IMU

Install the [firmware](https://github.com/leggedrobotics/ADIS16475_rosserial_firmware) on the microcontroller. (probably already done) 

Install Rosserial [driver](git@github.com:leggedrobotics/adis16475_ros.git):
```
sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-arduino

sudo copy box_drivers/adis16475_ros/udev/70-adis16475.rules /etc/udev/rules.d/
sudo service udev reload  
sudo service udev restart
```

### Setting up Workspace
```
mkdir -p ~/catkin_ws/src
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Cloning the Repository and Building the Workspace
```
mkdir -p ~/git
cd ~/git
# Make sure to initialize the submodule
git clone --recurse-submodules git@github.com:leggedrobotics/grand_tour_box.git
cd ~/git/grand_tour_box
ln -s ~/git/grand_tour_box ~/catkin_ws/src
cd ~/catkin_ws
catkin build box_launch
```

### For FKIE Mulitmaster
```
cd ~/catkin_ws/src/grand_tour_box/box_drivers
rosdep install -i --as-root pip:false --reinstall --from-paths multimaster_fkie
catkin build fkie_multimaster
```

### For IMU over Arduino: 
#### Generate library
```
catkin build versavis_adis16448_receiver
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/grand_tour_box/box_drivers/smb_imu_interface/firmware
```

#### Setup udev rule
Add yourself to `dialout` group
```
sudo adduser <username> dialout
```

Copy udev rule file to your system:
```
sudo cp firmware/98-versa-vis.rules /etc/udev/rules.d/98-versa-vis.rules
```
Afterwards, use the following commands to reload the rules
```
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig
```
Note: You might have to reboot your computer for this to take effect. You can check by see whether a `/dev/versavis` is available and pointing to the correct device.

### Submodules for visualization
The rviz visualization tool uses the rviz_rqt_wrapper from ANYbotics. Follow the newest release instructions on the [bitbucket](https://bitbucket.org/leggedrobotics/anymal_rsl/wiki/Simulation%20PC%20and%20OPC%20Setup%20-%20Release%20%2222.05%22).
```
catkin build anymal_rsl_user_interface
```

### Update submodules from remote if needed
```
git submodule update --remote usb_cam
```

<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="GrandTour">
<h2 id="contributing">
Contributing
</h2>
We use `dev/name`, `fix/bug` to develop the code. 
The `main` branch is protected and only updated via pull requests. 

We follow black formatting for python code.
```
black --line-length 120 .
```

Pre-commit hooks will be added soon:
- Formatting checking
- Filesize commit checking

Actions: 
- Building the stack and running tests

### Adding submodules:
Only add submodules to the box_drivers folder.

```
cd ~/git/grand_tour_box/box_drivers
git submodule add git@github.com:ros-drivers/usb_cam.git
```

Submodules are defined in: `.gitmodules`

<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="GrandTour">
<h2 id="credits">
Credits
</h2>

### Autostart
When powered on the jetson and the nuc both automatically start a tmux session specified by box_jetson.yaml respectively box_nuc.yaml. The tmux session is detached. 
```
# Attach to the session
attach-jetson
attach-nuc
# Restart the session without powering off and on again
restart-tmux
```

### Recording of bags
The recording of bags is handled in `box_recording`.
The following launch file manages the recording of bags:
```
roslaunch box_launch rosbag_record_coordinator.launch
```
For each PC a separate launch file is needed which handles the recording of the topics on the PC:
```
roslaunch box_launch rosbag_record_node.launch
```
The topics to be recorded are specified in `box_recording/cfg/box_default.yaml`.

To start recording the following service call is needed:
```
rosservice call /gt_box/rosbag_record_coordinator/start_recording "yaml_file: ''"
```
To stop recording the following service call is needed:
```
rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false"
```
The recorded bags are stored in `grand_tour_box/box_utils/box_recording/data`.


### PTP time sync between jetson and sensors

The clock of the network interface mgbe0 on the jetson is the grandmaster of the system. Both other clocks on the jetson (the system clock and the clock of network interface mgbe1) get synchronized to it using the services phc2sys_mgbe0.service (mgbe0 to system clock) and phc2sys_mgbe1.service (mgbe0 to mgbe1).

Both interfaces on the jetson run services to publish PTP to the ethernet network. This services are called ptp4l_mgbe0.service and ptp4l_mgbe1.service respectively.

The network interface enp45s0 on the NUC is connected by ethernet via switch to the network interface mgbe0 on the Jetson (the grandmaster). The interface enp45s0 get synchronized to mgbe0 with the service ptp4l_enp45s0.sevice. This service configures the enp45s0 interface to act as a slave, i.e. to not act as a clock master in the ethernet network, such that interface mgbe0 on the jetson will always be the clock grandmaster. However, otther clocks on the same machine can still synchronize themselves to the enp45s0 interface.

The other two clocks on the NUC (the system clock and the clock of the network interface enp46s0) get synchronized to the enp45s0 clock with the services phc2sys_sys.service and phc2sys_NIC.service respectively. 

The service ptp4l_enp46s0.service allows the interface enp46s0 to publish PTP to its ethernet network, although nothing is connected there.


# Share the jetson wifi with the nuc:
run on the jetson:
```
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
```
# Merge bags from Nuc and Jetson
cd into directory with both the bags (and only the bags, nothing else)
run: ```rosbag-merge --write_bag --outbag_name $OUTBAG_NAME```

# Replay a recording

Run:
``` 
l-replay 
rosbag play --clock $MERGED_BAG_NAME
```
