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

  - **box_rviz_plugins**: Provides nice plugins for RVIZ.

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

