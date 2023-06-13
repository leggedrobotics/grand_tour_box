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
catkin build TBD
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
cd ~git/grand_tour_box/box_drivers
git submodule add git@github.com:ros-drivers/usb_cam.git
```

Submodules are defined in: `.gitmodules`

<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="GrandTour">
<h2 id="credits">
Credits
</h2>
