<h1 style="text-align: center;">GrandTour - The Box Software</h1>

<p align="center">
  <a href="#citation">Citation</a> •
  <a href="#project-structure">Project Structure</a> •
  <a href="#getting-started">Getting Started</a> •
  <a href="#contributing">Contributing</a> •
  <a href="#credits">Credits</a>
    
  ![Formatting](https://github.com/leggedrobotics/wild_visual_navigation/actions/workflows/formatting.yml/badge.svg)
</p>

<img align="right" width="40" height="40" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="GrandTour">

## Citation

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

<h2 id="getting-started">
Getting Started
</h2>

## Contributing

## Credits
