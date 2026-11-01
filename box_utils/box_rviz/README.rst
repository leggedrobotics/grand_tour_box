Recording display dependency
============================

The STIM and CPT7 displays use ``rviz_plugin_tutorials/Imu``. This plugin must
be compiled against a compatible RViz library. An unresolved
``rviz::ColorProperty`` constructor when loading the plugin indicates a binary
compatibility mismatch; changing the sensor topic will not repair it.
The text overlays also require the ``jsk_rviz_plugins`` runtime dependency.
Use binaries from the same ROS distribution/vendor build as RViz. On the
validated workstation, RViz 1.14.8 required the matching ANYbotics JSK overlay
build (2.1.8); the newer upstream binary expected a different RViz ABI.
With that vendor repository configured, the matching package is
``ros-noetic-jsk-rviz-plugins=2.1.8-1focal.20220422.195006``.

To rebuild upstream release 0.11.2 against the RViz selected by an existing
catkin workspace::

    source /path/to/workspace/devel/setup.bash
    bash /path/to/grand_tour_box/box_utils/box_rviz/scripts/rebuild_rviz_plugin.sh /path/to/workspace
    source /path/to/workspace/devel/setup.bash
    roslaunch box_rviz recording.launch

The repair checks out a pinned upstream commit in
``src/rviz_plugin_tutorials_compat`` and builds only that package. Its source and
compiled plugin remain in the workspace as the installed repair. Existing
modified source checkouts are preserved and cause the script to stop. Rebuild
the plugin again if the workspace's RViz ABI changes.
