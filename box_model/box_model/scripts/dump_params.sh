#!/bin/bash
sleep 2
# Dump the parameter to a YAML file in a specified directory.
rosparam dump $(rospack find box_model)/urdf/box/boxi_description.yaml /boxi_description
