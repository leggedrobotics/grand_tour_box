#!/usr/bin/python3

import math
import yaml

'''
rosrun kalibr kalibr_calibrate_cameras \
 	--target /home/beni/catkin_ws/src/kalibr/aprilboard_6x6.yaml \
 	--models pinhole-radtan pinhole-radtan pinhole-radtan pinhole-radtan\
 	--topics /gt_box/alphasense_driver_node/cam0/compressed /gt_box/alphasense_driver_node/cam1/compressed /gt_box/alphasense_driver_node/cam3/compressed /gt_box/v4l2_camera_middle/image_raw/compressed\
 	--bag bagname.bag
'''

def roll(T):
    try:
        return math.atan(T[1][0] / T[0][0])
    except:
        print("[import_calibration] Error: gimbal lock in roll" )

def pitch(T):
    try:
        return math.atan(-T[2][0] / (math.sqrt( T[2][1]*T[2][1] + T[2][2]*T[2][2])))
    except:
        print("[import_calibration] Error: gimbal lock in pitch" )

def yaw(T):
    try:
        return math.atan(T[2][1] / T[2][2])
    except:
        print("[import_calibration] Error: gimbal lock in yaw" )

def transformation_to_xyz_rpy(transformation):
    xyz_rpy = {
        "x": transformation[0][3],
        "y": transformation[1][3],
        "z": transformation[2][3],
        "roll": roll(transformation),
        "pitch": pitch(transformation),
        "yaw": yaw(transformation),
    }
    return transformation


if __name__ == "__main__":

    topic_to_frame = {
        "/alphasense_driver_ros/cam0": "alphasense_front_left",
        "/alphasense_driver_ros/cam1": "alphasense_front_right",
        "/alphasense_driver_ros/cam3": "alphasense_front_middle",
        "/alphasense_driver_ros/cam4": "alphasense_left",
        "/alphasense_driver_ros/cam5": "alphasense_right",
    }

    kalibr_results_file = "/home/beni/data/calib/alphasense/c014/cam_cam/2023-05-24-11-14-20-camchain.yaml"
    diffcal_results_file = "/home/beni/catkin_ws/src/diffcal_gui_ros/outputs/1695973822261590242/output_config.file"
    default_calibration = "/home/beni/catkin_ws/src/grand_tour_box/box_model/box_model/urdf/box/calibrations/default_calibration.yaml"
    output_file = "/home/beni/catkin_ws/src/grand_tour_box/box_model/box_model/urdf/box/calibrations/calibration.yaml"

with open(default_calibration, 'r') as file:
    calibration = yaml.safe_load(file)

with open(kalibr_results_file, 'r') as file:
    kalibr_calibration = yaml.safe_load(file)

# print the calibration to file
with open(output_file, 'w') as file:
    yaml.dump(calibration, file)


previous_cam = None
for cam in kalibr_calibration:
    if previous_cam:
        xyz_rpy = transformation_to_xyz_rpy(kalibr_calibration[cam]["T_cn_cnm1"])
        name = topic_to_frame[kalibr_calibration[cam]["rostopic"]] + "_to_" + topic_to_frame[kalibr_calibration[previous_cam]["rostopic"]]
        if name in calibration:
            calibration[name] = xyz_rpy
        else:
            print("problem with name: ", name)

    previous_cam = cam
    #print(kalibr_calibration[c])





