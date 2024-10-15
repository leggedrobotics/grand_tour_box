import numpy as np
import pandas as pd
import rosbag
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, TransformStamped, Vector3
import argparse
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from pathlib import Path
import os

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")


'''
MIT License
Copyright (c) 2019 Michail Kalaitzakis

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

class GPS_utils:
	'''
		Contains the algorithms to convert a gps signal (longitude, latitude, height)
		to a local cartesian ENU system and vice versa
		
		Use setENUorigin(lat, lon, height) to set the local ENU coordinate system origin
		Use geo2enu(lat, lon, height) to get the position in the local ENU system
		Use enu2geo(x_enu, y_enu, z_enu) to get the latitude, longitude and height
	'''
	
	def __init__(self):
		# Geodetic System WGS 84 axes
		self.a  = 6378137.0
		self.b  = 6356752.314245
		self.a2 = self.a * self.a
		self.b2 = self.b * self.b
		self.e2 = 1.0 - (self.b2 / self.a2)
		self.e  = self.e2 / (1.0 - self.e2)
		
		# Local ENU Origin
		self.latZero = None
		self.lonZero = None
		self.hgtZero = None
		self.xZero = None
		self.yZero = None
		self.zZero = None
		self.R = np.asmatrix(np.eye(3))

	def setENUorigin(self, lat, lon, height):
		# Save origin lat, lon, height
		self.latZero = lat
		self.lonZero = lon
		self.hgtZero = height
		
		# Get origin ECEF X,Y,Z
		origin = self.geo2ecef(self.latZero, self.lonZero, self.hgtZero)		
		self.xZero = origin.item(0)
		self.yZero = origin.item(1)
		self.zZero = origin.item(2)
		self.oZero = np.array([[self.xZero], [self.yZero], [self.zZero]])
		
		# Build rotation matrix
		phi = np.deg2rad(self.latZero)
		lmd = np.deg2rad(self.lonZero)
		
		cPhi = np.cos(phi)
		cLmd = np.cos(lmd)
		sPhi = np.sin(phi)
		sLmd = np.sin(lmd)
		
		self.R[0, 0] = -sLmd
		self.R[0, 1] =  cLmd
		self.R[0, 2] =  0.0
		self.R[1, 0] = -sPhi * cLmd
		self.R[1, 1] = -sPhi * sLmd
		self.R[1, 2] =  cPhi
		self.R[2, 0] =  cPhi * cLmd
		self.R[2, 1] =  cPhi * sLmd
		self.R[2, 2] =  sPhi
	
	def geo2ecef(self, lat, lon, height):
		phi = np.deg2rad(lat)
		lmd = np.deg2rad(lon)
		
		cPhi = np.cos(phi)
		cLmd = np.cos(lmd)
		sPhi = np.sin(phi)
		sLmd = np.sin(lmd)
		
		N = self.a / np.sqrt(1.0 - self.e2 * sPhi * sPhi)
		
		x = (N + height) * cPhi * cLmd
		y = (N + height) * cPhi * sLmd
		z = ((self.b2 / self.a2) * N + height) * sPhi
		
		return np.array([[x], [y], [z]])
	
	def ecef2enu(self, x, y, z):
		ecef = np.array([[x], [y], [z]])
		
		return self.R * (ecef - self.oZero)
	
	def geo2enu(self, lat, lon, height):
		ecef = self.geo2ecef(lat, lon, height)
		
		return self.ecef2enu(ecef.item(0), ecef.item(1), ecef.item(2))
	
	def ecef2geo(self, x, y, z):
		p = np.sqrt(x*x + y*y)
		q = np.arctan2(self.a * z, self.b * p)
		
		sq = np.sin(q)
		cq = np.cos(q)
		
		sq3 = sq * sq * sq
		cq3 = cq * cq * cq
		
		phi = np.arctan2(z + self.e * self.b * sq3, p - self.e2 * self.a * cq3)
		lmd = np.arctan2(y, x)
		v = self.a / np.sqrt(1.0 - self.e2 * np.sin(phi) * np.sin(phi))

		lat = np.rad2deg(phi)
		lon = np.rad2deg(lmd)		
		h = (p / np.cos(phi)) - v
		
		return np.array([[lat], [lon], [h]])
		
	def enu2ecef(self, x, y, z):
		lmd = np.deg2rad(self.latZero)
		phi = np.deg2rad(self.lonZero)
		
		cPhi = np.cos(phi)
		cLmd = np.cos(lmd)
		sPhi = np.sin(phi)
		sLmd = np.sin(lmd)
		
		N = self.a / np.sqrt(1.0 - self.e2 * sLmd * sLmd)
		
		x0 = (self.hgtZero + N) * cLmd * cPhi
		y0 = (self.hgtZero + N) * cLmd * sPhi
		z0 = (self.hgtZero + (1.0 - self.e2) * N) * sLmd
		
		xd = -sPhi * x - cPhi * sLmd * y + cLmd * cPhi * z
		yd =  cPhi * x - sPhi * sLmd * y + cLmd * sPhi * z
		zd =  cLmd * y + sLmd * z
		
		return np.array([[x0+xd], [y0+yd], [z0+zd]])
	
	def enu2geo(self, x, y, z):
		ecef = self.enu2ecef(x, y, z)
		
		return self.ecef2geo(ecef.item(0), ecef.item(1), ecef.item(2))
      

    


def main():
    date = [(str(s.name)).split("_")[0] for s in Path(MISSION_DATA).glob("*_nuc_livox*.bag")][0]
    output = str(Path(MISSION_DATA) / f"{date}_cpt7_gps_optimized_trajectory.bag")
    gps_file = str(Path(MISSION_DATA) / "ie/ie.txt")

    gps_file_path = gps_file
    gps_file = pd.read_csv(gps_file_path)
    gps_file.columns = gps_file.columns.str.strip()
    position_columns = ["X-ECEF", "Y-ECEF", "Z-ECEF"]
    positions_std_columns = ["SDX-ECEF", "SDY-ECEF", "SDZ-ECEF"]
    orientation_columns = ["Heading", "Roll", "Pitch"]
    orientations_rpy_std_columns = [ "HdngSD", "RollSD",       "PitchSD"]
    times = gps_file.iloc[:, 0]
    positions = np.array(gps_file[position_columns].to_numpy())
    orientations_rpy = np.array(gps_file[orientation_columns].to_numpy())
    positions_std = np.array(gps_file[positions_std_columns].to_numpy())
    orientations_rpy_std = np.array(gps_file[orientations_rpy_std_columns].to_numpy())
    
    utils = GPS_utils()
    lat_long_h = utils.ecef2geo(positions[0,0], positions[0,1], positions[0,2])
    utils.setENUorigin(lat_long_h[0], lat_long_h[1], lat_long_h[2])
    start_time = None


    frame_id = "enu_origin"
    with rosbag.Bag(output, "w", compression="lz4") as bag:
        for i, (time, position, orientation_rpy, position_std, orientation_rpy_std) in enumerate(
            zip(times, positions, orientations_rpy, positions_std, orientations_rpy_std)
        ):
            if start_time is None:
                start_time = time
            
            R_enu__ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
                
            # Position
            position_ned = utils.ecef2enu(position[0], position[1], position[2])
            position_enu = R_enu__ned @ position_ned
            # Absolutely not sure if this is correct
            position_enu_std = R_enu__ned @ utils.R @ position_std

            # Covariance is diagonal - 6x6 matrix ( dx, dy, dz, droll, dpitch, dyaw)
            # TO ROS convention: fixed axis https://www.ros.org/reps/rep-0103.html
            quaternion_xyzw = Rotation.from_matrix( R_enu__ned @ Rotation.from_euler("ZXY", orientation_rpy, degrees=True).as_matrix() ) .as_quat()
            rot_std = Rotation.from_matrix( R_enu__ned @ Rotation.from_euler("ZXY", orientation_rpy_std, degrees=True).as_matrix() ).as_euler("xyz", degrees=False)

            # Missing **2
            covariance = np.diag(np.concatenate([rot_std[:], np.array(position_enu_std)[0,:]],axis=0))

            timestamp = rospy.Time.from_sec(time)
            output_msg = PoseWithCovarianceStamped()
            output_msg.header.seq = i
            output_msg.header.stamp = timestamp
            output_msg.header.frame_id = frame_id
            output_msg.pose.pose.position = Point(x=position_enu[1], y=position_enu[0], z=-position_enu[2])

            output_msg.pose.pose.orientation = Quaternion(
                x=quaternion_xyzw[0], y=quaternion_xyzw[1], z=quaternion_xyzw[2], w=quaternion_xyzw[3]
            )
            output_msg.pose.covariance = covariance.astype(np.float32).flatten().tolist()
            bag.write(topic="/gt_box/inertial_explorer/gt_poses_novatel", msg=output_msg, t=timestamp)
            
            odometry_msg = Odometry()
            odometry_msg.header = output_msg.header
            odometry_msg.pose.pose = output_msg.pose.pose
            odometry_msg.child_frame_id = "box_base"
            bag.write(topic="/gt_box/inertial_explorer/odometry", msg=odometry_msg, t=timestamp)

            tf_message = TFMessage()
            tf_message.transforms = []
            box_transform = TransformStamped()
            box_transform.header = odometry_msg.header
            box_transform.header.frame_id = "box_base"
            box_transform.child_frame_id = frame_id

            position_enu = np.array(position_enu)[:,0]
			
            SE3 = np.eye(4)
            SE3[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
            SE3[:3, 3] = [position_enu[1], position_enu[0], -position_enu[2]]
            SE3 = np.linalg.inv(SE3)
            box_transform.transform.translation = Vector3(x=SE3[0,3], y=SE3[1,3], z=SE3[2,3])

            q = Rotation.from_matrix(SE3[:3, :3]).as_quat()
            box_transform.transform.rotation = Quaternion(q[0],q[1],q[2],q[3])
			
            tf_message.transforms.append(box_transform)
            bag.write(topic="/tf", msg=tf_message, t=timestamp)


if __name__ == "__main__":
    main()
