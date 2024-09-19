import numpy as np
import torch
from rosbag import Bag
from tf_bag import BagTfTransformer
from tqdm import tqdm
import argparse
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from torchinterp1d import interp1d

class BagTfTransformerWrapper:
    def __init__(self, bag):
        self.tf_listener = BagTfTransformer(bag)
    def waitForTransform(self, parent_frame, child_frame, time, duration):
        return self.tf_listener.waitForTransform(parent_frame, child_frame, time)
    def lookupTransform(self, parent_frame, child_frame, time):
        try:
            return self.tf_listener.lookupTransform(parent_frame, child_frame, time)
        except Exception:
            return (None, None)
        
def extract_imu_data(bag_file, topic, tf_transformer=None, reference_frame=None):
    imu_data = []

    # Update return reference frame
    data_reference_frame = None
    if reference_frame is not None:
        data_reference_frame = reference_frame
    
    transform = None 

    with Bag(bag_file, 'r') as bag:
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            if reference_frame is not None:

                # Get static tf
                if transform is None:
                    p,q = tf_transformer.lookupTransform(reference_frame,
                                            msg.header.frame_id,
                                            msg.header.stamp)
                    
                    trans = np.array(p)
                    # Convert quaternion to rotation matrix
                    rot = Rotation.from_quat(q).as_matrix()
                    
                    # 1. Rotate angular velocity
                    ang = msg.angular_velocity
                    angular_velocity_target = rot @ np.array([ ang.x, ang.y, ang.z ])
                    
                    # 2. Rotate linear acceleration
                    lin = msg.linear_acceleration
                    linear_acceleration_rotated = rot @ np.array([ lin.x, lin.y, lin.z ])
                    
                    # 3. Account for lever arm effects
                    # 3a. Centripetal acceleration
                    r = np.array(t)
                    centripetal_acc = np.cross(angular_velocity_target, np.cross(angular_velocity_target, trans))
                    
                    # 3b. Tangential acceleration (assuming angular acceleration is zero for simplicity)
                    # If you have angular acceleration data, you can uncomment the following lines:
                    # alpha = ... # Calculate or provide angular acceleration
                    # tangential_acc = np.cross(alpha, r)
                    # linear_acceleration_target = linear_acceleration_rotated + centripetal_acc + tangential_acc
                    
                    # Without angular acceleration:
                    linear_acceleration_target = linear_acceleration_rotated + centripetal_acc
                    
                    msg.linear_acceleration.x = linear_acceleration_target[0]
                    msg.linear_acceleration.y = linear_acceleration_target[1]
                    msg.linear_acceleration.z = linear_acceleration_target[2]
                    msg.angular_velocity.x = angular_velocity_target[0]
                    msg.angular_velocity.y = angular_velocity_target[1]
                    msg.angular_velocity.z = angular_velocity_target[2]


                # Transform imu data to same orign here.
                msg = msg

            ts = t.to_sec()
            if data_reference_frame is None:
                data_reference_frame = msg.header.frame_id

            acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            rot_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            imu_data.append([ts] + acc + rot_vel)

    return np.array(imu_data, dtype=np.float64), data_reference_frame


import torch
import torch.nn as nn
import torch.optim as optim

def interpolate(t_common, t1_adjusted, y1_adjusted):
    # Find indices for lower bound of each t_common
    indices = torch.searchsorted(t1_adjusted, t_common, right=True) - 1
    
    # Clip indices to valid range
    indices = torch.clamp(indices, 0, len(t1_adjusted) - 2)

    # Get lower and upper bounds
    t0 = t1_adjusted[indices]
    t1 = t1_adjusted[indices + 1]
    y0 = y1_adjusted[indices]
    y1 = y1_adjusted[indices + 1]

    # Compute weights
    w1 = (t_common - t0) / (t1 - t0)
    w0 = 1 - w1

    # Perform linear interpolation
    y_common = w0 * y0 + w1 * y1

    return y_common

class TimeOffsetOptimizer(nn.Module):
    def __init__(self):
        super(TimeOffsetOptimizer, self).__init__()
        self.time_offset = nn.Parameter(torch.tensor(0.0))

    def forward(self, t1, y1, t2, y2):
        """Apply time offset to the second signal."""
        return t1, y1, t2.clone() + self.time_offset, y2

class IMUSyncOptimizer:
    def __init__(self, imu1_bag, imu2_bag, tf_bag, imu1_topic, imu2_topic):

        imu1_data, reference_frame = extract_imu_data(imu1_bag, imu1_topic)
        tf_transformer = BagTfTransformerWrapper(tf_bag)
        imu2_data, reference_frame = extract_imu_data(imu2_bag, imu2_topic, tf_transformer = tf_transformer, reference_frame=reference_frame)

        t1 = torch.from_numpy( imu1_data[:, 0])
        y1 = torch.from_numpy( imu1_data[:, 4])
        t2 = torch.from_numpy( imu2_data[:, 0])
        y2 = torch.from_numpy( imu2_data[:, 4])

        val = max( t1.min(), t2.min() )
        t1 = t1 - val
        t2 = t2 - val
        optimal_offset = self.optimize_time_offset(t1.cuda(), y1.cuda(), t2.cuda(), y2.cuda())
        print(f"Optimal time offset: {optimal_offset}")
    
    def optimize_time_offset(self, t1, y1, t2, y2, num_iterations=100, learning_rate=0.01):
        model = TimeOffsetOptimizer()
        model.to(t1.device)

        optimizer = optim.Adam(model.parameters(), lr=learning_rate)
        print("start optimizing")
        for _ in range(num_iterations):
            optimizer.zero_grad()
            
            # Apply time offset
            t1_adjusted, y1_adjusted, t2_adjusted, y2_adjusted = model(t1, y1, t2, y2)
            
            # Interpolate both signals to a common time grid
            t_min = min(t1_adjusted.min(), t2_adjusted.min())
            t_max = max(t1_adjusted.max(), t2_adjusted.max())
            t_common = torch.linspace(t_min, t_max, 100000)
            t_common = t_common.to(t1.device)
            
            y1_interp = interpolate(t_common, t1_adjusted, y1_adjusted)
            y2_interp = interpolate(t_common, t2_adjusted, y2_adjusted)
            
            # Compute MSE loss
            loss = nn.MSELoss()(y1_interp, y2_interp)
            
            # Backpropagate and update
            loss.backward()
            optimizer.step()
            print("Loss: ", float(loss), " Offset: ", model.time_offset) # started with 0.0076
        
        return model.time_offset.item()
    

if __name__ == "__main__":
    base = "/Data/Projects/GrandTour/lee_k_democracy/2024-09-18-10-59-00/2024-09-18-10-59-00"
    parser = argparse.ArgumentParser(description="IMU Synchronization and Time Offset Optimization")
    parser.add_argument("--imu1_bag", default=f"{base}_jetson_ap20_0.bag", help="Path to IMU1 rosbag file")
    parser.add_argument("--imu2_bag", default=f"{base}_nuc_cpt7_0.bag", help="Path to IMU2 rosbag file")
    parser.add_argument("--tf_bag", default=f"{base}_nuc_tf_0.bag", help="Path to TF rosbag file")
    parser.add_argument("--imu1_topic", default="/gt_box/ap20/imu", help="IMU1 topic name")
    parser.add_argument("--imu2_topic", default="/gt_box/cpt7/imu/data_raw", help="IMU2 topic name")
    
    args = parser.parse_args()
    
    sync_optimizer = IMUSyncOptimizer(
        args.imu1_bag, args.imu2_bag, args.tf_bag,
        args.imu1_topic, args.imu2_topic
    )