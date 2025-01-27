import numpy as np
import torch
from rosbag import Bag
from tf_bag import BagTfTransformer
from scipy.spatial.transform import Rotation
import torch.nn as nn
import torch.optim as optim
from pathlib import Path
import os
import rerun as rr
import yaml

from box_auto.utils import get_bag, MISSION_DATA


def extract_imu_data(bag_file, topic, tf_transformer=None, reference_frame=None):
    imu_data = []

    # Update return reference frame
    data_reference_frame = None
    if reference_frame is not None:
        data_reference_frame = reference_frame

    transform = None

    if not os.path.exists(bag_file):
        raise ValueError(f"Bag file {bag_file} does not exist")

    count = 0
    p = None
    with Bag(bag_file, "r") as bag:
        for _topic_name, msg, t in bag.read_messages(topics=[topic]):
            count += 1
            if count == 50000:
                print("Got 50000 samples...")
                break
            if reference_frame is not None:

                # Get static tf
                if transform is None:
                    if p is None:
                        p, q = tf_transformer.lookupTransform(reference_frame, msg.header.frame_id, msg.header.stamp)

                        trans = np.array(p)
                        # Convert quaternion to rotation matrix
                        rot = Rotation.from_quat(q).as_matrix()

                    # 1. Rotate angular velocity
                    ang = msg.angular_velocity
                    angular_velocity_target = rot @ np.array([ang.x, ang.y, ang.z])

                    # 2. Rotate linear acceleration
                    lin = msg.linear_acceleration
                    linear_acceleration_rotated = rot @ np.array([lin.x, lin.y, lin.z])

                    # 3. Account for lever arm effects
                    # 3a. Centripetal acceleration
                    # r = np.array(t)
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
    def __init__(self, time_offset_guess):
        super(TimeOffsetOptimizer, self).__init__()
        self.time_offset = nn.Parameter(torch.tensor(float(time_offset_guess), dtype=torch.float32))

    def forward(self, t1, y1, t2, y2):
        """Apply time offset to the second signal."""
        return t1, y1, t2.clone() + self.time_offset, y2


class IMUSyncOptimizer:
    def __init__(self, imu1_bag, imu1_topic, tf_bag, axis):
        self.axis = axis
        self.imu1_data, self.reference_frame = extract_imu_data(imu1_bag, imu1_topic)
        self.tf_transformer = BagTfTransformer(tf_bag)

    def time_sync_imu(self, imu2_bag, imu2_topic):
        try:
            imu2_data, reference_frame = extract_imu_data(
                imu2_bag, imu2_topic, tf_transformer=self.tf_transformer, reference_frame=self.reference_frame
            )
        except Exception as e:
            print(e)
            return 0, False

        if len(imu2_data) == 0:
            return 0, False

        t2 = torch.from_numpy(imu2_data[:, 0])
        if self.axis == "x":
            nr = 4
        elif self.axis == "y":
            nr = 5
        elif self.axis == "z":
            nr = 6

        y2 = torch.from_numpy(imu2_data[:, nr])

        t1 = torch.from_numpy(self.imu1_data[:, 0]).clone()
        y1 = torch.from_numpy(self.imu1_data[:, nr]).clone()

        val = max(t1.min(), t2.min())
        t1 = t1 - val
        t2 = t2 - val
        optimal_offset = self.optimize_time_offset(t1.cuda(), y1.cuda(), t2.cuda(), y2.cuda())
        return optimal_offset, True

    def optimize_time_offset(self, t1, y1, t2, y2, num_iterations=5000, learning_rate=0.001, max_duration_in_s=120):
        # Interpolate both signals to a common time grid
        t_min = max(t1.min(), t2.min())
        t_max = min(min(t1.max(), t2.max()), t_min + max_duration_in_s)
        t_common = torch.linspace(t_min, t_max, 50000)
        t_common = t_common.to(t1.device)
        y1_interp = interpolate(t_common, t1, y1)
        y2_interp = interpolate(t_common, t2, y2)

        # Apply cross-correlation to find initial guess
        convoluted_signal = np.correlate(
            y1_interp.cpu().detach().numpy(), y2_interp.cpu().detach().numpy(), mode="full"
        )
        num_frames = len(t_common) - 1
        frame_ids = np.arange(num_frames)
        conv_axis = np.hstack((-frame_ids[::-1][:-1], frame_ids))
        time_offset_id = conv_axis[np.argmax(convoluted_signal)]
        time_offset_guess = -t_common[abs(time_offset_id)] if time_offset_id < 0 else t_common[abs(time_offset_id)]

        # Initialize model and optimizer for fine adjustment
        model = TimeOffsetOptimizer(time_offset_guess)
        model.to(t1.device)
        optimizer = optim.Adam(model.parameters(), lr=learning_rate)

        self.t1_original = t1
        self.t2_original = t2
        self.y1_original = y1
        self.y2_original = y2

        smallest_loss, early_stopping_iter, patience, EPS = torch.inf, 0, 500, 0.0000001
        for _i in range(num_iterations):
            optimizer.zero_grad()

            # Apply time offset
            t1_adjusted, y1_adjusted, t2_adjusted, y2_adjusted = model(t1, y1, t2, y2)

            # Interpolate both signals to a common time grid
            t_min = max(t1_adjusted.min(), t2_adjusted.min())
            t_max = min(t1_adjusted.max(), t2_adjusted.max())
            t_common = torch.linspace(t_min, t_max, 100000)
            t_common = t_common.to(t1.device)

            y1_interp = interpolate(t_common, t1_adjusted, y1_adjusted)
            y2_interp = interpolate(t_common, t2_adjusted, y2_adjusted)

            # Compute MSE loss
            loss = nn.MSELoss()(y1_interp, y2_interp)
            # Backpropagate and update
            loss.backward()
            optimizer.step()
            # print("Loss: ", float(loss), " Offset: ", model.time_offset.item()) # started with 0.0076

            if loss + EPS < smallest_loss:
                smallest_loss = loss
                early_stopping_iter = 0
            else:
                early_stopping_iter += 1
                if early_stopping_iter > patience:
                    # print("Early stopping epoch: ", i)
                    break

        self.t2_final = self.t2_original.clone() + model.time_offset

        print(
            f"Time FFT Guess: {time_offset_guess * 10**9}ns,   Final Time Offset: {model.time_offset.item() * 10**9}ns"
        )

        # return time_offset_guess * 10**9
        return model.time_offset.item() * 10**9


def process_all(directory, output_folder, axis):

    summary = {}
    plot = True

    reference_imu = {
        "topic": "/gt_box/cpt7/offline_from_novatel_logs/imu",
        "bag_pattern": "*_cpt7_raw_imu.bag",
        "tf_bag_pattern": "*_tf_static.bag",
    }

    validation_imus = [
        {
            "topic": "/gt_box/ap20/imu",
            "bag_pattern": "*_jetson_ap20_synced.bag",
            "max_offset_ms": 1,
        },
        {
            "topic": "/gt_box/inertial_explorer/tc/gt_poses_novatel",
            "bag_pattern": "*_ie_tc_to_imu.bag",
            "max_offset_ms": 1,
        },
        # {
        #     "topic": "/gt_box/adis16475_node/imu",
        #     "bag_pattern": "*_jetson_adis.bag",
        #     "max_offset_ms": 1,
        # },
        # {
        #     "topic": "/gt_box/livox/imu",
        #     "bag_pattern": "*_nuc_livox.bag",
        #     "max_offset_ms": 1,
        # },
        # {
        #     "topic": "/gt_box/alphasense_driver_node/imu",
        #     "bag_pattern": "*_nuc_alphasense.bag",
        #     "max_offset_ms": 1,
        # },
        # {
        #     "topic": "/gt_box/cpt7/offline_from_novatel_logs/imu",
        #     "bag_pattern": "*_cpt7_raw_imu.bag",
        #     "max_offset_ms": 1,
        # },
    ]

    try:
        reference_imu_file = get_bag(reference_imu["bag_pattern"])
    except Exception as e:
        pattern = reference_imu["bag_pattern"]
        log = f"{pattern} [FAILED] - Getting Reference IMU - " + reference_imu["bag_pattern"]
        summary[pattern[2:]] = log
        print(log, e)
        return

    try:

        tf_bag_file = get_bag(reference_imu["tf_bag_pattern"])
    except Exception:
        pattern = reference_imu["tf_bag_pattern"]
        log = f"{pattern} [FAILED] - Getting TF bag pattern - " + reference_imu["tf_bag_pattern"]
        summary[pattern[2:]] = log
        print(log)
        return

    sync_optimizer = IMUSyncOptimizer(reference_imu_file, reference_imu["topic"], tf_bag_file, axis)

    rr.init("imu_timesync", spawn=False)

    for validation_imu in validation_imus:
        try:
            bag_file = get_bag(validation_imu["bag_pattern"])
        except Exception:
            pattern = validation_imu["bag_pattern"]
            log = f"{pattern} [FAILED] - validation_imu loading bag pattern - " + reference_imu["bag_pattern"]
            summary[pattern[2:]] = log
            print(log)
            continue

        pattern = validation_imu["bag_pattern"]

        optimal_offset_ns, suc = sync_optimizer.time_sync_imu(bag_file, validation_imu["topic"])
        if not suc:
            log = f"{pattern} [FAILED] - Error processing data"
            summary[pattern[2:]] = log
            print(log)
            continue

        if optimal_offset_ns > validation_imu["max_offset_ms"] * 10**6:
            log = f"{pattern} [FAILED] - Offset to high: {optimal_offset_ns}ns"
        else:
            log = f"{pattern} [SUC] - Offset: {optimal_offset_ns}ns"

        summary[pattern[2:]] = log
        print(log)

        if plot:

            for t in range(len(sync_optimizer.t1_original)):
                rr.set_time_seconds("time", float(sync_optimizer.t1_original[t]))
                rr.log(
                    pattern[2:] + "_reference_" + reference_imu["bag_pattern"][2:] + f"-{axis}",
                    rr.Scalar(float(sync_optimizer.y1_original[t])),
                )

            for t in range(len(sync_optimizer.t2_original)):
                rr.set_time_seconds("time", float(sync_optimizer.t2_original[t]))
                rr.log(pattern[2:] + "_original" + f"-{axis}", rr.Scalar(float(sync_optimizer.y2_original[t])))

            for t in range(len(sync_optimizer.t2_final)):
                rr.set_time_seconds("time", float(sync_optimizer.t2_final[t]))
                rr.log(pattern[2:] + "_adjusted" + f"-{axis}", rr.Scalar(float(sync_optimizer.y2_original[t])))

        rr.save(str(output_folder / f"imu_timesync-{axis}.rrd"))

    return summary


if __name__ == "__main__":
    master_summary = {}
    for axis in ["x"]:
        print("Running for axis: ", axis)

        summary_path = Path(MISSION_DATA) / "verification" / "imu_timesync_summary.yaml"
        summary_path.parent.mkdir(exist_ok=True)

        master_summary[axis] = process_all(MISSION_DATA, output_folder=summary_path.parent, axis=axis)

        # Dump the dictionary to a YAML file
        with open(str(summary_path), "w") as file:
            yaml.dump(master_summary, file, default_flow_style=False, width=1000)
