import yaml
import gspread
import pathlib
import os
from box_auto.utils import BOX_AUTO_DIR
import numpy as np
from pathlib import Path

# uploads output from create_mission_metadata and imu_timesync

# Google Sheet ID and sheet name
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
SHEET_NAME = "mission_overview_push"


def read_yaml(yaml_path):
    with open(yaml_path, "r") as yaml_file:
        return yaml.safe_load(yaml_file)


def upload_to_sheets(mission_data, sheet_name, spreadsheet_id, keys):
    gc = gspread.service_account(
        filename=pathlib.Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json"
    )
    sheet = gc.open_by_key(spreadsheet_id)
    worksheet = sheet.worksheet(sheet_name)
    worksheet.clear()

    cells_to_update = []
    row = 1
    for j, key in enumerate(keys):
        val = key.replace("/gt_box/", "").replace("/", "-")
        cells_to_update.append({"range": f"{chr(ord('A')+j)}{row}", "values": [[val]]})

    for mission_id, data in mission_data.items():
        row += 1
        for j, key in enumerate(keys):
            if key == "prism_distance":
                val = str(float(np.array(data["prism_distance"]).mean()))
            else:
                val = data.get(key, "")
            cells_to_update.append({"range": f"{chr(ord('A')+j)}{row}", "values": [[val]]})

    worksheet.batch_update(cells_to_update)


if __name__ == "__main__":
    # I) IMU DATA
    imu_res_path = Path(BOX_AUTO_DIR) / "cfg" / "all_missions_summary_xy_60_120_180.yaml"
    imu_res = read_yaml(imu_res_path)
    imu_summary = {}
    for timestamp_metadata, topics in imu_res.items():
        timestamp = timestamp_metadata[: timestamp_metadata.index("_")]

        if timestamp not in imu_summary:
            imu_summary[timestamp] = {k: [v / 10**6] for k, v in topics.items()}
        else:
            for k, v in topics.items():
                if k not in imu_summary[timestamp]:
                    imu_summary[timestamp][k] = [v / 10**6]
                else:
                    imu_summary[timestamp][k].append(v / 10**6)

    # II) MISSION META DATA
    yaml_path = os.path.join(BOX_AUTO_DIR, "cfg/mission_metadata.yaml")
    metadata = read_yaml(yaml_path)

    keys = [
        "mission_id",
        "bag_duration",
        "mission_duration",
        "distance_walked",
        "mission_start_time",
        "mission_stop_time",
        "prism",
        "ap20_bag_available",
        "prism_distance",
    ]
    keys += [
        "gnss_tc",
        "gnss_lc",
        "gnss_dgps",
        "gnss_ppp",
    ]

    for mission_id, data in metadata.items():
        data["mission_id"] = mission_id
        if mission_id in imu_summary:
            for k, v in imu_summary[mission_id].items():
                data[k + "_mean"] = np.mean(v)
                data[k + "_std"] = np.std(v)
                if k + "_mean" not in keys:
                    keys.append(k + "_mean")
                    keys.append(k + "_std")

    upload_to_sheets(metadata, SHEET_NAME, SPREADSHEET_ID, keys)
