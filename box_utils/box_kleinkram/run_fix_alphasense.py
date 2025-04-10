from box_auto.utils import get_uuid_mapping, read_sheet_data, upload_simple, get_bag
import os
from pathlib import Path
import kleinkram
import shutil
import logging

# Configure logging
logging.basicConfig(
    filename="/home/jonfrey/fix_alphasense.log", level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

uuid_mappings = get_uuid_mapping()
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read the data and print the list
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

for name, data in uuid_mappings.items():
    if name != "2024-11-15-15-07-36":
        continue

    if name not in MISSION_DATA.keys():
        logging.warning(f"Mission not found in sheet: {name}")
        continue

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid"]

        try:
            tmp_folder = Path("/tmp/fix_alphasense") / name
            tmp_folder.mkdir(parents=True, exist_ok=True)

            # make directory if not exists
            suc = True
            while suc:
                try:
                    res = kleinkram.download(
                        mission_ids=[data["uuid_pub"]],
                        file_names=["*alphasense_color.bag"],
                        dest=tmp_folder,
                        verbose=True,
                    )
                    print(res, "Finsihed suc")
                    suc = False
                except Exception as e:
                    print("Error: ", e)
                    suc = True

            path = get_bag("*alphasense_color.bag", directory=tmp_folder)
            path_out = path.replace("alphasense_color.bag", "alphasense_tmp.bag")
            os.system(f"cp {path} {path_out}")
            os.system(
                f"export MISSION_DATA={tmp_folder}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/camera/alphasense_fix_frame_id_color.py; sleep 5;"
            )
            path_upload = get_bag("*alphasense_color.bag", directory=tmp_folder)
            upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        except Exception as e:
            logging.error(f"Error processing alphasense_color.bag for mission {name}: {e}")
        finally:
            logging.info(f"Cleaning up temporary folder for mission {name}...")
            shutil.rmtree(tmp_folder)
            # delete shtutl rm tree the tmp_dir

        # try:
        #     tmp_folder = Path("/tmp/fix_alphasense") / name
        #     tmp_folder.mkdir(parents=True, exist_ok=True)

        #     # make directory if not exists
        #     kleinkram.download(
        #         mission_ids=[data["uuid_pub"]],
        #         file_names=["*alphasense_anon.bag"],
        #         dest=tmp_folder,
        #         verbose=True,
        #     )

        #     path = get_bag("*alphasense_anon.bag", directory=tmp_folder)
        #     path_out = path.replace("alphasense_anon.bag", "alphasense_tmp.bag")
        #     os.system(f"cp {path} {path_out}")
        #     os.system(
        #         f"export MISSION_DATA={tmp_folder}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/camera/alphasense_fix_frame_id_anon.py; sleep 5;"
        #     )
        #     path_upload = get_bag("*alphasense_anon.bag", directory=tmp_folder)
        #     upload_simple(project_name="GrandTour", mission_name="pub_" + name, path=path_upload)

        # except Exception as e:
        #     logging.error(f"Error processing alphasense_anon.bag for mission {name}: {e}")
        # finally:
        #     logging.info(f"Cleaning up temporary folder for mission {name}...")
        #     shutil.rmtree(tmp_folder)
