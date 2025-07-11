import kleinkram
import json
from pathlib import Path

path = "/home/jonfrey/git/grand_tour_webpage/frontend/public/data/missions.json"

MODE = "DELETE_MOVE"

# Load JSON from missions.json
with open(path, "r", encoding="utf-8") as f:
    missions = json.load(f)

# Example modification: add a new field to each mission
for mission in missions:
    if mission["split"] == "Test":
        if "MOVE" in MODE:
            out = Path("/data/GrandTour/benchmark") / mission["date_time"]
            out.mkdir(parents=True, exist_ok=True)
            kleinkram.download(
                mission_ids=[mission["uuid"]],
                file_names=["*_ap20_prism_position.bag", "*_ie_rt.bag", "*_cpt7_ie_tc.bag", "*_cpt7_imu.bag"],
                dest=out,
                verbose=True,
                overwrite=True,
            )

            files = [str(f) for f in out.rglob("*.bag")]
            kleinkram.upload(
                project_name="GrandTourBenchmark",
                mission_name="benchmark_" + mission["date_time"],
                files=files,
                create=True,
            )

        if "DELETE" in MODE:
            files = kleinkram.list_files(
                mission_ids=[mission["uuid"]],
                file_names=["*_ap20_prism_position.bag", "*_ie_rt.bag", "*_cpt7_ie_tc.bag", "*_cpt7_imu.bag"],
            )

            for f in files:
                print(f.mission_name, f.name)

                kleinkram.delete_files(file_ids=[f.id])


test_missions = [m["date_time"] for m in missions if m["split"] == "Test"]
print(test_missions)
