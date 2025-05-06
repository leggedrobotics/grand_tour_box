import kleinkram

if False:
    gt = ["3c97a27e-4180-4e40-b8af-59714de54a87"]
    gt_dev = ["e5a38ae3-78b9-45c1-b682-1cc956f70791"]

    ms = [k for k in kleinkram.list_missions(project_ids=gt, mission_names=["*"])]
    # [kleinkram.delete_mission(mission_id=m.id) for m in ms if m.name.endswith("_pub")]
    print("test")

    res = kleinkram.list_files(mission_ids=[m.id for m in ms], file_names=["*.mcap"])

    for j, f in enumerate(res):
        out = kleinkram.delete_files(file_ids=[f.id])
        print(j, f.name)

if False:
    ms = [k for k in kleinkram.list_missions(project_ids=["3c97a27e-4180-4e40-b8af-59714de54a87"], mission_names=["*"])]
    [
        kleinkram.create_mission(
            mission_name="pub_" + m.name, project_id=m.project_id, metadata={}, ignore_missing_metadata=True
        )
        for m in ms
    ]
