import kleinkram

if False:
    ms = [
        k
        for k in kleinkram.list_missions(project_ids=["3c97a27e-4180-4e40-b8af-59714de54a87"], mission_names=["*_pub"])
    ]
    [kleinkram.delete_mission(mission_id=m.id) for m in ms if m.name.endswith("_pub")]

if False:
    ms = [k for k in kleinkram.list_missions(project_ids=["3c97a27e-4180-4e40-b8af-59714de54a87"], mission_names=["*"])]
    [
        kleinkram.create_mission(
            mission_name="pub_" + m.name, project_id=m.project_id, metadata={}, ignore_missing_metadata=True
        )
        for m in ms
    ]
