import kleinkram
from pathlib import Path


def upload_simple(project_name, mission_name, path, delete=True):
    res = kleinkram.verify(project_name=project_name, mission_name=mission_name, files=[path])
    if res[Path(path)] != "uploaded":
        print(res[Path(path)])

        fileinfo = [
            f
            for f in kleinkram.list_files(project_names=[project_name], mission_names=[mission_name])
            if Path(path).name in f.name
        ]

        if len(fileinfo) != 1:
            print("Too many files to verify aboarding - fail")
            return False

        if fileinfo[0].state == "ERROR" or fileinfo[0].state == "UPLOADING":
            if delete:
                print("File already exists - deleting")
                kleinkram.delete_file(fileinfo[0].id)
            else:
                print("File already exists - aboarding - fail")
                return False
        else:
            print("File may have other error - aboarding - fail", fileinfo[0])
            return False
    else:
        print("File already uploaded - suc")
        return True

    kleinkram.upload(
        mission_name=mission_name,
        project_name="GrandTour",
        files=[path],
        create=False,
    )
    print("File uploaded - (suc ?)")
    return True
