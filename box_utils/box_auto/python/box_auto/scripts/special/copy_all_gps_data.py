from pathlib import Path
import os

OUT = "/media/jonfrey/T7/ALL_RINEX_DATA"
paths = [
    "/media/jonfrey/BoxiS1-1TB/deployment_day_14",
    "/media/jonfrey/BoxiS2-2TB/deployment_day_7",
    "/media/jonfrey/BoxiS2-2TB/deployment_day_8",
    "/media/jonfrey/BoxiS2-2TB/deployment_day_9",
    "/media/jonfrey/BoxiS4-2TB/deployment_day_5",
    "/media/jonfrey/BoxiS4-2TB/deployment_day_6",
    "/media/jonfrey/BoxiS4-2TB/deployment_day_15",
    "/media/jonfrey/T7/deployment_day_10",
    "/media/jonfrey/T7/deployment_day_11",
    "/media/jonfrey/T7/deployment_day_12",
    "/media/jonfrey/T7/deployment_day_13",
]
for p in paths:
    copy_over = [str(s) for s in Path(p).rglob("*NMZT*")]

    for file in copy_over:
        out = Path(OUT) / file.replace(str(Path(p).parent) + "/", "")
        print(f"cp {file} {out}")
        out.parent.mkdir(parents=True, exist_ok=True)
        os.system(f"cp {file} {out}")
