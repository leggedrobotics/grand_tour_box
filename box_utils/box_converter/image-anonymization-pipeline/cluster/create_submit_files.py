"""Generate cluster jobs; remote copying/submission requires --submit."""

import argparse
from pathlib import Path
import re
import shlex
import subprocess
from uuid import UUID

SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
SCRATCH_MB = {"hdr": 100000, "alphasense": 100000, "zed2i": 200000}
TIME_LIMIT = {"hdr": "10:00:00", "alphasense": "10:00:00", "zed2i": "8:00:00"}
REMOTE_SUBMIT_DIR = "/cluster/scratch/jonfrey/.submit"


def render_job(name, mission_id, camera):
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", name):
        raise ValueError(f"Mission name is not safe for a job filename: {name!r}")
    mission_id = str(UUID(str(mission_id)))
    if camera not in SCRATCH_MB:
        raise ValueError(f"Unknown camera: {camera}")
    command = (
        'export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"; '
        f"exec /entrypoint.sh python /app/anonymization.py --mission-id {shlex.quote(mission_id)} --cam {camera} --archive-dir /scratch"
    )
    return f"""#!/bin/bash
#SBATCH --account=es_hutter
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --gpus=1
#SBATCH --time={TIME_LIMIT[camera]}
#SBATCH --mem-per-cpu=4048
#SBATCH --tmp={SCRATCH_MB[camera]}
#SBATCH --output="{REMOTE_SUBMIT_DIR}/{name}_{camera}_out.log"
#SBATCH --error="{REMOTE_SUBMIT_DIR}/{name}_{camera}_err.log"
#SBATCH --open-mode=truncate

set -e -o pipefail

tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset.tar -C "$TMPDIR"
module purge
module load stack/2024-04 gcc/8.5.0 cuda/12.1.1

exec apptainer exec --nv --containall --writable --bind /cluster/scratch/jonfrey:/scratch --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" "$TMPDIR/grand-tour-dataset.sif" /bin/bash -c {shlex.quote(command)}
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--submit", action="store_true", help="Copy generated jobs to the cluster and submit them")
    parser.add_argument("--host", default="euler.ethz.ch")
    parser.add_argument("--cameras", nargs="+", choices=sorted(SCRATCH_MB), default=["zed2i"])
    parser.add_argument("--limit", type=int, default=100)
    parser.add_argument("--submit-dir", type=Path, default=Path(__file__).parent / ".submit")
    args = parser.parse_args(argv)
    if args.limit <= 0:
        parser.error("--limit must be positive")
    # Importing this module for review/tests must not read cloud sheets or schedule jobs.
    from box_auto.utils import get_uuid_mapping, read_sheet_data

    _, mission_data = read_sheet_data(SPREADSHEET_ID)
    mappings = get_uuid_mapping()
    jobs = []
    count = 0
    for name, data in mappings.items():
        if mission_data[name]["GOOD_MISSION"] != "TRUE":
            continue
        for camera in args.cameras:
            jobs.append((f"{name}_{camera}.sh", render_job(name, data["uuid_pub"], camera)))
        count += 1
        if count >= args.limit:
            break
    args.submit_dir.mkdir(parents=True, exist_ok=True)
    for name, content in jobs:
        (args.submit_dir / name).write_text(content)
    if args.submit and jobs:
        subprocess.run(["ssh", args.host, "mkdir -p " + shlex.quote(REMOTE_SUBMIT_DIR)], check=True)
        for name, _ in jobs:
            subprocess.run(["scp", str(args.submit_dir / name), f"{args.host}:{REMOTE_SUBMIT_DIR}/"], check=True)
            subprocess.run(["ssh", args.host, "sbatch " + shlex.quote(f"{REMOTE_SUBMIT_DIR}/{name}")], check=True)
    print(f"{'Submitted' if args.submit else 'Generated'} {len(jobs)} jobs in {args.submit_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
