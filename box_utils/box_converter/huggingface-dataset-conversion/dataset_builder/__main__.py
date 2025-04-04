from __future__ import annotations

from argparse import ArgumentParser
from pathlib import Path
from uuid import UUID

import kleinkram

from dataset_builder.build_data import build_data_part
from dataset_builder.build_metadata import build_metadata_part
from dataset_builder.dataset_config import load_config

#TODO: make this configurable
MISSION_NAME = "release_2024-11-11-12-42-47"
# MISSION_NAME = "2024-10-01-12-00-49"

DEFAULT_CONFIG_PATH = Path(__file__).parent.parent / "configs" / f"{MISSION_NAME}.yaml"

DATA_PATH = Path("/data")  # Path(__file__).parent.parent / "data"
INPUT_PATH = DATA_PATH / "files" / MISSION_NAME
DATASET_PATH = DATA_PATH / "dataset" / MISSION_NAME

DOWNLOAD_FLAG = True


def download_mission(mission_id: UUID, input_path: Path) -> None:
    input_path.mkdir(parents=True, exist_ok=True)
    kleinkram.download(
            mission_ids=[mission_id], file_names=["*.bag"], dest=input_path, verbose=True
        )


def run_converter(
    input_path: Path, output_path: Path, *, config_path: Path, mission_prefix: str
) -> None:
    
    output_path.mkdir(parents=True, exist_ok=True)
    assert input_path.is_dir()
    topic_registry, metadata_config = load_config(config_path, mission_prefix)

    build_metadata_part(
        base_dataset_path=output_path,
        bags_path=input_path,
        metadata_config=metadata_config,
        topic_registry=topic_registry,
    )
    build_data_part(
        topic_registry=topic_registry,
        bags_path=input_path,
        dataset_base_path=output_path,
    )


def main() -> int:
    parser = ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="Path to the configuration file",
    )
    parser.add_argument(
        "--mission-name",
        type=str,
        default=MISSION_NAME,
        help="Prefix for the mission",
    )

    args = parser.parse_args()

    # TODO: delete this
    breakpoint()

    #TODO: delete this line
    # if DOWNLOAD_FLAG:
    missions = kleinkram.list_missions(mission_names=[args.mission_name])      
    assert len(missions) == 1
    mission = missions[0]

    download_mission(mission_id=mission.id, input_path=INPUT_PATH)

    run_converter(
        input_path=INPUT_PATH,
        output_path=DATASET_PATH,
        config_path=args.config,
        mission_prefix=MISSION_NAME,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
