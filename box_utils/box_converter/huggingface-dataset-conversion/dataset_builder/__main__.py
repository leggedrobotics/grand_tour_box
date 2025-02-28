from __future__ import annotations

from pathlib import Path

from dataset_builder.build_data import build_data_part
from dataset_builder.build_metadata import build_metadata_part
from dataset_builder.dataset_config import load_config


MISSION_NAME = "2024-11-14-13-45-37"
DEFAULT_CONFIG_PATH = Path(__file__).parent / "configs" / f"{MISSION_NAME}.yaml"

DATA_PATH = Path(__file__).parent.parent / "data"

INPUT_PATH = DATA_PATH / "files" / MISSION_NAME
DATASET_PATH = DATA_PATH / "dataset" / MISSION_NAME


def main() -> int:

    DATASET_PATH.mkdir(parents=True, exist_ok=True)
    topic_registry, metadata_config = load_config(DEFAULT_CONFIG_PATH, MISSION_NAME)

    build_metadata_part(
        base_dataset_path=DATASET_PATH,
        mcaps_path=INPUT_PATH,
        metadata_config=metadata_config,
        topic_registry=topic_registry,
    )
    build_data_part(
        topic_registry=topic_registry,
        mcaps_path=INPUT_PATH,
        dataset_base_path=DATASET_PATH,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
