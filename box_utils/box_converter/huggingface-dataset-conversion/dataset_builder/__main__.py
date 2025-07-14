from __future__ import annotations
import os
from argparse import ArgumentParser
import shutil
from argparse import ArgumentParser
from pathlib import Path
from uuid import UUID
from functools import wraps
import kleinkram
from huggingface_hub import HfApi, HfFolder, Repository
import sys
from dataset_builder.build_data import build_data_part
from dataset_builder.build_metadata import build_metadata_part
from dataset_builder.dataset_config import load_config


parser = ArgumentParser(description="Dataset Builder Script")
parser.add_argument("--mission_name", type=str, default="2024-10-01-11-29-55", help="Specify the mission name to use.")
parser.add_argument("--config_file", type=str, default="grandtour_release", help="Specify the config file to use.")
parser.add_argument("--skip_download", action="store_true", help="Skip the download of the mission data.")
args = parser.parse_args()
DOWNLOAD_FLAG = not args.skip_download
print(f"Download flag: {DOWNLOAD_FLAG}")
CONFIG_FILE_NAME = args.config_file
MISSION_NAME = args.mission_name
DEFAULT_CONFIG_PATH = Path(__file__).parent.parent / "configs" / f"{CONFIG_FILE_NAME}.yaml"
DATA_PATH = Path(f"/tmp_disk/dataset_builder_{MISSION_NAME}")

INPUT_PATH = DATA_PATH / "files" / MISSION_NAME
DATASET_PATH = DATA_PATH / "dataset" / MISSION_NAME
LOG_FILE_PATH = DATASET_PATH / "generation_report.txt"

HUGGINGFACE_DATASET_NAME = f"leggedrobotics/{MISSION_NAME}"
USERTOKEN_PATH = Path(__file__).parent.parent / "configs" / "user_token.txt"

# TODO: check how this is needed in the end

PUSHDATASET_FLAG = False


class Tee:
    """
    A class to duplicate stdout and stderr to both the terminal and a log file.
    """

    def __init__(self, log_file, stream):
        self.log_file = log_file
        self.stream = stream

    def write(self, data):
        self.log_file.write(data)
        self.stream.write(data)

    def flush(self):
        self.log_file.flush()
        self.stream.flush()


def redirect_output_to_file(log_file_path: Path):
    """
    Decorator to redirect stdout and stderr to both a log file and the terminal.
    """

    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            log_file_path.parent.mkdir(parents=True, exist_ok=True)  # Ensure the folder exists
            with open(log_file_path, "w") as log_file:
                # Redirect stdout and stderr to both the terminal and the log file
                original_stdout = sys.stdout
                original_stderr = sys.stderr
                sys.stdout = Tee(log_file, original_stdout)
                sys.stderr = Tee(log_file, original_stderr)
                try:
                    return func(*args, **kwargs)
                finally:
                    # Restore original stdout and stderr
                    sys.stdout = original_stdout
                    sys.stderr = original_stderr

        return wrapper

    return decorator


def download_mission(mission_id: UUID, input_path: Path) -> None:
    input_path.mkdir(parents=True, exist_ok=True)
    kleinkram.download(mission_ids=[mission_id], file_names=["*.bag"], dest=input_path, verbose=True)


def push_dataset_to_huggingface_api(dataset_repo: str, dataset_path: str = ".", token_path: str = USERTOKEN_PATH):
    """
    Uploads a dataset to the Hugging Face Hub using the Python API.

    """

    # TODO change loading of token
    if os.path.exists(token_path):
        with open(token_path, "r") as token_file:
            token = token_file.read().strip()
    else:
        raise FileNotFoundError(f"Token file not found at {token_path}")

    with open(token_path, "r") as token_file:
        token = token_file.read().strip()
    os.environ["HF_TOKEN"] = token

    api = HfApi(token=token)

    try:
        api.create_repo(repo_id=dataset_repo, repo_type="dataset", exist_ok=True, token=token)

    except Exception as e:
        raise RuntimeError(f"Error creating repository: {e}")

    try:
        # Use upload_large_folder for large datasets
        api.upload_large_folder(
            folder_path=dataset_path,
            repo_id=dataset_repo,
            repo_type="dataset",
            private=True,
            print_report=True,
            print_report_every=1,
        )
        print("Dataset uploaded successfully.")

    except Exception as e:
        raise RuntimeError(f"Error uploading dataset: {e}")


def run_converter(input_path: Path, output_path: Path, *, config_path: Path, mission_prefix: str) -> None:

    output_path.mkdir(parents=True, exist_ok=True)
    assert input_path.is_dir()
    topic_registry, metadata_config = load_config(config_path, mission_prefix)

    metadata = build_metadata_part(
        base_dataset_path=output_path,
        bags_path=input_path,
        metadata_config=metadata_config,
        topic_registry=topic_registry,
    )

    build_data_part(
        topic_registry=topic_registry,
        bags_path=input_path,
        metadata_config=metadata_config,
        dataset_base_path=output_path,
        metadata=metadata,
    )


@redirect_output_to_file(LOG_FILE_PATH)
def main() -> int:
    missions = kleinkram.list_missions(mission_names=["release_" + MISSION_NAME])
    assert len(missions) == 1
    mission = missions[0]

    if DOWNLOAD_FLAG:
        download_mission(mission_id=mission.id, input_path=INPUT_PATH)

    run_converter(
        input_path=INPUT_PATH,
        output_path=DATASET_PATH,
        config_path=DEFAULT_CONFIG_PATH,
        mission_prefix=MISSION_NAME,
    )

    if PUSHDATASET_FLAG:
        push_dataset_to_huggingface_api(
            dataset_repo=HUGGINGFACE_DATASET_NAME, dataset_path=DATASET_PATH, token_path=USERTOKEN_PATH
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
