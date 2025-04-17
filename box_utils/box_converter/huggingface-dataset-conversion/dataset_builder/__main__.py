from __future__ import annotations
import os

import shutil
from argparse import ArgumentParser
from pathlib import Path
from uuid import UUID

import kleinkram
from huggingface_hub import HfApi, HfFolder, Repository

from dataset_builder.build_data import build_data_part
from dataset_builder.build_metadata import build_metadata_part
from dataset_builder.dataset_config import load_config

#TODO: make this configurable
# MISSION_NAME = "pub_2024-10-01-11-29-55"
MISSION_NAME = "release_2024-10-01-11-29-55"
# MISSION_NAME = "2024-10-01-12-00-49"

DEFAULT_CONFIG_PATH = Path(__file__).parent.parent / "configs" / f"{MISSION_NAME}.yaml"
DATA_PATH =  Path(__file__).parent.parent / "data"
INPUT_PATH = DATA_PATH / "files" / MISSION_NAME
DATASET_PATH = DATA_PATH / "dataset" / MISSION_NAME

USERTOKEN_PATH = Path(__file__).parent.parent /"configs" / "user_token.txt"

DOWNLOAD_FLAG = True


def download_mission(mission_id: UUID, input_path: Path) -> None:
    input_path.mkdir(parents=True, exist_ok=True)
    kleinkram.download(
            mission_ids=[mission_id], 
            file_names=["*.bag"], 
            dest=input_path, 
            verbose=True
        )

def push_dataset_to_huggingface_api(dataset_repo: str, dataset_path: str = ".", 
                                    token_path: str = USERTOKEN_PATH):
    """
    Uploads a dataset to the Hugging Face Hub using the Python API.

    Parameters:
    - dataset_repo (str): The full Hugging Face dataset repository name (e.g., 'leggedrobotics/GT-Testing-MLI').
    - dataset_path (str): Path to the dataset files. Defaults to current directory.
    - token (str): Your Hugging Face access token.
    """
    if os.path.exists(token_path):
        with open(token, "r") as token_file:
            token = token_file.read().strip()
    else:
        raise FileNotFoundError(f"Token file not found at {token}")


    api = HfApi()

    # Save the token for future use
    HfFolder.save_token(token)

    # Create the dataset repo on the Hub if it doesn't exist
    try:
        api.create_repo(repo_id=dataset_repo, 
                        repo_type="dataset", 
                        exist_ok=True, 

                        token=token)
    except Exception as e:
        print(f"Error creating dataset repo: {e}")
        return

    if os.path.exists(DATASET_PATH):
        shutil.rmtree(DATASET_PATH)
    repo_url = api.get_repo_url(dataset_repo, repo_type="dataset")
    repo = Repository(local_dir=DATASET_PATH, clone_from=repo_url, repo_type="dataset", use_auth_token=token)

    for file_name in os.listdir(dataset_path):
        full_file_path = os.path.join(dataset_path, file_name)
        if os.path.isfile(full_file_path):
            shutil.copy(full_file_path, os.path.join(DATASET_PATH, file_name))

    repo.push_to_hub(commit_message="Upload dataset")

    # TODO: uncomment if succensfull
    # shutil.rmtree(DATASET_PATH)
    print("Dataset uploaded successfully.")

# Example usage:
# push_dataset_to_huggingface_api("leggedrobotics/GT-Testing-MLI", ".", token="your_hf_token_here")




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

    missions = kleinkram.list_missions(mission_names=[args.mission_name])      
    assert len(missions) == 1
    mission = missions[0]
    
    if DOWNLOAD_FLAG:
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




