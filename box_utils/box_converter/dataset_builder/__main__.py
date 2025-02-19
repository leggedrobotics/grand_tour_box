from __future__ import annotations

from dataset_builder.builder import generate_dataset
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).parent / "configs" / "default.yaml"


def main() -> int:
    generate_dataset(config_path=DEFAULT_CONFIG_PATH)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
