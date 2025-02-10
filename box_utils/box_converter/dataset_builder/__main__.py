from __future__ import annotations

from dataset_builder.builder import generate_dataset


def main() -> int:
    generate_dataset()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
