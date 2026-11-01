#!/usr/bin/env python3
"""Plot already computed ATE and RTE values; this tool does not recompute metrics.

Input CSV columns: name, ate, rte. Supply units matching the evaluation preset;
RTE can be a distance, an angle, or a normalized error depending on that preset.
"""

import argparse
import csv
import math
from pathlib import Path
from box_auto.utils.atomic_file import atomic_output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ate-unit", required=True)
    parser.add_argument("--rte-unit", required=True)
    args = parser.parse_args(argv)
    if args.output.suffix.lower() not in (".png", ".pdf", ".svg"):
        parser.error("output must be PNG, PDF or SVG")
    with args.csv.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not {"name", "ate", "rte"}.issubset(reader.fieldnames or []):
            raise ValueError("CSV requires name, ate and rte columns")
        rows = [(row["name"], float(row["ate"]), float(row["rte"])) for row in reader]
    if not rows or any(
        not name.strip() or not all(math.isfinite(v) and v >= 0 for v in (ate, rte)) for name, ate, rte in rows
    ):
        raise ValueError("Expected named samples with finite, nonnegative ATE and RTE")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    try:
        for name, ate, rte in rows:
            ax.scatter(ate, rte)
            ax.annotate(name, (ate, rte), xytext=(4, 4), textcoords="offset points")
        ax.set(xlabel=f"ATE ({args.ate_unit})", ylabel=f"RTE ({args.rte_unit})")
        ax.grid(alpha=0.3)
        with atomic_output(args.output, overwrite=False) as pending:
            fig.savefig(str(pending), format=args.output.suffix[1:].lower(), bbox_inches="tight")
    finally:
        plt.close(fig)


if __name__ == "__main__":
    main()
