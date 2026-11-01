#!/usr/bin/env python3

from __future__ import annotations

import argparse
import hashlib
import copy
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml
from box_auto.utils.evaluation_config import validate_evaluation_config
from box_auto.utils.ros_session import private_ros_session


# ---------------------------------------------------------------------------
# User configuration
# ---------------------------------------------------------------------------
# Paths and execution options are supplied through the CLI. Existing result
# directories require explicit reuse or replacement, plus our ownership marker.
DRY_RUN = False
REUSE_EXISTING = False
FORCE = False
COMPILE_PDF = False

# If a KIO method bag is missing for the from-bag evaluation run, skip that
# method in the temporary ROS conversion config and render it as missing.
# Reference and TF bags are always required.
SKIP_MISSING_METHOD_BAGS = True

# ---------------------------------------------------------------------------

POINT_RELATION_SCRIPT = Path(__file__).resolve().parent / "grandtour_point_relation.py"
PAPER_REPO = Path("/home/tutuna/ijrr_paper/IJRR---GrandTour")

MISSIONS: Tuple[Tuple[str, str], ...] = (
    ("SPX-2", "2024-11-02-17-18-32"),
    ("SNOW-2", "2024-11-03-07-57-34"),
    ("EIG-1", "2024-11-03-13-51-43"),
    ("CON-4", "2024-12-09-11-53-11"),
    ("ARC-2", "2024-11-18-13-22-14"),
    ("ARC-7", "2024-11-18-17-31-36"),
)

MISSION_FOLDER_NAMES = {
    "SPX-2": "spx_2",
    "SNOW-2": "snow_2",
    "EIG-1": "eig_1",
    "CON-4": "con_4",
    "ARC-2": "arc_2",
    "ARC-7": "arc_7",
}

REFERENCE_TEST_NAME = "ap20_robot"
TF_ONLY_TEST_NAME = ""
KIO_METHODS: Tuple[Tuple[str, str], ...] = (
    ("anymal_estimator", "ANYmal estimator"),
    ("MUSE (Smoother)", "MUSE (Smoother)"),
    ("MUSE (Invariant)", "MUSE (InEKF)"),
    ("DRIFT (1x)", "DRIFT"),
    ("CAPO", "CAPO"),
    ("MUSE", "MUSE"),
    ("CERBERUS 2.0", "Cerberus 2.0"),
    ("DRIFT(DCE)", "DRIFT + DCE"),
)

METHOD_ORDER = {test_name: i for i, (test_name, _) in enumerate(KIO_METHODS)}
METHOD_LABEL = dict(KIO_METHODS)
EXPECTED_RESULT_ZIP_COUNT = len(MISSIONS) * len(KIO_METHODS) * 2
DIVERGED_ERROR_M = 999_999.0


@dataclass(frozen=True)
class MissionRun:
    short_name: str
    timestamp: str
    bag_dir: Path
    run_dir: Path
    result_dir: Path


@dataclass
class MetricResult:
    metric: str
    path: Optional[Path]
    mean_m: Optional[float] = None
    std_m: Optional[float] = None
    rmse_m: Optional[float] = None
    status: str = "missing"
    error: Optional[str] = None

    @property
    def valid(self) -> bool:
        if self.status != "ok":
            return False
        values = (self.mean_m, self.std_m)
        return all(v is not None and math.isfinite(v) for v in values)

    @property
    def mean_cm(self) -> Optional[float]:
        return None if self.mean_m is None else self.mean_m * 100.0

    @property
    def std_cm(self) -> Optional[float]:
        return None if self.std_m is None else self.std_m * 100.0


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def write_yaml(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def find_mission_bag_dir(parent: Path, timestamp: str) -> Path:
    pattern = f"{timestamp}_jetson_ap20_robot.bag"
    matches = sorted(p for p in parent.rglob(pattern) if p.is_file())
    if not matches:
        raise FileNotFoundError(f"Could not find {pattern} under {parent}")
    if len(matches) > 1:
        listed = "\n".join(str(p) for p in matches)
        raise RuntimeError(f"Found multiple AP20 bags for timestamp {timestamp}:\n{listed}")
    return matches[0].parent


def expected_bag_name(timestamp: str, config_bag_name: str) -> str:
    if config_bag_name.startswith("_"):
        return f"{timestamp}{config_bag_name}"
    return config_bag_name


def validate_required_bags(mission_dir: Path, timestamp: str, run_config: dict) -> List[Path]:
    missing = []
    required = []
    for entry in run_config["bag_topic_pairs"]:
        name = expected_bag_name(timestamp, entry["bag"])
        path = mission_dir / name
        required.append(path)
        if not path.is_file():
            missing.append(path)
    if missing:
        msg = "\n".join(str(p) for p in missing)
        raise FileNotFoundError(f"Missing required bags for {timestamp} in {mission_dir}:\n{msg}")
    return required


def missing_bag_paths_by_test_name(parent: Path, run_config: dict) -> Dict[str, List[Path]]:
    missing: Dict[str, List[Path]] = {}
    for _, timestamp in MISSIONS:
        mission_dir = find_mission_bag_dir(parent, timestamp)
        for entry in run_config["bag_topic_pairs"]:
            test_name = entry.get("test_name", "")
            name = expected_bag_name(timestamp, entry["bag"])
            path = mission_dir / name
            if not path.is_file():
                missing.setdefault(test_name, []).append(path)
    return missing


def make_bag_backed_run_config(parent: Path, table_config: dict) -> dict:
    missing = missing_bag_paths_by_test_name(parent, table_config)
    required_support_names = {REFERENCE_TEST_NAME, TF_ONLY_TEST_NAME}
    missing_support = {test_name: paths for test_name, paths in missing.items() if test_name in required_support_names}
    if missing_support:
        lines = []
        for test_name, paths in missing_support.items():
            label = "TF model" if test_name == TF_ONLY_TEST_NAME else test_name
            lines.extend(f"{label}: {path}" for path in paths)
        raise FileNotFoundError("Missing required support bags:\n" + "\n".join(lines))

    missing_methods = [test_name for test_name, _ in KIO_METHODS if test_name in missing]
    if missing_methods and not SKIP_MISSING_METHOD_BAGS:
        lines = []
        for test_name in missing_methods:
            lines.extend(f"{test_name}: {path}" for path in missing[test_name])
        raise FileNotFoundError("Missing KIO method bags:\n" + "\n".join(lines))

    run_config = copy.deepcopy(table_config)
    if missing_methods:
        missing_set = set(missing_methods)
        run_config["bag_topic_pairs"] = [
            entry for entry in run_config["bag_topic_pairs"] if entry.get("test_name", "") not in missing_set
        ]
        print("Skipping missing KIO method bags in ROS conversion config:")
        for test_name in missing_methods:
            print(f"  {test_name}")
            for path in missing[test_name]:
                print(f"    missing: {path}")

    return run_config


def discover_missions(parent: Path, run_config: dict, output_dir: Path, reuse_existing: bool) -> List[MissionRun]:
    missions = []
    for short_name, timestamp in MISSIONS:
        bag_dir = find_mission_bag_dir(parent, timestamp)
        validate_required_bags(bag_dir, timestamp, run_config)
        run_dir = output_dir / "evo_runs" / short_name
        result_dir = run_dir / f"{timestamp}_point_relation_results"
        missions.append(MissionRun(short_name, timestamp, bag_dir, run_dir, result_dir))
    return missions


def make_kio_only_config(config: dict) -> dict:
    run_config = copy.deepcopy(config)
    keep_test_names = {REFERENCE_TEST_NAME, TF_ONLY_TEST_NAME}
    keep_test_names.update(test_name for test_name, _ in KIO_METHODS)

    pairs = []
    for entry in run_config["bag_topic_pairs"]:
        test_name = entry.get("test_name", "")
        if test_name not in keep_test_names:
            continue
        if test_name == REFERENCE_TEST_NAME:
            entry = copy.deepcopy(entry)
            entry.setdefault("evaluations", {}).setdefault("point_relation", {})["reference_file"] = ""
        pairs.append(entry)

    present = {entry.get("test_name", "") for entry in pairs}
    missing = [name for name in keep_test_names if name not in present]
    if missing:
        raise KeyError(f"KIO-only config is missing expected test_name entries: {missing}")

    run_config["bag_topic_pairs"] = pairs
    return run_config


def shell_join(args: Sequence[object]) -> str:
    return " ".join(shlex.quote(str(arg)) for arg in args)


def sourced_command(args: Sequence[object], workspace: Path) -> str:
    setup = workspace / "devel/setup.bash"
    return f"set -e; source /opt/ros/noetic/setup.bash; source {shlex.quote(str(setup))}; {shell_join(args)}"


def run_sourced(args: Sequence[object], workspace: Path, cwd: Optional[Path] = None, env=None) -> None:
    setup = workspace / "devel/setup.bash"
    if not setup.is_file():
        raise FileNotFoundError(f"Workspace setup file does not exist: {setup}")
    env = dict(os.environ if env is None else env)
    env["WS"] = str(workspace)
    env["PYTHONUNBUFFERED"] = "1"
    cmd = sourced_command(args, workspace)
    print(f"\n$ {cmd}")
    result = subprocess.run(["bash", "-lc", cmd], cwd=str(cwd or workspace), env=env)
    if result.returncode != 0:
        raise subprocess.CalledProcessError(result.returncode, cmd)


def expected_zip_path(result_dir: Path, test_name: str, metric: str, alignment: str) -> Path:
    return result_dir / f"{metric}_{test_name}_align-{alignment}_results.zip"


def alignment_name(config_entry: dict) -> str:
    n_to_align = config_entry.get("evaluations", {}).get("parameters", {}).get("n_to_align", 800)
    return "all" if n_to_align == -1 else str(n_to_align)


def method_config_entries(run_config: dict) -> Dict[str, dict]:
    entries = {}
    for entry in run_config["bag_topic_pairs"]:
        test_name = entry.get("test_name", "")
        if test_name in METHOD_LABEL:
            entries[test_name] = entry
    return entries


def expected_result_paths(result_dir: Path, run_config: dict) -> List[Path]:
    entries = method_config_entries(run_config)
    paths = []
    for test_name, _ in KIO_METHODS:
        if test_name not in entries:
            continue
        alignment = alignment_name(entries[test_name])
        for metric in ("ate", "rte"):
            paths.append(expected_zip_path(result_dir, test_name, metric, alignment))
    return paths


def all_expected_zips_exist(result_dir: Path, run_config: dict) -> bool:
    paths = expected_result_paths(result_dir, run_config)
    return bool(paths) and all(path.is_file() for path in paths)


def run_evaluation_for_mission(
    mission: MissionRun,
    run_config_path: Path,
    run_config: dict,
    workspace: Path,
    reuse_existing: bool,
) -> None:
    # Reuse is content-based: configuration or source data changes invalidate it.
    digest = hashlib.sha256(json.dumps(run_config, sort_keys=True).encode())
    for path in sorted(set(validate_required_bags(mission.bag_dir, mission.timestamp, run_config))):
        digest.update(path.name.encode())
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(block)
    # Evaluation implementation changes also invalidate cached metrics.
    pipeline_sources = sorted(Path(__file__).parent.glob("*.py"))
    converter = workspace / "devel/lib/box_auto/evo_conversion_node"
    if converter.is_file():
        pipeline_sources.append(converter)
    for path in pipeline_sources:
        digest.update(path.name.encode())
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(block)
    fingerprint = digest.hexdigest()
    provenance = mission.run_dir / "inputs.sha256"
    if reuse_existing and all_expected_zips_exist(mission.result_dir, run_config):
        if provenance.is_file() and provenance.read_text().strip() == fingerprint:
            print(f"Reusing verified results for {mission.short_name}")
            return
        raise ValueError(f"Result provenance differs or is absent: {mission.run_dir}")
    mission.run_dir.mkdir(parents=True, exist_ok=True)
    if mission.result_dir.exists():
        raise FileExistsError(f"Incomplete results require a fresh output directory: {mission.result_dir}")
    mission.result_dir.mkdir()
    with private_ros_session() as ros_env:
        run_sourced(
            [
                "roslaunch",
                "box_auto",
                "evo_preparation.launch",
                f"bag_file_directory:={mission.bag_dir}",
                f"output_folder_path:={mission.run_dir}",
                f"config_path:={run_config_path}",
                f"prefix:={mission.timestamp}",
            ],
            workspace,
            env=ros_env,
        )

    run_sourced(
        [
            "python3",
            POINT_RELATION_SCRIPT,
            "--config",
            run_config_path,
            "--input_folder_path",
            mission.run_dir,
            "--output_dir_name",
            mission.result_dir,
            "--prefix",
            mission.timestamp,
            "--disable_viz",
            "True",
        ],
        workspace,
    )

    provenance.write_text(fingerprint + "\n")


def load_metric_result(path: Path, metric: str) -> MetricResult:
    if not path.is_file():
        return MetricResult(metric=metric, path=path, status="missing")
    try:
        from evo.tools import file_interface

        result = file_interface.load_res_file(path, load_trajectories=False)
        mean_m = float(result.stats["mean"])
        std_m = float(result.stats["std"])
        rmse_m = float(result.stats["rmse"])
    except Exception as exc:
        return MetricResult(metric=metric, path=path, status="load_error", error=str(exc))

    if not all(math.isfinite(v) for v in (mean_m, std_m, rmse_m)) or mean_m >= DIVERGED_ERROR_M:
        return MetricResult(
            metric=metric,
            path=path,
            mean_m=mean_m,
            std_m=std_m,
            rmse_m=rmse_m,
            status="diverged",
        )
    return MetricResult(metric=metric, path=path, mean_m=mean_m, std_m=std_m, rmse_m=rmse_m, status="ok")


def parse_all_results(
    missions: Sequence[MissionRun],
    run_config: dict,
) -> Dict[str, Dict[str, Dict[str, MetricResult]]]:
    entries = method_config_entries(run_config)
    data: Dict[str, Dict[str, Dict[str, MetricResult]]] = {}
    for mission in missions:
        data[mission.short_name] = {}
        for test_name, _ in KIO_METHODS:
            alignment = alignment_name(entries[test_name])
            data[mission.short_name][test_name] = {
                "ate": load_metric_result(
                    expected_zip_path(mission.result_dir, test_name, "ate", alignment),
                    "ate",
                ),
                "rte": load_metric_result(
                    expected_zip_path(mission.result_dir, test_name, "rte", alignment),
                    "rte",
                ),
            }
    return data


def metric_rank_for_mission(
    results: Dict[str, Dict[str, MetricResult]],
    metric: str,
) -> Dict[str, int]:
    n_methods = len(KIO_METHODS)
    valid = []
    for test_name, _ in KIO_METHODS:
        result = results[test_name][metric]
        if result.valid:
            valid.append((result.mean_m, result.std_m, METHOD_ORDER[test_name], test_name))
    valid.sort()

    ranks = {test_name: n_methods for test_name, _ in KIO_METHODS}
    for rank, (_, _, _, test_name) in enumerate(valid, start=1):
        ranks[test_name] = rank
    return ranks


def compute_ranks(
    data: Dict[str, Dict[str, Dict[str, MetricResult]]],
) -> Tuple[Dict[str, Dict[str, Dict[str, float]]], Dict[str, float]]:
    mission_ranks: Dict[str, Dict[str, Dict[str, float]]] = {}
    summary: Dict[str, float] = {test_name: 0.0 for test_name, _ in KIO_METHODS}

    for short_name, _ in MISSIONS:
        ate_ranks = metric_rank_for_mission(data[short_name], "ate")
        rte_ranks = metric_rank_for_mission(data[short_name], "rte")
        mission_ranks[short_name] = {}
        for test_name, _ in KIO_METHODS:
            combined = (ate_ranks[test_name] + rte_ranks[test_name]) / 2.0
            mission_ranks[short_name][test_name] = {
                "ate_rank": float(ate_ranks[test_name]),
                "rte_rank": float(rte_ranks[test_name]),
                "mission_rank": combined,
            }
            summary[test_name] += combined

    for test_name in summary:
        summary[test_name] /= len(MISSIONS)

    return mission_ranks, summary


def best_second_for_metric(
    mission_results: Dict[str, Dict[str, MetricResult]],
    metric: str,
) -> Tuple[Optional[str], Optional[str]]:
    valid = []
    for test_name, _ in KIO_METHODS:
        result = mission_results[test_name][metric]
        if result.valid:
            valid.append((result.mean_m, result.std_m, METHOD_ORDER[test_name], test_name))
    valid.sort()
    best = valid[0][3] if len(valid) >= 1 else None
    second = valid[1][3] if len(valid) >= 2 else None
    return best, second


def latex_escape(text: str) -> str:
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(ch, ch) for ch in text)


def wrap_latex(text: str, wrapper: Optional[str]) -> str:
    if wrapper == "best":
        return rf"\textbf{{{text}}}"
    if wrapper == "second":
        return rf"\underline{{{text}}}"
    return text


def format_rank(value: float, wrapper: Optional[str]) -> str:
    return wrap_latex(f"{value:.2f}", wrapper)


def format_metric_cell(result: MetricResult, wrapper: Optional[str]) -> str:
    if not result.valid or result.mean_cm is None or result.std_cm is None:
        return "-"
    return wrap_latex(f"{result.mean_cm:.2f} ({result.std_cm:.2f})", wrapper)


def table_row_order(summary: Dict[str, float]) -> List[str]:
    return [
        test_name
        for test_name, _ in sorted(
            ((test_name, summary[test_name]) for test_name, _ in KIO_METHODS),
            key=lambda item: (item[1], METHOD_ORDER[item[0]]),
        )
    ]


def build_table_body(
    data: Dict[str, Dict[str, Dict[str, MetricResult]]],
    summary: Dict[str, float],
) -> str:
    sorted_methods = table_row_order(summary)
    summary_sorted = sorted((summary[test_name], METHOD_ORDER[test_name], test_name) for test_name, _ in KIO_METHODS)
    best_summary = summary_sorted[0][2]
    second_summary = summary_sorted[1][2] if len(summary_sorted) > 1 else None

    best_second = {}
    for short_name, _ in MISSIONS:
        best_second[(short_name, "rte")] = best_second_for_metric(data[short_name], "rte")
        best_second[(short_name, "ate")] = best_second_for_metric(data[short_name], "ate")

    rows = []
    for row_idx, test_name in enumerate(sorted_methods):
        prefix = r"\rowcolor{gray!10} " if row_idx % 2 == 1 else ""
        label = latex_escape(METHOD_LABEL[test_name])
        summary_wrapper = "best" if test_name == best_summary else "second" if test_name == second_summary else None
        cells = [label, format_rank(summary[test_name], summary_wrapper)]

        for short_name, _ in MISSIONS:
            for metric in ("rte", "ate"):
                best, second = best_second[(short_name, metric)]
                wrapper = "best" if test_name == best else "second" if test_name == second else None
                cells.append(format_metric_cell(data[short_name][test_name][metric], wrapper))

        rows.append(f"    {prefix}" + " & ".join(cells) + r" \\")
    return "\n".join(rows)


def build_standalone_tex(
    data: Dict[str, Dict[str, Dict[str, MetricResult]]],
    summary: Dict[str, float],
) -> str:
    body = build_table_body(data, summary)
    mission_headers = " & ".join(rf"\multicolumn{{2}}{{c}}{{{short_name}}}" for short_name, _ in MISSIONS)
    cmidrules = " ".join(rf"\cmidrule(lr){{{start}-{start + 1}}}" for start in range(3, 3 + 2 * len(MISSIONS), 2))
    metric_headers = (
        "    & "
        + " & ".join(
            header
            for _ in MISSIONS
            for header in (
                r"\makecell{RTE \\ $\mu(\sigma)[\si{\centi\meter}]$ }",
                r"\makecell{ATE \\ $\mu(\sigma)[\si{\centi\meter}]$ }",
            )
        )
        + r" \\"
    )

    return rf"""\documentclass{{article}}
\usepackage[landscape,margin=0.6in]{{geometry}}
\usepackage{{booktabs}}
\usepackage{{graphicx}}
\usepackage{{makecell}}
\usepackage{{siunitx}}
\usepackage[table]{{xcolor}}
\pagestyle{{empty}}

\begin{{document}}
\begin{{table}}[p]\centering
    \caption{{Error metrics for legged-specific kinematic-inertial odometry methods on the six GrandTour benchmark sequences, reported in \si{{\centi\meter}} (best in \textbf{{bold}}, second-best \underline{{underlined}}). \emph{{Summary}} column reports the average rank across the six missions, where for each mission the method rank is the mean of its RTE-rank and ATE-rank (lower is better). Metrics use the AP20 prism reference, the point-distance RTE/ATE protocol, $\Delta=\SI{{0.5}}{{\meter}}$, and no scale correction.}}
    \resizebox{{\linewidth}}{{!}}{{%
    \renewcommand{{\arraystretch}}{{1.18}}%
    \begin{{tabular}}{{l c @{{\hspace{{1.2em}}}} cc cc cc cc cc cc}}
    \toprule[1pt]
    & \multicolumn{{1}}{{c}}{{Summary}} & {mission_headers} \\
    \cmidrule(lr){{2-2}} {cmidrules}
    & \makecell{{Avg.\\rank}}
{metric_headers}
    \midrule[1pt]

{body}

    \bottomrule[1pt]
    \end{{tabular}}%
    }}
    \label{{table:RTE_ATE_results_KIO_generated}}
\end{{table}}
\end{{document}}
"""


def metric_result_to_json(result: MetricResult) -> dict:
    return {
        "status": result.status,
        "path": str(result.path) if result.path else None,
        "mean_m": result.mean_m,
        "std_m": result.std_m,
        "rmse_m": result.rmse_m,
        "mean_cm": result.mean_cm,
        "std_cm": result.std_cm,
        "error": result.error,
    }


def build_results_json(
    missions: Sequence[MissionRun],
    data: Dict[str, Dict[str, Dict[str, MetricResult]]],
    mission_ranks: Dict[str, Dict[str, Dict[str, float]]],
    summary: Dict[str, float],
    config_path: Path,
    run_config_path: Path,
    missions_parent: Path,
    output_dir: Path,
) -> dict:
    found = sum(
        1
        for short_name, _ in MISSIONS
        for test_name, _ in KIO_METHODS
        for metric in ("ate", "rte")
        if data[short_name][test_name][metric].path and data[short_name][test_name][metric].path.is_file()
    )

    return {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "missions_parent": str(missions_parent),
        "output_dir": str(output_dir),
        "source_config": str(config_path),
        "run_config": str(run_config_path),
        "expected_result_zip_count": EXPECTED_RESULT_ZIP_COUNT,
        "found_result_zip_count": found,
        "missions": {
            mission.short_name: {
                "timestamp": mission.timestamp,
                "bag_dir": str(mission.bag_dir),
                "run_dir": str(mission.run_dir),
                "result_dir": str(mission.result_dir),
            }
            for mission in missions
        },
        "methods": [
            {
                "test_name": test_name,
                "label": METHOD_LABEL[test_name],
                "summary_rank": summary[test_name],
                "missions": {
                    short_name: {
                        "rank": mission_ranks[short_name][test_name],
                        "ate": metric_result_to_json(data[short_name][test_name]["ate"]),
                        "rte": metric_result_to_json(data[short_name][test_name]["rte"]),
                    }
                    for short_name, _ in MISSIONS
                },
            }
            for test_name in table_row_order(summary)
        ],
    }


def compile_tex(tex_path: Path) -> None:
    cmd = ["latexmk", "-g", "-pdf", "-interaction=nonstopmode", tex_path.name]
    print(f"\n$ {shell_join(cmd)}")
    result = subprocess.run(cmd, cwd=str(tex_path.parent))
    if result.returncode != 0:
        raise subprocess.CalledProcessError(result.returncode, shell_join(cmd))


def ensure_safe_output_dir(output_dir: Path, missions_parent: Path, workspace: Path) -> None:
    resolved = output_dir.resolve()
    protected = {
        missions_parent.resolve(),
        workspace.resolve(),
        PAPER_REPO.resolve(),
        Path.home().resolve(),
        Path("/").resolve(),
    }
    if (
        resolved in protected
        or any(resolved in p.parents for p in protected)
        or missions_parent.resolve() in resolved.parents
    ):
        raise ValueError(f"Refusing to use protected directory as output-dir: {resolved}")


def prepare_output_dir(
    output_dir: Path, missions_parent: Path, workspace: Path, force: bool, reuse_existing: bool
) -> None:
    ensure_safe_output_dir(output_dir, missions_parent, workspace)
    if output_dir.exists():
        if force:
            if output_dir.is_symlink() or not (output_dir / ".box_auto_kio_output").is_file():
                raise ValueError("Replacement requires an output directory created by this tool")
            shutil.rmtree(output_dir)
        elif not reuse_existing:
            raise FileExistsError(
                f"Output directory already exists: {output_dir}\n"
                "Use --force to replace it or --reuse-existing to reuse completed evo result zips."
            )
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / ".box_auto_kio_output").touch()


def print_dry_run(
    missions: Sequence[MissionRun],
    config_path: Path,
    output_dir: Path,
    workspace: Path,
) -> None:
    print("Dry run: no files will be written and no commands will be executed.")
    print(f"Source config: {config_path}")
    print(f"Output dir:    {output_dir}")
    print(f"Workspace:     {workspace}")
    print("\nDiscovered missions:")
    for mission in missions:
        print(f"  {mission.short_name:6s} {mission.timestamp} -> {mission.bag_dir}")
    print(f"\nExpected evo result zips: {EXPECTED_RESULT_ZIP_COUNT}")
    print("\nCommands per mission:")
    for mission in missions:
        print(f"  roslaunch box_auto evo_preparation.launch ... prefix:={mission.timestamp}")
        print(f"  python3 {POINT_RELATION_SCRIPT.name} ... --prefix {mission.timestamp}")


def print_run_config(
    missions_parent: Path,
    output_dir: Path,
    config_path: Path,
    workspace: Path,
) -> None:
    print("IJRR KIO Table 12 standalone generator")
    print(f"MISSIONS_PARENT: {missions_parent}")
    print(f"OUTPUT_DIR:      {output_dir}")
    print(f"CONFIG_PATH:     {config_path}")
    print(f"WORKSPACE:       {workspace}")
    print(f"DRY_RUN:         {DRY_RUN}")
    print(f"REUSE_EXISTING:  {REUSE_EXISTING}")
    print(f"FORCE:           {FORCE}")
    print(f"COMPILE_PDF:     {COMPILE_PDF}")


def main(argv=None) -> int:
    global DRY_RUN, REUSE_EXISTING, FORCE, COMPILE_PDF
    parser = argparse.ArgumentParser(description="Reproducible IJRR KIO evaluation")
    parser.add_argument("--missions-parent", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--force", action="store_true")
    mode.add_argument("--reuse-existing", action="store_true")
    parser.add_argument("--compile-pdf", action="store_true")
    args = parser.parse_args(argv)
    missions_parent, output_dir, config_path, workspace = (
        p.expanduser().resolve() for p in (args.missions_parent, args.output_dir, args.config, args.workspace)
    )
    DRY_RUN, REUSE_EXISTING, FORCE, COMPILE_PDF = args.dry_run, args.reuse_existing, args.force, args.compile_pdf
    ensure_safe_output_dir(output_dir, missions_parent, workspace)

    if FORCE and REUSE_EXISTING:
        raise ValueError("FORCE and REUSE_EXISTING are mutually exclusive. Set only one of them to True.")

    print_run_config(missions_parent, output_dir, config_path, workspace)

    if not missions_parent.is_dir():
        raise NotADirectoryError(f"Mission parent directory does not exist: {missions_parent}")
    if not config_path.is_file():
        raise FileNotFoundError(f"Config file does not exist: {config_path}")
    if not POINT_RELATION_SCRIPT.is_file():
        raise FileNotFoundError(f"Point-relation script does not exist: {POINT_RELATION_SCRIPT}")

    source_config = validate_evaluation_config(load_yaml(config_path))
    table_config = make_kio_only_config(source_config)
    run_config = make_bag_backed_run_config(missions_parent, table_config)
    missions = discover_missions(missions_parent, run_config, output_dir, REUSE_EXISTING)

    if DRY_RUN:
        print_dry_run(missions, config_path, output_dir, workspace)
        return 0

    prepare_output_dir(output_dir, missions_parent, workspace, FORCE, REUSE_EXISTING)
    run_config_path = output_dir / "IJRR_revision_kio_table12.generated.yaml"
    write_yaml(run_config_path, run_config)

    for mission in missions:
        run_evaluation_for_mission(mission, run_config_path, run_config, workspace, REUSE_EXISTING)

    data = parse_all_results(missions, table_config)
    mission_ranks, summary = compute_ranks(data)

    results_json = build_results_json(
        missions,
        data,
        mission_ranks,
        summary,
        config_path,
        run_config_path,
        missions_parent,
        output_dir,
    )
    results_json_path = output_dir / "kio_table_12_results.json"
    results_json_path.write_text(json.dumps(results_json, indent=2), encoding="utf-8")

    tex_path = output_dir / "kio_table_12_standalone.tex"
    tex_path.write_text(build_standalone_tex(data, summary), encoding="utf-8")

    found_count = results_json["found_result_zip_count"]
    if found_count != EXPECTED_RESULT_ZIP_COUNT:
        print(
            f"Warning: found {found_count}/{EXPECTED_RESULT_ZIP_COUNT} expected evo result zips. "
            "Missing or diverged entries are rendered as '-'."
        )

    if COMPILE_PDF:
        compile_tex(tex_path)

    print("\nGenerated artifacts:")
    print(f"  {tex_path}")
    print(f"  {results_json_path}")
    if COMPILE_PDF:
        print(f"  {tex_path.with_suffix('.pdf')}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt as cause:
        print("Interrupted.", file=sys.stderr)
        raise SystemExit(130) from cause
