"""Validate the shipped evaluation schema before creating result files."""

import copy
import math


def validate_evaluation_config(source):
    config = copy.deepcopy(source)
    if not isinstance(config, dict) or not isinstance(config.get("bag_topic_pairs"), list):
        raise ValueError("Evaluation config requires bag_topic_pairs")
    # Older presets predate these optional filters. Missing means inactive.
    config.setdefault("ap20_velocity_rejection", False)
    config.setdefault("ap20_velocity_rejection_threshold", 3.0)
    names = set()
    for entry in config["bag_topic_pairs"]:
        name = entry.get("test_name")
        if not isinstance(name, str) or name in names or "/" in name or "\\" in name or name in (".", ".."):
            raise ValueError("Invalid or duplicate evaluation name: %r" % name)
        names.add(name)
        if not entry.get("bag"):
            raise ValueError("Evaluation entry requires a bag")
        if name and (not entry.get("topic") or not entry.get("frame")):
            raise ValueError("Evaluation requires an explicit topic and frame: " + name)
    for entry in config["bag_topic_pairs"]:
        if not entry["test_name"]:
            continue  # TF-only bag.
        evaluation = entry["evaluations"]
        params = evaluation["parameters"]
        for key in ("t_max_diff", "delta"):
            if not math.isfinite(params[key]) or params[key] <= 0:
                raise ValueError(key + " must be positive and finite")
        if not math.isfinite(params.get("t_offset", 0)):
            raise ValueError("t_offset must be finite")
        n = params.get("n_to_align", -1)
        if isinstance(n, bool) or not isinstance(n, int) or (n != -1 and n < 3):
            raise ValueError("n_to_align must be -1 or at least three poses")
        for mode in ("ape_rpe", "point_relation"):
            reference = evaluation.get(mode, {}).get("reference_file", "")
            if reference and reference not in names:
                raise ValueError("Unknown reference: " + reference)
    return config
