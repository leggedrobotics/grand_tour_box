# Reproducible IJRR KIO table

Run `generate_ijrr_kio_table.py --missions-parent /path/to/missions
--output-dir /path/to/new-results --workspace /path/to/catkin-workspace
--config /path/to/IJRR_revision.yaml --dry-run` on one command line to inspect
mission discovery. Omit `--dry-run` to execute. Add `--compile-pdf` for LaTeX.

The runner uses its own ROS master. Inputs remain read-only. Existing output
requires `--reuse-existing` or `--force`; replacement is restricted to directories
previously created by this tool, outside the mission tree and protected ancestors.
Reuse requires complete expected ZIPs and identical configuration/source-bag
SHA256 provenance. Incomplete runs need a fresh output directory.

The six benchmark mission identities and method labels remain explicit in the
script. A method missing from any mission is omitted from the shared run config
and reported as missing, preserving the existing comparison policy. Missing
reference or TF bags fail before evaluation. The generated YAML records the
selected methods; JSON records missing/diverged/valid results separately.

`IJRR_revision.yaml` already enables stationary-tail removal through the shared
trajectory filter. Its radius, duration and detection plane remain preset choices.
The covariance plot displays raw trajectories in the estimator world frame,
aligning only the reference. It requires the full covariance sidecar from the
updated converter and labels its uncertainty multiplier as a display choice.

The Chiyun radar experiment is preserved in `evo_evaluations_chiyun_radar.yaml`.
Select it explicitly with the evaluation launch's `config_path` argument.
