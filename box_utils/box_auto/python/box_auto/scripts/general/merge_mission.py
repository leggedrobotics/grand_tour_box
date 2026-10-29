from pathlib import Path
from fnmatch import fnmatchcase
from rosbag import Bag
import re
import argparse
import tqdm
from box_auto.utils import MISSION_DATA, upload_bag
from box_auto.utils.atomic_file import atomic_output
from datetime import datetime, timezone
import errno
import shutil
import os
from typing import Dict, List


def str2bool(v):
    # Robust bool parsing for argparse: --overwrite=True/False, --overwrite yes/no, etc.
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in {"1", "true", "t", "yes", "y"}:
        return True
    if s in {"0", "false", "f", "no", "n"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value for --overwrite: {v}")


def merge_bags_single(input_bag, output_bag, topics="*", upload="No", verbose=False, overwrite=True):
    # From https://www.clearpathrobotics.com/assets/downloads/support/merge_bag.py
    topics = topics.split(" ")

    total_included_count = 0
    total_skipped_count = 0

    if verbose:
        print("Writing bag file: " + output_bag)
        print("Matching topics against patters: '%s'" % " ".join(topics))

    if not input_bag:
        raise ValueError("No input bags to merge")
    if any(Path(p).resolve() == Path(output_bag).resolve() for p in input_bag):
        raise ValueError("Output bag must differ from every input bag")

    with atomic_output(Path(output_bag), overwrite=overwrite) as temporary, Bag(str(temporary), "w", compression="lz4") as o:
        with tqdm.tqdm(total=len(input_bag), desc="Merging", unit="subbags", colour="green") as pbar:
            for ifile in input_bag:
                matchedtopics = []
                included_count = 0
                skipped_count = 0
                if verbose:
                    print("> Reading bag file: " + ifile)
                try:
                    with Bag(ifile, "r") as ib:
                        for topic, msg, t in ib:
                            if any(fnmatchcase(topic, pattern) for pattern in topics):
                                if topic not in matchedtopics:
                                    matchedtopics.append(topic)
                                    if verbose:
                                        print("Including matched topic '%s'" % topic)
                                o.write(topic, msg, t)
                                included_count += 1
                            else:
                                skipped_count += 1
                except Exception as e:
                    raise RuntimeError(f"Failed to merge input bag {ifile}: {e}") from e
                total_included_count += included_count
                total_skipped_count += skipped_count
                if verbose:
                    print("< Included %d messages and skipped %d" % (included_count, skipped_count))
                pbar.update(1)

        if total_included_count == 0:
            raise ValueError("Merge retained no messages; existing output is preserved")

    if verbose:
        print("Total: Included %d messages and skipped %d" % (total_included_count, total_skipped_count))

    if upload == "Yes":
        upload_bag(output_bag)

    return total_included_count, total_skipped_count


def group_key_for_output(prefix: str) -> str:
    """
    Legacy (kept for compatibility): last two underscore-separated tokens.
    Not used by the main loop after FIX 1.
    """
    parts = prefix.split("_")
    if len(parts) >= 2:
        return "_".join(parts[-2:])
    return prefix


def ensure_parent_dir(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def move_file(src: Path, dst: Path, overwrite: bool, verbose: bool) -> bool:
    """
    Move src -> dst, with cross-device fallback.
    """
    ensure_parent_dir(dst)

    if src.resolve() == dst.resolve() or (dst.exists() and src.samefile(dst)):
        return False

    if not overwrite and dst.exists():
        if verbose:
            print(f"Destination exists, skipping move: {dst}")
        return False

    try:
        if overwrite:
            os.replace(src, dst)
        else:
            os.link(src, dst)
            src.unlink()
    except OSError as exc:
        if exc.errno != errno.EXDEV:
            raise
        with atomic_output(dst, overwrite=overwrite) as temporary:
            shutil.copy2(str(src), str(temporary))
        src.unlink()
    return True


def create_output_from_single_bag(src: Path, dst: Path, overwrite: bool, verbose: bool) -> bool:
    """
    Create output bag from a single indexed bag without merging/re-writing messages.
    Tries hardlink first (no data duplication). Falls back to copy.
    Returns True if output was created/overwritten, False if skipped.
    """
    ensure_parent_dir(dst)

    if src.resolve() == dst.resolve() or (dst.exists() and src.samefile(dst)):
        return False

    if not overwrite and dst.exists():
        print(f"Output file {dst} already exists. Skipping output creation...")
        return False

    with atomic_output(dst, overwrite=overwrite) as temporary:
        try:
            os.link(str(src), str(temporary))
        except OSError:
            shutil.copy2(str(src), str(temporary))
    if verbose:
        print(f"Created single-bag output: {src} -> {dst}")

    return True


def archive_original_bags(original_files: List[Path], archive_dir: Path, overwrite: bool, verbose: bool) -> None:
    """
    Move original *_<index>.bag files into MISSION_DATA/ORIGINAL_FILES/.
    Keeps filename unchanged; if collisions happen, it appends a numeric suffix.
    """
    archive_dir.mkdir(parents=True, exist_ok=True)

    for src in original_files:
        dst = archive_dir / src.name

        if src.resolve() == dst.resolve():
            continue

        if dst.exists():
            if not overwrite:
                stem = dst.stem
                suffix = dst.suffix
                k = 1
                while True:
                    cand = archive_dir / f"{stem}__dup{k}{suffix}"
                    if not cand.exists():
                        dst = cand
                        break
                    k += 1

        if verbose:
            print(f"Archiving original: {src} -> {dst}")

        move_file(src, dst, overwrite=True, verbose=verbose)


def output_suffix_from_prefix(prefix: str, time_bag_prefix: str) -> str:
    """
    FIX 1: Collision-free output suffix.
    If prefix starts with '{time_bag_prefix}_', strip that leading mission part and keep the rest.
    Example:
      time_bag_prefix = 'example_mission_1'
      prefix          = 'example_mission_1_lpc_state_estimator'
      -> 'lpc_state_estimator'
    Fallback: return prefix unchanged.
    """
    mission_prefix = f"{time_bag_prefix}_"
    if prefix.startswith(mission_prefix):
        return prefix[len(mission_prefix):]
    return prefix


def cleanup_old_named_merged_bags(
    mission_dir: Path,
    originals_dir: Path,
    time_bag_prefix: str,
    time_str: str,
    overwrite: bool,
    upload: str,
    verbose: bool,
) -> None:
    """
    If there exist non-indexed bags with the old mission prefix name, e.g.
      example_mission_1_nuc_livox.bag
    then:
      - if the new name already exists: retain both files for explicit reconciliation
      - else: rename/move old -> new (so old disappears)
    Never touches ORIGINAL_FILES/.
    """
    indexed_pattern = re.compile(r"^(.*)_(\d+)\.bag$")
    mission_prefix = f"{time_bag_prefix}_"

    for f in mission_dir.rglob("*.bag"):
        # Skip anything in ORIGINAL_FILES
        try:
            f.relative_to(originals_dir)
            continue
        except ValueError:
            pass

        # Skip indexed bags (they're handled by the main pipeline and archived)
        if indexed_pattern.match(f.name):
            continue

        # Only old-name merged bags with mission prefix
        if not f.name.startswith(mission_prefix):
            continue

        rest = f.name[len(mission_prefix):]  # e.g. "nuc_livox.bag"
        if not rest.endswith(".bag"):
            continue

        rest_no_ext = rest[:-4]  # e.g. "nuc_livox"
        new_path = mission_dir / f"{time_str}_{rest_no_ext}.bag"

        if f.resolve() == new_path.resolve():
            continue

        if new_path.exists():
            print(f"Keeping old-name bag {f}: destination {new_path} already exists")
            continue

        # Otherwise rename/move old -> new, which removes the old name.
        if verbose:
            print(f"Renaming old-name merged bag: {f} -> {new_path}")
        move_file(f, new_path, overwrite=overwrite, verbose=verbose)

        if upload == "Yes" and new_path.exists():
            upload_bag(str(new_path))


def mission_prefixes(mission_dir: Path):
    """Reuse the established name on reruns; otherwise use the first bag time in UTC."""
    markers = sorted(mission_dir.glob("*__original__*.yml"))
    if len(markers) > 1:
        raise ValueError(f"Ambiguous mission markers: {markers}")
    if markers:
        time_str, original = markers[0].stem.split("__original__", 1)
        datetime.strptime(time_str, "%Y-%m-%d-%H-%M-%S")
        if not original:
            raise ValueError(f"Missing original mission prefix in {markers[0]}")
        return original, time_str

    suffix = "_nuc_livox_0.bag"
    candidates = sorted(mission_dir.glob(f"*{suffix}"))
    if not candidates:
        candidates = sorted((mission_dir / "ORIGINAL_FILES").glob(f"*{suffix}"))
    if len(candidates) != 1:
        raise ValueError(f"Expected one time-reference bag (*{suffix}), found: {candidates}")
    time_bag = candidates[0]
    original = time_bag.name[:-len(suffix)]
    with Bag(str(time_bag), "r") as bag:
        first = next(bag.read_messages(), None)
        if first is None:
            raise ValueError(f"No messages in time-reference bag: {time_bag}")
        time_str = datetime.fromtimestamp(first.timestamp.to_sec(), timezone.utc).strftime("%Y-%m-%d-%H-%M-%S")
    (mission_dir / f"{time_str}__original__{original}.yml").touch(exist_ok=False)
    return original, time_str


def main():
    parser = argparse.ArgumentParser(description="Fix, reindex, and merge ROS bag files.")
    parser.add_argument(
        "--overwrite",
        type=str2bool,
        default=False,
        help="Whether to overwrite existing output bag files (default: False). --overwrite=False --overwrite=True",
    )
    parser.add_argument("--upload", default="No", help="(default: No or Yes).")
    args = parser.parse_args()

    mission_dir = Path(MISSION_DATA)
    originals_dir = mission_dir / "ORIGINAL_FILES"
    originals_dir.mkdir(parents=True, exist_ok=True)

    time_bag_prefix, time_str = mission_prefixes(mission_dir)
    print(f"Mission prefix: {time_bag_prefix} -> {time_str} (UTC)")

    bag_files = sorted(mission_dir.rglob("*.bag"))
    print("Found files: ", bag_files)

    # Group bags by base prefix (everything except the trailing _<index>.bag)
    grouped_files: Dict[str, List[Path]] = {}
    indexed_pattern = re.compile(r"^(.*)_(\d+)\.bag$")

    for file in bag_files:
        # Skip already-archived bags
        try:
            file.relative_to(originals_dir)
            continue
        except ValueError:
            pass

        m = indexed_pattern.match(file.name)
        if not m:
            continue
        prefix = m.group(1)
        grouped_files.setdefault(prefix, []).append(file)

    with tqdm.tqdm(total=len(grouped_files), desc="Processing", unit="bags") as pbar:
        for prefix, files in grouped_files.items():
            # FIX 1 applied here: collision-free suffix
            out_suffix = output_suffix_from_prefix(prefix, time_bag_prefix)
            output_bag = mission_dir / f"{time_str}_{out_suffix}.bag"

            def idx(p: Path) -> int:
                mm = indexed_pattern.match(p.name)
                return int(mm.group(2)) if mm else 0

            files_sorted = sorted(files, key=idx)

            if len(files_sorted) == 1:
                src = files_sorted[0]

                created = create_output_from_single_bag(
                    src=src,
                    dst=output_bag,
                    overwrite=args.overwrite,
                    verbose=True,
                )
                if created and args.upload == "Yes":
                    upload_bag(str(output_bag))

                # Move original indexed bag into ORIGINAL_FILES (if we created output)
                if created:
                    archive_original_bags([src], originals_dir, overwrite=False, verbose=True)

                pbar.update(1)
                continue

            if not args.overwrite and output_bag.exists():
                print(f"Output file {output_bag} already exists. Skipping merge...")
                # Conservative: do not move originals if we didn't produce/overwrite output
                pbar.update(1)
                continue

            filelist = [str(f) for f in files_sorted]
            print(f"Merging files: {[str(f) for f in files_sorted]} into {output_bag}")
            merge_bags_single(filelist, str(output_bag), "*", upload=args.upload, verbose=True, overwrite=args.overwrite)

            # After successful merge: move originals into ORIGINAL_FILES
            archive_original_bags(files_sorted, originals_dir, overwrite=False, verbose=True)

            pbar.update(1)

    # Handle .svo2 files - rename them with time_str prefix
    # Example: etna_indide_cave_3_jetson_zed2i.svo2 -> {time_str}_jetson_zed2i.svo2
    svo2_pattern = re.compile(r"^(.*)_(jetson_zed2i)\.svo2$")

    for svo2_file in mission_dir.rglob("*.svo2"):
        # Skip anything in ORIGINAL_FILES
        try:
            svo2_file.relative_to(originals_dir)
            continue
        except ValueError:
            pass

        m = svo2_pattern.match(svo2_file.name)
        if not m:
            continue

        # Extract the suffix we want to keep (e.g., "jetson_zed2i")
        suffix = m.group(2)

        # Create new filename with time_str prefix
        new_name = f"{time_str}_{suffix}.svo2"
        new_path = mission_dir / new_name

        # Move the file
        move_file(svo2_file, new_path, overwrite=args.overwrite, verbose=True)

    # Retain conflicting old-name bags; rename only when the destination is absent.
    cleanup_old_named_merged_bags(
        mission_dir=mission_dir,
        originals_dir=originals_dir,
        time_bag_prefix=time_bag_prefix,
        time_str=time_str,
        overwrite=args.overwrite,
        upload=args.upload,
        verbose=True,
    )


if __name__ == "__main__":
    main()
