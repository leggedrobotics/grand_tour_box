# Image anonymization

The local and deployed entry points share `bag_anonymization.py` and
`image_codec.py`. Detection thresholds and model generations remain selectable
as described below; this code does not establish detection recall on a dataset.

## Local environment

Use the existing `christian_grandtour` conda environment (Python 3.11, PyTorch,
Torchvision, Detectron2, OpenCV and ROS 1 Python modules). Run commands from this
directory. The local tools require CUDA.

```bash
conda run -n christian_grandtour python anonimization_standalone_chat.py \
  /path/to/input.bag --model gen2 --camera alphasense --output /path/to/output_anon.bag
```

`anonimization_standalone.py` supports Gen2 only. The `_chat` entry point supports
`--model gen1` and `--model gen2`. Both accept `--topics` followed by exact topic
names, `--models`, `--face-threshold`, `--plate-threshold`, and `--head`.

Model files in `models/` use these names:

- Gen1: `ego_blur_face_gen1.jit`, `ego_blur_lp_gen1.jit`
- Gen2: `ego_blur_face_gen2.jit`, `ego_blur_lp_gen2.jit`

Download models from the [EgoBlur project](https://www.projectaria.com/tools/egoblur/).
The standalone face thresholds remain 0.4 (Gen2-only script) and 0.51
(selectable script); their plate threshold remains 0.85.

## Output guarantees and scope

- Every selected topic must exist, contain images, and have processed messages.
  Full runs check processed image counts against the input bag index.
- All entry points write to a temporary sibling directory and publish the closed
  bag atomically. Existing files are preserved, including concurrent outputs;
  exceptions and interruption remove the unfinished bag.
- Sensor headers, bag timestamps, image dimensions, numeric depth, RGB/BGR
  encoding, byte order and alpha are retained. Raw rows may be repacked without
  padding. PNG remains PNG; JPEG is re-encoded at quality 95 and is lossy.
- `--head N` counts **all bag messages**, including metadata. Every selected
  camera must appear in the sample. Default sample names end in `_anon_sample.bag`.
- Local tools process only selected image topics; other streams pass through.
  The cloud tool requires **all image topics** in each input bag to be selected.
- Previews are disabled by default. Explicit `--preview-dir /path/to/previews`
  produces side-by-side panels containing the **original image** as well as the
  redacted image. Preview files therefore contain unredacted content.

## Docker and cloud processing

`anonymization.py` retains the deployed **Gen1** model choice and thresholds
(face 0.4, plate 0.85). It accepts both the Gen1 names above and the original
`ego_blur_face.jit` / `ego_blur_lp.jit` names. It never substitutes Gen2 weights.
Docker checks for these files at build time. The Noetic container uses Kleinkram
0.57.0 for Python 3.8; calls are also compatible with the installed 0.63.3 SDK in
`christian_grandtour`.

```bash
docker build -t grand-tour-dataset .
docker run --rm --gpus all \
  --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" \
  --mount type=bind,src=/absolute/path/to/fresh-work-directory,dst=/data \
  grand-tour-dataset python /app/anonymization.py --mission-id MISSION_UUID --cam hdr
```

Camera choices are `hdr`, `alphasense`, and `zed2i`. `--models` changes the model
directory and `--data-dir` changes the input/output root. Use a fresh work
directory; previous inputs and outputs are preserved. Importing the script or
asking for `--help` creates no data directories.

A full cloud run uploads only after all configured bags finish. Upload and
verification use the exact mission ID; success requires a matching remote hash.
Pending upload/hash computation is polled for up to approximately five minutes;
timeouts and API/transfer failures propagate as failures. Existing remote files
are preserved unless `--replace-existing` is supplied. Replacement matches the
exact filename and never deletes an in-progress upload.

`--no-upload` retains full results locally. `--head N` always produces sample
names and skips uploading, including when `--replace-existing` is supplied.
Outputs remain under `--data-dir/output` for inspection or a later upload retry.
`--archive-dir` retains an additional completed copy before uploading. Generated
cluster jobs use `/scratch` for this copy; archive publication is atomic and
preserves existing files.

## Cluster jobs

```bash
python cluster/create_submit_files.py --cameras zed2i --limit 1
```

This reads mission metadata and generates jobs locally. Add `--submit` to copy
and submit those jobs; `--host` and `--submit-dir` select their destinations.
Generated jobs propagate setup and processing failures to Slurm. Importing the
generator performs no network or scheduling operations. The cluster paths and
account in the generator are installation-specific and must match the cluster.

`cluster/convert_docker_to_tar.sh` stops on failed build/export/copy steps.
`GRAND_TOUR_EXPORT_DIR` can select its local export directory.

## Focused verification

```bash
conda run -n christian_grandtour python -m unittest discover -s tests -v
```

These tests use disposable synthetic bags and mocked cloud APIs. They exercise
encoding and metadata preservation, missing/corrupt streams, interruption,
concurrent publication, sample upload exclusion, exact mission/file selection,
verification failures, and generated job exit codes. They perform no inference,
cloud requests, dataset processing or cluster submission.

A separate one-image inference check can use an ordinary image outside any
recording. A successful smoke test establishes runtime compatibility, not a
missed-face/plate rate or sufficient obscuration for dataset release.
