# Image Anonymization

this folder contains the code and dependencies to run the image anonymization pipline.

## Running The Anonymization Pipeline

### Running Docker Container

- build:
  make sure to `klein login` before building the docker image
  (perhaps also delete the old config before you login to make sure the refresh token is as new as possible).

  ```bash
  docker build -t grand-tour-dataset .
  ```

- run
  for the `--cam` argument you can provide either `hdr`, `alphasense` or `zed2i`.

  ```bash
  docker run --rm -it --gpus all \
    --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" \
    grand-tour-dataset \
    python anonymization.py --mission-id 344d631c-387a-4aa4-bb21-92b9659ea21f --cam hdr
  ```

- debug for cluster
  ```bash
  srun --account=es_hutter --ntasks=1 --cpus-per-task=4 --gpus=1 --time=4:00:00 --mem-per-cpu=8024 --tmp=20000 --pty bash
  

  singularity exec --nv --writable  --containall $TMPDIR/grand-tour-dataset.sif /bin/bash -c 'export KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)"; /entrypoint.sh  python /app/anonymization.py --mission-id 344d631c-387a-4aa4-bb21-92b9659ea21f --cam hdr --head 100'
  ```

## Profiling

- [Download](https://www.projectaria.com/tools/egoblur/) the ego blur models `ego_blur_face.jit` and `ego_blur_lp.jit`.
  Place them here in then `./models` folder.

- Download the data:

  ```
  klein download --dest=data/files 80a3bf09-14e0-4424-94a5-4fe1901a8481
  ```

- Run the profiling setup:

  ```bash
  docker build -t grand-tour-dataset . && docker run -v "$(pwd):/app" --rm -it --gpus all grand-tour-dataset python -m cProfile -o o.pstat anonymization.py --head 200
  ```

- Render the profiling results:

  ```bash
  gprof2dot -f pstats o.pstat | dot -Tsvg -o o.svg
  ```

### Results:

- see [`rtx_4090.svg`](./profiling/rtx_4090.svg) for the profiling results on a RTX 4090 GPU.
- see [`rtx_4060.svg`](./profiling/rtx_4060.svg) for the profiling results on a RTX 4060 GPU.

Based on this we can estimate that the compute time on a single `RTX 4090` is approximately
**30-40s per 1s of mission time** . This already accounts for the different cameras that need to be blurred.

With a total mission duration of approximately 32,000s we get a total compute time of **355** hours.
