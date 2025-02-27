# Image Anonymization

this folder contains the code and dependencies to run the image anonymization pipline.

## Running The Anonymization Pipeline

### Starting the Docker Container

### Running the Anonymization

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
