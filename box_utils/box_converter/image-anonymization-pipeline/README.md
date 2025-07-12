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

- run:
    for the `--cam` argument you can provide either `hdr`, `alphasense` or `zed2i`.

    ```bash
    docker run --rm -it --gpus all \
    --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" \
    grand-tour-dataset \
    python anonymization.py --mission-id 344d631c-387a-4aa4-bb21-92b9659ea21f --cam hdr
    ```

- singularity to cluster
    ```bash
    cd /home/jonfrey/git/grand_tour_box/box_utils/box_converter/image-anonymization-pipeline; docker build -t grand-tour-dataset . ; \
    cd ~/git/grand_tour_box/box_utils/box_converter/cluster/.export; rm -fr ./grand-tour-dataset.*; \
    SINGULARITY_NOHTTPS=1 singularity build --sandbox grand-tour-dataset.sif docker-daemon://grand-tour-dataset:latest; \
    sudo tar -cvf grand-tour-dataset.tar grand-tour-dataset.sif; \
    scp ./grand-tour-dataset.tar jonfrey@euler.ethz.ch:/cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset.tar
    ```

- change singularity image on cluster workflow
    ```
    mv /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset.tar  /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset-old.tar;
    
    rm -r $TMPDIR$/grand-tour-dataset.sif/data;
    cd $TMPDIR; tar -cvf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset.tar grand-tour-dataset.sif;

    ```

- debug for cluster
    ```bash
    srun --account=es_hutter --ntasks=1 --cpus-per-task=4 --gpus=1 --time=4:00:00 --mem-per-cpu=8024 --tmp=20000 --pty bash

    # Copy container
    tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-dataset.tar  -C $TMPDIR

    # Run test script 
    apptainer exec --nv --writable --bind /cluster/scratch/jonfrey:/scratch --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" --containall $TMPDIR/grand-tour-dataset.sif /bin/bash -c 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH; export HOME=/home; /entrypoint.sh  python /app/anonymization.py --mission-id d7525079-5564-461e-a144-e7479247d268 --cam hdr --head 200'

    # WARNING: Cluster requires different endpoint then local pc. Therefore copying over ~/.kleinkram.json wont work.
    #          Instead run container and login using kleinkram login via CLI and copy manual. It seems the prod_no_proxy works just fine.

    ```

- cluster ~/.kleinkram.json
    ```json
    {"version": "0.43.6", "endpoints": {"local": {"name": "local", "api": "http://localhost:3000", "s3": "http://localhost:9000"}, "dev": {"name": "dev", "api": "https://api.datasets.dev.leggedrobotics.com", "s3": "https://minio.datasets.dev.leggedrobotics.com"}, "prod": {"name": "prod", "api": "https://api.datasets.leggedrobotics.com", "s3": "https://minio.datasets.leggedrobotics.com"}}, "endpoint_credentials": {"prod": {"auth_token": "something", "refresh_token": "something", "api_key": null}}, "selected_endpoint": "prod"}
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
